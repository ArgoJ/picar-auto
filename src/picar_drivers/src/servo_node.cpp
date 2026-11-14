#include "picar_drivers/servo_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <thread>

static std::atomic<int> g_servo_signal_count{0};

namespace picar_drivers {

ServoNode::ServoNode(const rclcpp::NodeOptions &options)
    : Node("servo_node", options)
{
    current_angles_.fill(90.0);
    has_angle_.fill(false);

    servo_slots_ = {
        {"lenkung",  "channel_lenkung",  "picar/servo/lenkung",  0},
        {"rotation", "channel_rotation", "picar/servo/rotation", 1},
        {"shoulder", "channel_shoulder", "picar/servo/shoulder", 2},
        {"elbow",    "channel_elbow",    "picar/servo/elbow",    3},
        {"gripper",  "channel_gripper",  "picar/servo/gripper",  4},
    };
    for (size_t i = 0; i < servo_slots_.size(); ++i) {
        slot_index_by_name_[servo_slots_[i].name] = i;
    }

    declareParameters();
    loadParameters();
    initHardware();

    // Dynamischer Parameter-Callback fuer Live-Kalibrierung und Kanal-Neuzuordnung
    param_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&ServoNode::onParametersChange, this, std::placeholders::_1)
    );

    // Ein eigenes Topic pro benanntem Servo (std_msgs/Float32 = Zielwinkel in Grad)
    for (size_t i = 0; i < servo_slots_.size(); ++i) {
        const std::string topic = servo_slots_[i].topic_name;
        const std::string servo_name = servo_slots_[i].name;
        auto sub = create_subscription<std_msgs::msg::Float32>(
            topic, 10,
            [this, servo_name](const std_msgs::msg::Float32::SharedPtr msg) {
                const int ch = getChannelForServo(servo_name);
                if (servo_name == "lenkung") {
                    current_steering_angle_ = static_cast<double>(msg->data);
                }
                setServoAngleClamped(ch, static_cast<double>(msg->data));
            }
        );
        sub_named_servos_.push_back(sub);
    }

    // Generisches Servo-Topic fuer Kalibrierungstools: data = [channel, angle_deg]
    sub_servo_cmd_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        "picar/servo/cmd", 10,
        std::bind(&ServoNode::servoCmdCallback, this, std::placeholders::_1)
    );

    // Kanal stromlos schalten
    sub_servo_release_ = create_subscription<std_msgs::msg::Int32>(
        "picar/servo/release", 10,
        std::bind(&ServoNode::servoReleaseCallback, this, std::placeholders::_1)
    );

    // Optionale Lenkungssteuerung ueber cmd_vel (angular.z)
    if (subscribe_cmd_vel_) {
        sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&ServoNode::cmdVelCallback, this, std::placeholders::_1)
        );
        sub_picar_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(
            "picar/motor/cmd_vel", 10,
            std::bind(&ServoNode::cmdVelCallback, this, std::placeholders::_1)
        );
    }

    // Status-Publisher: [channel, target_clamped, actual_hardware_angle]
    pub_servo_status_ = create_publisher<std_msgs::msg::Float32MultiArray>("picar/servo/status", 10);

    // Haelt servo_node beim Shutdown kurz aktiv, solange arm_handler_node noch in Ruheposition faehrt
    shutdown_timer_ = create_wall_timer(
        std::chrono::milliseconds(25),
        std::bind(&ServoNode::onShutdownTimer, this)
    );

    RCLCPP_INFO(
        get_logger(),
        "ServoNode aktiv (PCA9685: 0x%02X | lenkung=CH%d, rotation=CH%d, shoulder=CH%d, elbow=CH%d, gripper=CH%d)",
        pca9685_address_,
        getChannelForServo("lenkung"),
        getChannelForServo("rotation"),
        getChannelForServo("shoulder"),
        getChannelForServo("elbow"),
        getChannelForServo("gripper")
    );
}

ServoNode::~ServoNode()
{
    try {
        if (!shutdown_handled_ && release_on_shutdown_) {
            releaseAllServos();
        }
    } catch (...) {
    }
}

void ServoNode::declareParameters()
{
    declare_parameter<int>("bus_number", 1);
    declare_parameter<int>("pca9685_address", 0x5F);
    declare_parameter<double>("pwm_frequency", 50.0);
    declare_parameter<double>("servo_min_pulse_us", 500.0);
    declare_parameter<double>("servo_max_pulse_us", 2400.0);

    declare_parameter<bool>("initialize_on_start", true);
    declare_parameter<bool>("move_to_defaults_on_shutdown", true);
    declare_parameter<bool>("release_on_shutdown", true);
    declare_parameter<double>("shutdown_move_duration_s", 0.8);
    for (const auto &slot : servo_slots_) {
        declare_parameter<int>(slot.param_name, slot.channel);
    }

    declare_parameter<bool>("subscribe_cmd_vel", true);
    declare_parameter<double>("steering_center_angle", 90.0);
    declare_parameter<double>("steering_gain", 30.0);
    declare_parameter<bool>("invert_steering", false);

    std::vector<double> def_offsets(16, 0.0);
    std::vector<double> def_mins = {
        60.0, 10.0, 30.0, 30.0, 50.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    };
    std::vector<double> def_maxs = {
        120.0, 170.0, 170.0, 170.0, 130.0, 180.0, 180.0, 180.0,
        180.0, 180.0, 180.0, 180.0, 180.0, 180.0, 180.0, 180.0
    };
    std::vector<double> def_defaults = {
        90.0, 90.0, 60.0, 170.0, 67.0, 90.0, 90.0, 90.0,
        90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0
    };

    declare_parameter<std::vector<double>>("servo_offsets", def_offsets);
    declare_parameter<std::vector<double>>("servo_mins", def_mins);
    declare_parameter<std::vector<double>>("servo_maxs", def_maxs);
    declare_parameter<std::vector<double>>("servo_defaults", def_defaults);
}

void ServoNode::loadParameters()
{
    bus_number_            = get_parameter("bus_number").as_int();
    pca9685_address_       = get_parameter("pca9685_address").as_int();
    pwm_frequency_         = get_parameter("pwm_frequency").as_double();
    servo_min_pulse_us_    = get_parameter("servo_min_pulse_us").as_double();
    servo_max_pulse_us_    = get_parameter("servo_max_pulse_us").as_double();

    initialize_on_start_          = get_parameter("initialize_on_start").as_bool();
    move_to_defaults_on_shutdown_ = get_parameter("move_to_defaults_on_shutdown").as_bool();
    release_on_shutdown_          = get_parameter("release_on_shutdown").as_bool();
    shutdown_move_duration_s_     = std::max(0.0, get_parameter("shutdown_move_duration_s").as_double());
    for (auto &slot : servo_slots_) {
        int ch = get_parameter(slot.param_name).as_int();
        if (ch < 0 || ch >= 16) {
            RCLCPP_WARN(get_logger(),
                        "Ungueltiger Kanal %d fuer '%s', behalte Standardkanal %d.",
                        ch, slot.param_name.c_str(), slot.channel);
        } else {
            slot.channel = ch;
        }
    }

    subscribe_cmd_vel_     = get_parameter("subscribe_cmd_vel").as_bool();
    steering_center_angle_ = get_parameter("steering_center_angle").as_double();
    steering_gain_         = get_parameter("steering_gain").as_double();
    invert_steering_       = get_parameter("invert_steering").as_bool();
    current_steering_angle_ = steering_center_angle_;

    servo_offsets_         = get_parameter("servo_offsets").as_double_array();
    servo_mins_            = get_parameter("servo_mins").as_double_array();
    servo_maxs_            = get_parameter("servo_maxs").as_double_array();
    servo_defaults_        = get_parameter("servo_defaults").as_double_array();

    if (servo_offsets_.size() < 16)  servo_offsets_.resize(16, 0.0);
    if (servo_mins_.size() < 16)     servo_mins_.resize(16, 0.0);
    if (servo_maxs_.size() < 16)     servo_maxs_.resize(16, 180.0);
    if (servo_defaults_.size() < 16) servo_defaults_.resize(16, 90.0);
}

void ServoNode::initHardware()
{
    try {
        pca_ = std::make_unique<PCA9685>(bus_number_, pca9685_address_);
        pca_->init(pwm_frequency_);

        if (initialize_on_start_) {
            for (const auto &slot : servo_slots_) {
                setServoAngleClamped(slot.channel, servo_defaults_[slot.channel]);
            }
            const int steering_ch = getChannelForServo("lenkung");
            current_steering_angle_ = servo_defaults_[steering_ch];
            RCLCPP_INFO(
                get_logger(),
                "Startpositionen gesetzt: lenkung=%.1f°, rotation=%.1f°, shoulder=%.1f°, elbow=%.1f°, gripper=%.1f°",
                servo_defaults_[servo_slots_[0].channel],
                servo_defaults_[servo_slots_[1].channel],
                servo_defaults_[servo_slots_[2].channel],
                servo_defaults_[servo_slots_[3].channel],
                servo_defaults_[servo_slots_[4].channel]
            );
        } else {
            // Mindestens Lenkung in Mittelstellung bringen
            setServoAngleClamped(getChannelForServo("lenkung"), steering_center_angle_);
        }
    } catch (const std::exception &e) {
        RCLCPP_FATAL(get_logger(), "PCA9685 Initialisierung fehlgeschlagen: %s", e.what());
        throw;
    }
}

int ServoNode::getChannelForServo(const std::string &name) const
{
    auto it = slot_index_by_name_.find(name);
    if (it == slot_index_by_name_.end()) {
        return -1;
    }
    return servo_slots_[it->second].channel;
}

rcl_interfaces::msg::SetParametersResult ServoNode::onParametersChange(
    const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto &param : parameters) {
        const auto &pname = param.get_name();
        if (pname == "servo_offsets") {
            auto arr = param.as_double_array();
            if (arr.size() == 16) {
                servo_offsets_ = arr;
                RCLCPP_INFO(get_logger(), "Dynamische Kalibrierung: servo_offsets aktualisiert.");
                setServoAngleClamped(getChannelForServo("lenkung"), current_steering_angle_);
            } else {
                result.successful = false;
                result.reason = "servo_offsets muss exakt 16 Elemente lang sein.";
            }
        } else if (pname == "servo_mins") {
            auto arr = param.as_double_array();
            if (arr.size() == 16) {
                servo_mins_ = arr;
                RCLCPP_INFO(get_logger(), "Dynamische Kalibrierung: servo_mins aktualisiert.");
            } else {
                result.successful = false;
                result.reason = "servo_mins muss exakt 16 Elemente lang sein.";
            }
        } else if (pname == "servo_maxs") {
            auto arr = param.as_double_array();
            if (arr.size() == 16) {
                servo_maxs_ = arr;
                RCLCPP_INFO(get_logger(), "Dynamische Kalibrierung: servo_maxs aktualisiert.");
            } else {
                result.successful = false;
                result.reason = "servo_maxs muss exakt 16 Elemente lang sein.";
            }
        } else if (pname == "servo_defaults") {
            auto arr = param.as_double_array();
            if (arr.size() == 16) {
                servo_defaults_ = arr;
                RCLCPP_INFO(get_logger(), "servo_defaults aktualisiert.");
            } else {
                result.successful = false;
                result.reason = "servo_defaults muss exakt 16 Elemente lang sein.";
            }
        } else if (pname == "steering_center_angle") {
            steering_center_angle_ = param.as_double();
        } else if (pname == "steering_gain") {
            steering_gain_ = param.as_double();
        } else if (pname == "invert_steering") {
            invert_steering_ = param.as_bool();
        } else if (pname == "servo_min_pulse_us") {
            servo_min_pulse_us_ = param.as_double();
        } else if (pname == "servo_max_pulse_us") {
            servo_max_pulse_us_ = param.as_double();
        } else if (pname == "move_to_defaults_on_shutdown") {
            move_to_defaults_on_shutdown_ = param.as_bool();
        } else if (pname == "release_on_shutdown") {
            release_on_shutdown_ = param.as_bool();
        } else if (pname == "shutdown_move_duration_s") {
            const double val = param.as_double();
            if (val < 0.0) {
                result.successful = false;
                result.reason = "shutdown_move_duration_s muss >= 0.0 sein.";
            } else {
                shutdown_move_duration_s_ = val;
            }
        } else {
            // Pruefen, ob einer der Servo-Kanaele geaendert wurde
            for (auto &slot : servo_slots_) {
                if (pname == slot.param_name) {
                    int new_ch = param.as_int();
                    if (new_ch < 0 || new_ch >= 16) {
                        result.successful = false;
                        result.reason = pname + " muss zwischen 0 und 15 liegen.";
                    } else {
                        slot.channel = new_ch;
                        RCLCPP_INFO(get_logger(), "Kanal fuer '%s' auf CH%d geaendert.",
                                    slot.name.c_str(), new_ch);
                    }
                    break;
                }
            }
        }
    }

    return result;
}

void ServoNode::setServoAngleClamped(int channel, double target_angle)
{
    if (!pca_ || channel < 0 || channel >= 16) {
        return;
    }

    double min_angle = servo_mins_[channel];
    double max_angle = servo_maxs_[channel];
    double offset    = servo_offsets_[channel];

    double clamped_target = std::clamp(target_angle, min_angle, max_angle);
    current_angles_[channel] = clamped_target;
    has_angle_[channel]      = true;

    double actual_angle   = std::clamp(clamped_target + offset, 0.0, 180.0);

    pca_->setServoAngle(channel, actual_angle, servo_min_pulse_us_, servo_max_pulse_us_, 180.0);

    if (pub_servo_status_ && rclcpp::ok()) {
        std_msgs::msg::Float32MultiArray status_msg;
        status_msg.data = {
            static_cast<float>(channel),
            static_cast<float>(clamped_target),
            static_cast<float>(actual_angle)
        };
        pub_servo_status_->publish(status_msg);
    }
}

void ServoNode::releaseServo(int channel)
{
    if (!pca_) {
        return;
    }

    if (channel == -1) {
        releaseAllServos();
        return;
    }

    if (channel < 0 || channel >= 16) {
        return;
    }
    pca_->setChannelFullOff(channel);
}

void ServoNode::releaseAllServos()
{
    if (!pca_) {
        return;
    }
    for (const auto &slot : servo_slots_) {
        if (slot.channel >= 0 && slot.channel < 16) {
            pca_->setChannelFullOff(slot.channel);
        }
    }
}

bool ServoNode::hasActiveArmPublisher() const
{
    for (size_t i = 0; i < sub_named_servos_.size() && i < servo_slots_.size(); ++i) {
        const auto &name = servo_slots_[i].name;
        if (name != "lenkung" && sub_named_servos_[i] &&
            sub_named_servos_[i]->get_publisher_count() > 0)
        {
            return true;
        }
    }
    return false;
}

void ServoNode::onShutdownTimer()
{
    if (g_servo_signal_count.load() == 0) {
        return;
    }

    const auto now = get_clock()->now();
    if (shutdown_signal_time_.nanoseconds() == 0) {
        shutdown_signal_time_ = now;
        arm_active_at_shutdown_ = hasActiveArmPublisher();
    }

    const bool arm_busy = hasActiveArmPublisher();
    const bool timed_out = (now - shutdown_signal_time_).seconds() >= 4.5;
    if (!arm_active_at_shutdown_ || !arm_busy || timed_out) {
        if (timed_out && arm_busy) {
            RCLCPP_WARN(get_logger(), "Wartezeit auf arm_handler_node beim Shutdown abgelaufen.");
        }
        shutdown_timer_->cancel();
        rclcpp::shutdown();
    }
}

void ServoNode::shutdownServos(const std::atomic<int> *signal_count, int initial_signal_count)
{
    if (shutdown_handled_) {
        return;
    }
    shutdown_handled_ = true;

    if (!pca_) {
        return;
    }

    const auto force_abort = [&]() {
        return initial_signal_count >= 2 ||
               (signal_count != nullptr && signal_count->load() > initial_signal_count);
    };

    if (move_to_defaults_on_shutdown_ && !force_abort()) {
        std::array<double, 16> start_angles{};
        double max_delta = 0.0;
        bool any_unknown = false;

        for (const auto &slot : servo_slots_) {
            if (arm_active_at_shutdown_ && slot.name != "lenkung") {
                continue;
            }
            const int ch = slot.channel;
            if (ch < 0 || ch >= 16) {
                continue;
            }
            const double target = std::clamp(servo_defaults_[ch], servo_mins_[ch], servo_maxs_[ch]);
            if (has_angle_[ch]) {
                start_angles[ch] = current_angles_[ch];
                max_delta = std::max(max_delta, std::abs(target - start_angles[ch]));
            } else {
                start_angles[ch] = target;
                any_unknown = true;
            }
        }

        if (any_unknown || max_delta >= 0.5) {
            double duration = shutdown_move_duration_s_;
            if (!any_unknown && duration > 0.0) {
                duration = std::clamp(duration * std::min(1.0, max_delta / 45.0), 0.15, duration);
            }

            const int steps = (duration > 0.0)
                ? std::max(1, static_cast<int>(std::round(duration / 0.02)))
                : 1;

            for (int step = 1; step <= steps; ++step) {
                if (force_abort()) {
                    break;
                }
                const double alpha = static_cast<double>(step) / static_cast<double>(steps);
                for (const auto &slot : servo_slots_) {
                    if (arm_active_at_shutdown_ && slot.name != "lenkung") {
                        continue;
                    }
                    const int ch = slot.channel;
                    if (ch < 0 || ch >= 16) {
                        continue;
                    }
                    const double target = std::clamp(
                        servo_defaults_[ch], servo_mins_[ch], servo_maxs_[ch]);
                    const double angle = has_angle_[ch]
                        ? (start_angles[ch] + alpha * (target - start_angles[ch]))
                        : target;
                    setServoAngleClamped(ch, angle);
                }
                if (duration > 0.0) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                }
            }

            if (!force_abort() && duration > 0.0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }

    if (release_on_shutdown_) {
        releaseAllServos();
    }
}

void ServoNode::servoCmdCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (msg->data.size() < 2) {
        RCLCPP_WARN(get_logger(), "servoCmdCallback benoetigt mind. 2 Elemente: [kanal, winkel]");
        return;
    }

    int channel  = static_cast<int>(msg->data[0]);
    double angle = static_cast<double>(msg->data[1]);
    setServoAngleClamped(channel, angle);
}

void ServoNode::servoReleaseCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    releaseServo(msg->data);
}

void ServoNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    double dir_mult = invert_steering_ ? -1.0 : 1.0;
    double steering_delta = msg->angular.z * steering_gain_ * dir_mult;
    current_steering_angle_ = steering_center_angle_ + steering_delta;

    setServoAngleClamped(getChannelForServo("lenkung"), current_steering_angle_);
}

}  // namespace picar_drivers

static std::shared_ptr<picar_drivers::ServoNode> g_servo_node = nullptr;

void servoNodeSignalHandler(int sig)
{
    (void)sig;
    const int prev = g_servo_signal_count.fetch_add(1);
    if (prev >= 1) {
        rclcpp::shutdown();
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::signal(SIGINT, servoNodeSignalHandler);
    std::signal(SIGTERM, servoNodeSignalHandler);

    try {
        g_servo_node = std::make_shared<picar_drivers::ServoNode>();
        rclcpp::spin(g_servo_node);
    } catch (const std::exception &e) {
        std::cerr << "ServoNode Fehler: " << e.what() << std::endl;
    }

    if (g_servo_node) {
        const int sig_at_exit = g_servo_signal_count.load();
        g_servo_node->shutdownServos(&g_servo_signal_count, sig_at_exit);
        g_servo_node.reset();
    }

    rclcpp::shutdown();
    return 0;
}
