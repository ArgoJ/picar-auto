#include "picar_drivers/motor_node.hpp"

#include <algorithm>
#include <csignal>
#include <iostream>

namespace picar_drivers {

MotorNode::MotorNode(const rclcpp::NodeOptions &options)
    : Node("motor_node", options)
{
    declareParameters();

    bus_number_             = get_parameter("bus_number").as_int();
    pca9685_address_        = get_parameter("pca9685_address").as_int();
    pwm_frequency_          = get_parameter("pwm_frequency").as_double();

    steering_channel_       = get_parameter("steering_channel").as_int();
    steering_center_angle_  = get_parameter("steering_center_angle").as_double();
    steering_min_pulse_us_  = get_parameter("steering_min_pulse_us").as_double();
    steering_max_pulse_us_  = get_parameter("steering_max_pulse_us").as_double();
    steering_gain_          = get_parameter("steering_gain").as_double();
    invert_steering_        = get_parameter("invert_steering").as_bool();

    servo_offsets_          = get_parameter("servo_offsets").as_double_array();
    servo_mins_             = get_parameter("servo_mins").as_double_array();
    servo_maxs_             = get_parameter("servo_maxs").as_double_array();

    // Sicherstellen, dass die Arrays 16 Elemente besitzen
    if (servo_offsets_.size() < 16) servo_offsets_.resize(16, 0.0);
    if (servo_mins_.size() < 16)    servo_mins_.resize(16, 0.0);
    if (servo_maxs_.size() < 16)    servo_maxs_.resize(16, 180.0);

    motor1_in1_             = get_parameter("motor1_in1").as_int();
    motor1_in2_             = get_parameter("motor1_in2").as_int();
    motor2_in1_             = get_parameter("motor2_in1").as_int();
    motor2_in2_             = get_parameter("motor2_in2").as_int();
    invert_motor1_          = get_parameter("invert_motor1").as_bool();
    invert_motor2_          = get_parameter("invert_motor2").as_bool();

    max_linear_speed_       = get_parameter("max_linear_speed").as_double();
    cmd_vel_timeout_        = get_parameter("cmd_vel_timeout").as_double();

    if (max_linear_speed_ <= 0.0) {
        max_linear_speed_ = 1.0;
    }

    initHardware();

    // Dynamischer Parameter-Callback fuer Live-Kalibrierung
    param_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&MotorNode::onParametersChange, this, std::placeholders::_1)
    );

    // Fahrbefehle (Twist)
    sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&MotorNode::cmdVelCallback, this, std::placeholders::_1)
    );

    sub_picar_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(
        "picar/motor/cmd_vel", 10,
        std::bind(&MotorNode::cmdVelCallback, this, std::placeholders::_1)
    );

    sub_stop_ = create_subscription<std_msgs::msg::Bool>(
        "picar/motor/stop", 10,
        std::bind(&MotorNode::stopCallback, this, std::placeholders::_1)
    );

    // Servo-Steuerung fuer alle Kanaele: data = [channel, angle_deg]
    sub_servo_cmd_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        "picar/servo/cmd", 10,
        std::bind(&MotorNode::servoCmdCallback, this, std::placeholders::_1)
    );

    // Spezielles Topic fuer den AD002 Greifer (CH4)
    sub_gripper_cmd_ = create_subscription<std_msgs::msg::Float32>(
        "picar/gripper/cmd", 10,
        std::bind(&MotorNode::gripperCmdCallback, this, std::placeholders::_1)
    );

    // Kanal stromlos schalten
    sub_servo_release_ = create_subscription<std_msgs::msg::Int32>(
        "picar/servo/release", 10,
        std::bind(&MotorNode::servoReleaseCallback, this, std::placeholders::_1)
    );

    // Feedback-Publisher
    pub_status_ = create_publisher<geometry_msgs::msg::Twist>("picar/motor/status", 10);

    // Watchdog-Timer (20 Hz)
    watchdog_timer_ = create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&MotorNode::watchdogCallback, this)
    );

    last_cmd_vel_time_ = now();
    current_steering_angle_ = steering_center_angle_;

    RCLCPP_INFO(get_logger(),
                "MotorNode aktiv (PCA9685: 0x%02X, 16 Servokanaele mit dynamischer Kalibrierung konfiguriert)",
                pca9685_address_);
}

MotorNode::~MotorNode()
{
    try {
        emergencyStop();
    } catch (...) {
    }
}

void MotorNode::declareParameters()
{
    declare_parameter<int>("bus_number", 1);
    declare_parameter<int>("pca9685_address", 0x5F);
    declare_parameter<double>("pwm_frequency", 50.0);

    declare_parameter<int>("steering_channel", 0);
    declare_parameter<double>("steering_center_angle", 90.0);
    declare_parameter<double>("steering_min_pulse_us", 500.0);
    declare_parameter<double>("steering_max_pulse_us", 2400.0);
    declare_parameter<double>("steering_gain", 30.0);
    declare_parameter<bool>("invert_steering", false);

    // 3-fache Standard-Arrays fuer alle 16 Kanaele
    std::vector<double> def_offsets(16, 0.0);
    std::vector<double> def_mins = {
        60.0, 10.0, 30.0, 30.0, 50.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    };
    std::vector<double> def_maxs = {
        120.0, 170.0, 150.0, 150.0, 130.0, 180.0, 180.0, 180.0,
        180.0, 180.0, 180.0, 180.0, 180.0, 180.0, 180.0, 180.0
    };

    declare_parameter<std::vector<double>>("servo_offsets", def_offsets);
    declare_parameter<std::vector<double>>("servo_mins", def_mins);
    declare_parameter<std::vector<double>>("servo_maxs", def_maxs);

    declare_parameter<int>("motor1_in1", 15);
    declare_parameter<int>("motor1_in2", 14);
    declare_parameter<int>("motor2_in1", 12);
    declare_parameter<int>("motor2_in2", 13);
    declare_parameter<bool>("invert_motor1", false);
    declare_parameter<bool>("invert_motor2", false);

    declare_parameter<double>("max_linear_speed", 1.0);
    declare_parameter<double>("cmd_vel_timeout", 0.5);
}

void MotorNode::initHardware()
{
    try {
        pca_ = std::make_unique<PCA9685>(bus_number_, pca9685_address_);
        pca_->init(pwm_frequency_);

        // Lenkung in Mittelstellung mit aktuellem Offset bringen
        setServoAngleClamped(steering_channel_, steering_center_angle_);
    } catch (const std::exception &e) {
        RCLCPP_FATAL(get_logger(), "PCA9685 Initialisierung fehlgeschlagen: %s", e.what());
        throw;
    }
}

rcl_interfaces::msg::SetParametersResult MotorNode::onParametersChange(
    const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto &param : parameters) {
        if (param.get_name() == "servo_offsets") {
            auto arr = param.as_double_array();
            if (arr.size() == 16) {
                servo_offsets_ = arr;
                RCLCPP_INFO(get_logger(), "Dynamische Kalibrierung: servo_offsets aktualisiert.");
                // Lenkung direkt anpassen
                setServoAngleClamped(steering_channel_, current_steering_angle_);
            } else {
                result.successful = false;
                result.reason = "servo_offsets muss exakt 16 Elemente lang sein.";
            }
        } else if (param.get_name() == "servo_mins") {
            auto arr = param.as_double_array();
            if (arr.size() == 16) {
                servo_mins_ = arr;
                RCLCPP_INFO(get_logger(), "Dynamische Kalibrierung: servo_mins aktualisiert.");
            } else {
                result.successful = false;
                result.reason = "servo_mins muss exakt 16 Elemente lang sein.";
            }
        } else if (param.get_name() == "servo_maxs") {
            auto arr = param.as_double_array();
            if (arr.size() == 16) {
                servo_maxs_ = arr;
                RCLCPP_INFO(get_logger(), "Dynamische Kalibrierung: servo_maxs aktualisiert.");
            } else {
                result.successful = false;
                result.reason = "servo_maxs muss exakt 16 Elemente lang sein.";
            }
        }
    }

    return result;
}

void MotorNode::setServoAngleClamped(int channel, double target_angle)
{
    if (!pca_ || channel < 0 || channel >= 16) {
        return;
    }

    // Sollwinkel-Begrenzung: min und max definieren den symmetrischen Steuerbereich (vor Offset).
    // Der Offset verschiebt den gesamten Bereich auf der physischen Hardware.
    double min_angle = servo_mins_[channel];
    double max_angle = servo_maxs_[channel];
    double offset    = servo_offsets_[channel];

    double clamped_target = std::clamp(target_angle, min_angle, max_angle);
    double actual_angle   = std::clamp(clamped_target + offset, 0.0, 180.0);

    pca_->setServoAngle(channel, actual_angle, steering_min_pulse_us_, steering_max_pulse_us_, 180.0);
}

void MotorNode::releaseServo(int channel)
{
    if (!pca_) {
        return;
    }

    if (channel == -1) {
        pca_->allOff();
        RCLCPP_INFO(get_logger(), "Alle 16 PWM-Kanaele (Servos & Motoren) stromlos geschaltet.");
        return;
    }

    if (channel < 0 || channel >= 16) {
        return;
    }
    pca_->setChannelFullOff(channel);
}

void MotorNode::servoCmdCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (msg->data.size() < 2) {
        RCLCPP_WARN(get_logger(), "servoCmdCallback benoetigt mind. 2 Elemente: [kanal, winkel]");
        return;
    }

    int channel = static_cast<int>(msg->data[0]);
    double angle = static_cast<double>(msg->data[1]);
    setServoAngleClamped(channel, angle);
}

void MotorNode::gripperCmdCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    // CH4 = AD002 Greifer
    setServoAngleClamped(4, static_cast<double>(msg->data));
}

void MotorNode::servoReleaseCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    releaseServo(msg->data);
}

void MotorNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    last_cmd_vel_time_ = now();

    if (emergency_stopped_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Notstopp aktiv! Sende picar/motor/stop: false zum Zuruecksetzen.");
        return;
    }

    double throttle = std::clamp(msg->linear.x / max_linear_speed_, -1.0, 1.0);

    double dir_mult = invert_steering_ ? -1.0 : 1.0;
    double steering_delta = msg->angular.z * steering_gain_ * dir_mult;
    double steering_angle = steering_center_angle_ + steering_delta;

    applyDrive(throttle, steering_angle);
}

void MotorNode::applyDrive(double throttle, double steering_angle_deg)
{
    if (!pca_) {
        return;
    }

    current_throttle_ = throttle;
    current_steering_angle_ = steering_angle_deg;

    // 1. Lenkservo (CH0) ueber gesichertes Clamping ansteuern
    setServoAngleClamped(steering_channel_, current_steering_angle_);

    // 2. DC-Antriebsmotoren
    double t1 = current_throttle_ * (invert_motor1_ ? -1.0 : 1.0);
    double t2 = current_throttle_ * (invert_motor2_ ? -1.0 : 1.0);

    pca_->setMotorThrottle(motor1_in1_, motor1_in2_, t1, true);
    pca_->setMotorThrottle(motor2_in1_, motor2_in2_, t2, true);

    // 3. Status rueckmelden
    if (pub_status_) {
        geometry_msgs::msg::Twist status_msg;
        status_msg.linear.x = current_throttle_;
        // Berechneten Hardware-Winkel zurueckmelden (Sollwinkel geklemmt + Offset)
        double clamped_target = std::clamp(
            current_steering_angle_,
            servo_mins_[steering_channel_],
            servo_maxs_[steering_channel_]
        );
        double actual_steering = std::clamp(
            clamped_target + servo_offsets_[steering_channel_],
            0.0,
            180.0
        );
        status_msg.angular.z = actual_steering;
        pub_status_->publish(status_msg);
    }
}

void MotorNode::emergencyStop()
{
    current_throttle_ = 0.0;
    current_steering_angle_ = steering_center_angle_;

    if (pca_) {
        pca_->setMotorThrottle(motor1_in1_, motor1_in2_, 0.0, true);
        pca_->setMotorThrottle(motor2_in1_, motor2_in2_, 0.0, true);
        pca_->allOff();
    }

    if (pub_status_) {
        geometry_msgs::msg::Twist status_msg;
        status_msg.linear.x = 0.0;
        status_msg.angular.z = steering_center_angle_;
        pub_status_->publish(status_msg);
    }
}

void MotorNode::stopCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data) {
        emergency_stopped_ = true;
        emergencyStop();
        RCLCPP_WARN(get_logger(), "Notstopp ausgeloest! Motoren gestoppt.");
    } else {
        emergency_stopped_ = false;
        RCLCPP_INFO(get_logger(), "Notstopp aufgehoben. Normalbetrieb bereit.");
    }
}

void MotorNode::watchdogCallback()
{
    if (cmd_vel_timeout_ <= 0.0 || emergency_stopped_) {
        return;
    }

    rclcpp::Time current_time = now();
    double elapsed = (current_time - last_cmd_vel_time_).seconds();

    if (elapsed > cmd_vel_timeout_ && std::abs(current_throttle_) > 0.001) {
        RCLCPP_WARN(get_logger(),
                    "cmd_vel Timeout (%.2fs ohne Nachricht). Sicherheits-Stopp der Motoren.", elapsed);
        applyDrive(0.0, current_steering_angle_);
    }
}

}  // namespace picar_drivers

static std::shared_ptr<picar_drivers::MotorNode> g_motor_node = nullptr;

void motorNodeSignalHandler(int sig)
{
    (void)sig;
    if (g_motor_node) {
        g_motor_node->emergencyStop();
    }
    rclcpp::shutdown();
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::signal(SIGINT, motorNodeSignalHandler);
    std::signal(SIGTERM, motorNodeSignalHandler);

    try {
        g_motor_node = std::make_shared<picar_drivers::MotorNode>();
        rclcpp::spin(g_motor_node);
    } catch (const std::exception &e) {
        std::cerr << "MotorNode Fehler: " << e.what() << std::endl;
    }

    if (g_motor_node) {
        g_motor_node->emergencyStop();
        g_motor_node.reset();
    }

    rclcpp::shutdown();
    return 0;
}
