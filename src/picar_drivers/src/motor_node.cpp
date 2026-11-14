#include "picar_drivers/motor_node.hpp"

#include <algorithm>
#include <cmath>
#include <csignal>
#include <iostream>

namespace picar_drivers {

MotorNode::MotorNode(const rclcpp::NodeOptions &options)
    : Node("motor_node", options)
{
    declareParameters();

    bus_number_       = get_parameter("bus_number").as_int();
    pca9685_address_  = get_parameter("pca9685_address").as_int();
    pwm_frequency_    = get_parameter("pwm_frequency").as_double();

    motor1_in1_       = get_parameter("motor1_in1").as_int();
    motor1_in2_       = get_parameter("motor1_in2").as_int();
    motor2_in1_       = get_parameter("motor2_in1").as_int();
    motor2_in2_       = get_parameter("motor2_in2").as_int();
    invert_motor1_    = get_parameter("invert_motor1").as_bool();
    invert_motor2_    = get_parameter("invert_motor2").as_bool();

    max_linear_speed_ = get_parameter("max_linear_speed").as_double();
    cmd_vel_timeout_  = get_parameter("cmd_vel_timeout").as_double();

    if (max_linear_speed_ <= 0.0) {
        max_linear_speed_ = 1.0;
    }

    initHardware();

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

    // Feedback-Publisher
    pub_status_ = create_publisher<geometry_msgs::msg::Twist>("picar/motor/status", 10);

    // Watchdog-Timer (20 Hz)
    watchdog_timer_ = create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&MotorNode::watchdogCallback, this)
    );

    last_cmd_vel_time_ = now();

    RCLCPP_INFO(get_logger(),
                "MotorNode aktiv (PCA9685: 0x%02X, DC-Motoren M1=[%d,%d], M2=[%d,%d])",
                pca9685_address_, motor1_in1_, motor1_in2_, motor2_in1_, motor2_in2_);
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

        // DC-Motoren beim Start sicher auf 0 setzen
        pca_->setMotorThrottle(motor1_in1_, motor1_in2_, 0.0, true);
        pca_->setMotorThrottle(motor2_in1_, motor2_in2_, 0.0, true);
    } catch (const std::exception &e) {
        RCLCPP_FATAL(get_logger(), "PCA9685 Initialisierung fehlgeschlagen: %s", e.what());
        throw;
    }
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
    applyDrive(throttle);
}

void MotorNode::applyDrive(double throttle)
{
    if (!pca_) {
        return;
    }

    current_throttle_ = throttle;

    double t1 = current_throttle_ * (invert_motor1_ ? -1.0 : 1.0);
    double t2 = current_throttle_ * (invert_motor2_ ? -1.0 : 1.0);

    pca_->setMotorThrottle(motor1_in1_, motor1_in2_, t1, true);
    pca_->setMotorThrottle(motor2_in1_, motor2_in2_, t2, true);

    if (pub_status_) {
        geometry_msgs::msg::Twist status_msg;
        status_msg.linear.x = current_throttle_;
        pub_status_->publish(status_msg);
    }
}

void MotorNode::emergencyStop()
{
    current_throttle_ = 0.0;

    if (pca_) {
        pca_->setMotorThrottle(motor1_in1_, motor1_in2_, 0.0, true);
        pca_->setMotorThrottle(motor2_in1_, motor2_in2_, 0.0, true);
    }

    if (pub_status_) {
        geometry_msgs::msg::Twist status_msg;
        status_msg.linear.x = 0.0;
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
        applyDrive(0.0);
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
