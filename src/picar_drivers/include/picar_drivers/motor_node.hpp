#pragma once

#include <memory>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "picar_drivers/pca9685.hpp"

namespace picar_drivers {

/**
 * @brief Hardware-Treiber fuer die beiden DC-Antriebsmotoren ueber PCA9685.
 */
class MotorNode : public rclcpp::Node {
public:
    explicit MotorNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~MotorNode() override;

    void emergencyStop();

private:
    void declareParameters();
    void initHardware();
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void stopCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void watchdogCallback();
    void applyDrive(double throttle);

    std::unique_ptr<PCA9685> pca_;

    // ROS 2 Subscriptions & Publishers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_picar_cmd_vel_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_stop_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_status_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;

    // Hardware- & Schnittstellen-Parameter
    int bus_number_{1};
    int pca9685_address_{0x5F};
    double pwm_frequency_{50.0};

    // DC-Antriebsmotoren
    int motor1_in1_{15};
    int motor1_in2_{14};
    int motor2_in1_{12};
    int motor2_in2_{13};
    bool invert_motor1_{false};
    bool invert_motor2_{false};

    // Regelung & Watchdog
    double max_linear_speed_{1.0};
    double cmd_vel_timeout_{0.5};
    rclcpp::Time last_cmd_vel_time_{0, 0, RCL_ROS_TIME};
    bool emergency_stopped_{false};

    double current_throttle_{0.0};
};

}  // namespace picar_drivers
