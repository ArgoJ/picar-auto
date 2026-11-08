#pragma once

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>

#include "picar_drivers/pca9685.hpp"

namespace picar_drivers {

class MotorNode : public rclcpp::Node {
public:
    explicit MotorNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~MotorNode() override;

    // Servo-Winkel mit Offset und Min/Max-Schutz ansteuern
    void setServoAngleClamped(int channel, double target_angle);
    void releaseServo(int channel);
    void emergencyStop();

private:
    void declareParameters();
    void initHardware();
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void stopCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void watchdogCallback();

    void servoCmdCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void gripperCmdCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void servoReleaseCallback(const std_msgs::msg::Int32::SharedPtr msg);

    rcl_interfaces::msg::SetParametersResult onParametersChange(
        const std::vector<rclcpp::Parameter> &parameters);

    void applyDrive(double throttle, double steering_angle_deg);

    std::unique_ptr<PCA9685> pca_;

    // ROS 2 Subscriptions & Publishers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_picar_cmd_vel_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_stop_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_servo_cmd_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_gripper_cmd_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_servo_release_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_status_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // Hardware- & Schnittstellen-Parameter
    int bus_number_{1};
    int pca9685_address_{0x5F};
    double pwm_frequency_{50.0};

    // 3-fache Arrays fuer alle 16 Kanaele (CH0 bis CH15)
    std::vector<double> servo_offsets_;
    std::vector<double> servo_mins_;
    std::vector<double> servo_maxs_;

    // Lenkservo (Kanal CH0)
    int steering_channel_{0};
    double steering_center_angle_{90.0};
    double steering_min_pulse_us_{500.0};
    double steering_max_pulse_us_{2400.0};
    double steering_gain_{30.0};
    bool invert_steering_{false};

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
    double current_steering_angle_{90.0};
};

}  // namespace picar_drivers
