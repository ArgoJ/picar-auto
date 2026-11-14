#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>

#include "picar_drivers/pca9685.hpp"

namespace picar_drivers {

/**
 * @brief Hardware-Treiber fuer alle Servos (Lenkung & Roboterarm) ueber PCA9685.
 *
 * Stellt fuer jeden benannten Servo (gripper, elbow, shoulder, rotation, lenkung)
 * ein eigenes Topic bereit und erlaubt die freie Kanal-Zuordnung per Parameter.
 */
class ServoNode : public rclcpp::Node {
public:
    explicit ServoNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~ServoNode() override;

    void setServoAngleClamped(int channel, double target_angle);
    void releaseServo(int channel);
    void releaseAllServos();
    void shutdownServos(const std::atomic<int> *signal_count = nullptr, int initial_signal_count = 0);

private:
    struct ServoSlot {
        std::string name;
        std::string param_name;
        std::string topic_name;
        int channel;
    };

    void declareParameters();
    void loadParameters();
    void initHardware();

    void servoCmdCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void servoReleaseCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void onShutdownTimer();
    bool hasActiveArmPublisher() const;

    rcl_interfaces::msg::SetParametersResult onParametersChange(
        const std::vector<rclcpp::Parameter> &parameters);

    int getChannelForServo(const std::string &name) const;

    std::unique_ptr<PCA9685> pca_;

    // Subscriptions & Publishers
    std::vector<rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr> sub_named_servos_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_servo_cmd_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_servo_release_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_picar_cmd_vel_;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_servo_status_;
    rclcpp::TimerBase::SharedPtr shutdown_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // Hardware-Parameter
    int bus_number_{1};
    int pca9685_address_{0x5F};
    double pwm_frequency_{50.0};
    double servo_min_pulse_us_{500.0};
    double servo_max_pulse_us_{2400.0};

    // Benannte Servos mit konfigurierbarem PCA9685-Kanal
    std::vector<ServoSlot> servo_slots_;
    std::unordered_map<std::string, size_t> slot_index_by_name_;

    // Lenkungs- & Shutdown-Verhalten
    bool initialize_on_start_{true};
    bool move_to_defaults_on_shutdown_{true};
    bool release_on_shutdown_{true};
    double shutdown_move_duration_s_{0.8};
    bool arm_active_at_shutdown_{false};
    rclcpp::Time shutdown_signal_time_{0, 0, RCL_ROS_TIME};
    bool shutdown_handled_{false};
    bool subscribe_cmd_vel_{true};
    double steering_center_angle_{90.0};
    double steering_gain_{30.0};
    bool invert_steering_{false};
    double current_steering_angle_{90.0};

    // Letzte kommandierte Sollwinkel pro Kanal (fuer sanftes Anfahren der Ruheposition)
    std::array<double, 16> current_angles_{};
    std::array<bool, 16> has_angle_{};

    // 4-fache Arrays fuer alle 16 Kanaele (CH0 bis CH15)
    std::vector<double> servo_offsets_;
    std::vector<double> servo_mins_;
    std::vector<double> servo_maxs_;
    std::vector<double> servo_defaults_;
};

}  // namespace picar_drivers
