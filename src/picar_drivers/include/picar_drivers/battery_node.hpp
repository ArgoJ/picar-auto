#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/bool.hpp>

#include "picar_drivers/ads7830_sensor.hpp"

namespace picar_drivers {

/**
 * @brief ROS 2 node monitoring battery voltage via the ADS7830 ADC (A0) on the Adeept Robot HAT V3.2.
 */
class BatteryNode : public rclcpp::Node {
public:
    explicit BatteryNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~BatteryNode() override = default;

private:
    void initParameters();
    void timerCallback();

    std::unique_ptr<ADS7830Sensor> sensor_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_battery_state_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_low_alarm_;
    rclcpp::TimerBase::SharedPtr timer_;

    int bus_number_{1};
    int device_address_{0x48};
    int adc_channel_{0};
    double rate_hz_{2.0};
    std::string frame_id_{"battery_link"};

    double adc_ref_voltage_{5.0};
    double division_ratio_{0.25};
    double full_voltage_{8.4};            // 2S 18650 full charge [V]
    double empty_voltage_{6.75};          // 0% cutoff reference [V]
    double low_battery_percentage_{20.0}; // Low battery alarm threshold [%]
};

}  // namespace picar_drivers
