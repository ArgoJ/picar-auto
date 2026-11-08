#pragma once

#include <memory>
#include <string>
#include <gpiod.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

namespace picar_drivers {

class UltrasonicNode : public rclcpp::Node {
public:
    UltrasonicNode();
    ~UltrasonicNode() override;

private:
    void initHardware();
    void timerCallback();
    double measureDistanceMeters();

    std::string chip_path_{"/dev/gpiochip0"};
    int pin_trig_{23};
    int pin_echo_{24};
    std::string frame_id_{"ultrasonic_link"};
    double rate_hz_{10.0};
    double min_range_m_{0.02};
    double max_range_m_{4.00};

    std::unique_ptr<::gpiod::chip> chip_;
    std::unique_ptr<::gpiod::line_request> request_;

    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_range_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace picar_drivers
