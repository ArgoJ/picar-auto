#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace picar_drivers {

class OledNode : public rclcpp::Node {
public:
    OledNode();
    ~OledNode() override = default;

private:
    void initHardware();
    void textCallback(const std_msgs::msg::String::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_text_;

    int bus_number_{1};
    int i2c_address_{0x3C};
};

}  // namespace picar_drivers

