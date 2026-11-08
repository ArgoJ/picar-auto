#pragma once

#include <memory>
#include <string>
#include <gpiod.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"

namespace picar_drivers {

class LedNode : public rclcpp::Node {
public:
    LedNode();
    ~LedNode() override;

    void setLed(int index, bool state);
    void setAll(bool state);
    void setMask(int mask);

private:
    void initHardware();
    void publishStatus();

    void onLedCmd(const std_msgs::msg::Int32::SharedPtr msg);
    void onLed1Cmd(const std_msgs::msg::Bool::SharedPtr msg);
    void onLed2Cmd(const std_msgs::msg::Bool::SharedPtr msg);
    void onLed3Cmd(const std_msgs::msg::Bool::SharedPtr msg);
    void onAllCmd(const std_msgs::msg::Bool::SharedPtr msg);

    std::string chip_path_{"/dev/gpiochip0"};
    int pin_led1_{9};
    int pin_led2_{25};
    int pin_led3_{11};

    bool led1_state_{false};
    bool led2_state_{false};
    bool led3_state_{false};

    std::unique_ptr<::gpiod::chip> chip_;
    std::unique_ptr<::gpiod::line_request> request_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_cmd_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_led1_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_led2_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_led3_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_all_;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_status_;
};

}  // namespace picar_drivers
