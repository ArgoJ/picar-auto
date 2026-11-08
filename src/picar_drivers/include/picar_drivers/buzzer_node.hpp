#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"

namespace picar_drivers {

class BuzzerNode : public rclcpp::Node {
public:
    BuzzerNode();
    ~BuzzerNode() override;

    void playTone(float freq_hz);
    void stopTone();
    void beep(int duration_ms, float freq_hz);

private:
    void initHardware();

    void onBuzzerCmd(const std_msgs::msg::Bool::SharedPtr msg);
    void onBuzzerFreq(const std_msgs::msg::Int32::SharedPtr msg);
    void onBuzzerBeep(const std_msgs::msg::Int32::SharedPtr msg);

    int gpio_chip_{0};
    int pin_buzzer_{18};
    float default_freq_{1000.0f};

    int lg_handle_{-1};
    bool is_playing_{false};

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_cmd_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_freq_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_beep_;
    rclcpp::TimerBase::SharedPtr beep_timer_;
};

}  // namespace picar_drivers
