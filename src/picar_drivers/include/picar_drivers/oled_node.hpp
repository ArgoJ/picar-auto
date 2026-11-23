#pragma once

#include <array>
#include <memory>
#include <random>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "picar_drivers/ssd1306_display.hpp"

namespace picar_drivers {

/**
 * @brief ROS 2 node driving the 128x64 monochrome SSD1306 OLED display.
 *
 * Defaults to animated robot eyes with random gaze and blinking, renders custom text on
 * `picar/oled_text`, and displays a blinking low-battery icon on `picar/battery/low_alarm`.
 */
class OledNode : public rclcpp::Node {
public:
    explicit OledNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~OledNode() override;

private:
    enum class Mode { EYES, TEXT };

    void onText(const std_msgs::msg::String::SharedPtr msg);
    void onLowBattery(const std_msgs::msg::Bool::SharedPtr msg);
    void updateFrame();
    void renderEyesFrame();
    void renderLowBatteryFrame(bool visible);

    std::unique_ptr<SSD1306Display> display_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_text_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_low_battery_;
    rclcpp::TimerBase::SharedPtr timer_;

    double fps_{25.0};
    double text_timeout_s_{0.0};
    Mode mode_{Mode::EYES};
    std::string last_text_;
    rclcpp::Time text_time_{0, 0, RCL_ROS_TIME};

    bool low_battery_{false};
    bool low_blink_on_{false};
    int low_blink_rendered_{-1};
    rclcpp::Time next_low_blink_{0, 0, RCL_ROS_TIME};

    double gaze_x_{0.0};
    double gaze_y_{0.0};
    int target_dx_{0};
    int target_dy_{0};
    bool blinking_{false};
    size_t blink_step_{0};
    rclcpp::Time next_gaze_{0, 0, RCL_ROS_TIME};
    rclcpp::Time next_blink_{0, 0, RCL_ROS_TIME};
    std::array<int, 4> last_eye_state_{-999, -999, -999, -999};
    std::mt19937 rng_;
};

}  // namespace picar_drivers
