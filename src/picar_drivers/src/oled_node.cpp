#include "picar_drivers/oled_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <utility>

namespace picar_drivers {

namespace {

constexpr std::array<std::pair<int, int>, 10> kGazeTargets = {{
    {0, 0},     // Center
    {0, 0},     // Center (weighted slightly higher)
    {-14, 0},   // Left
    {14, 0},    // Right
    {0, -10},   // Up
    {0, 10},    // Down
    {-12, -9},  // Up-Left
    {12, -9},   // Up-Right
    {-12, 9},   // Down-Left
    {12, 9}     // Down-Right
}};
constexpr std::array<double, 7> kBlinkProfile = {0.45, 0.12, 0.06, 0.16, 0.50, 0.85, 1.0};

double stepSaccade(double cur, int target)
{
    const double diff = static_cast<double>(target) - cur;
    return (std::abs(diff) < 0.35) ? static_cast<double>(target) : (cur + diff * 0.42);
}

}  // namespace

OledNode::OledNode(const rclcpp::NodeOptions &options)
    : Node("oled_node", options), rng_(std::random_device{}())
{
    const int bus   = declare_parameter<int>("bus_number", 1);
    const int addr  = declare_parameter<int>("i2c_address", 0x3C);
    const auto font = declare_parameter<std::string>("font_path", "/usr/share/consolefonts/Lat15-VGA8.psf.gz");
    fps_            = std::max(1.0, declare_parameter<double>("fps", fps_));
    text_timeout_s_ = declare_parameter<double>("text_timeout_s", text_timeout_s_);
    mode_           = (declare_parameter<std::string>("default_mode", "eyes") == "text") ? Mode::TEXT : Mode::EYES;

    try {
        display_ = std::make_unique<SSD1306Display>(bus, addr, font);
    } catch (const std::exception &e) {
        RCLCPP_FATAL(get_logger(), "Failed to initialize SSD1306 OLED: %s", e.what());
        throw;
    }

    sub_text_ = create_subscription<std_msgs::msg::String>(
        "picar/oled_text", 10, std::bind(&OledNode::onText, this, std::placeholders::_1));
    sub_low_battery_ = create_subscription<std_msgs::msg::Bool>(
        "picar/battery/low_alarm", 10, std::bind(&OledNode::onLowBattery, this, std::placeholders::_1));

    const rclcpp::Time now = get_clock()->now();
    next_gaze_ = now + rclcpp::Duration::from_seconds(0.8);
    next_blink_ = now + rclcpp::Duration::from_seconds(2.5);

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / fps_));
    timer_ = create_wall_timer(period, std::bind(&OledNode::updateFrame, this));

    RCLCPP_INFO(get_logger(), "OledNode active (bus=%d, addr=0x%02X, %.1f FPS)", bus, addr, fps_);
}

OledNode::~OledNode()
{
    if (display_) {
        try {
            display_->clear();
            display_->display();
        } catch (...) {
        }
    }
}

void OledNode::onText(const std_msgs::msg::String::SharedPtr msg)
{
    if (msg->data.empty() || msg->data == "eyes" || msg->data == "__eyes__") {
        mode_ = Mode::EYES;
        last_eye_state_.fill(-999);
        return;
    }

    mode_ = Mode::TEXT;
    last_text_ = msg->data;
    text_time_ = get_clock()->now();
    if (!low_battery_ && display_) {
        display_->clear();
        display_->drawText(2, 2, last_text_);
        display_->display();
    }
}

void OledNode::onLowBattery(const std_msgs::msg::Bool::SharedPtr msg)
{
    const bool active = msg->data;
    if (active == low_battery_) {
        return;
    }
    low_battery_ = active;
    if (low_battery_) {
        low_blink_on_ = true;
        low_blink_rendered_ = -1;
        next_low_blink_ = get_clock()->now() + rclcpp::Duration::from_seconds(0.45);
        return;
    }

    // Restore active mode immediately once low-battery alarm clears
    last_eye_state_.fill(-999);
    if (mode_ == Mode::TEXT && display_) {
        display_->clear();
        display_->drawText(2, 2, last_text_);
        display_->display();
    }
}

void OledNode::updateFrame()
{
    if (!display_) {
        return;
    }

    const rclcpp::Time now = get_clock()->now();

    if (low_battery_) {
        if (now >= next_low_blink_) {
            low_blink_on_ = !low_blink_on_;
            next_low_blink_ = now + rclcpp::Duration::from_seconds(0.45);
        }
        renderLowBatteryFrame(low_blink_on_);
        return;
    }

    if (mode_ == Mode::TEXT) {
        if (text_timeout_s_ > 0.0 && (now - text_time_).seconds() >= text_timeout_s_) {
            mode_ = Mode::EYES;
            last_eye_state_.fill(-999);
        } else {
            return;
        }
    }

    if (now >= next_gaze_) {
        std::uniform_int_distribution<size_t> dist(0, kGazeTargets.size() - 1);
        size_t idx = dist(rng_);
        for (int i = 0; i < 4 && kGazeTargets[idx] == std::make_pair(target_dx_, target_dy_); ++i) {
            idx = dist(rng_);
        }
        target_dx_ = kGazeTargets[idx].first;
        target_dy_ = kGazeTargets[idx].second;
        next_gaze_ = now + rclcpp::Duration::from_seconds(std::uniform_real_distribution<double>(0.65, 2.5)(rng_));
    }

    if (!blinking_ && now >= next_blink_) {
        blinking_ = true;
        blink_step_ = 0;
        const bool dbl = std::bernoulli_distribution(0.2)(rng_);
        const double delay = dbl ? 0.22 : std::uniform_real_distribution<double>(1.8, 5.2)(rng_);
        next_blink_ = now + rclcpp::Duration::from_seconds(delay);
    }

    gaze_x_ = stepSaccade(gaze_x_, target_dx_);
    gaze_y_ = stepSaccade(gaze_y_, target_dy_);
    renderEyesFrame();
}

void OledNode::renderEyesFrame()
{
    double scale = 1.0;
    if (blinking_) {
        scale = kBlinkProfile[blink_step_++];
        if (blink_step_ >= kBlinkProfile.size()) {
            blinking_ = false;
        }
    }

    const int h = std::max(2, static_cast<int>(std::lround(34.0 * scale)));
    const int r = (h <= 6) ? 1 : std::min(7, h / 2);
    const int dx = static_cast<int>(std::lround(gaze_x_));
    const int dy = static_cast<int>(std::lround(gaze_y_));
    const std::array<int, 4> state{dx, dy, h, r};
    if (state == last_eye_state_) {
        return;
    }

    const int top_y = 32 + dy - h / 2;
    display_->clear();
    display_->fillRoundedRect(38 + dx - 14, top_y, 28, h, r, true);
    display_->fillRoundedRect(90 + dx - 14, top_y, 28, h, r, true);
    if (display_->display()) {
        last_eye_state_ = state;
    } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Failed to flush OLED eye frame over I2C.");
    }
}

void OledNode::renderLowBatteryFrame(bool visible)
{
    const int state = visible ? 1 : 0;
    if (state == low_blink_rendered_) {
        return;
    }

    display_->clear();
    if (visible) {
        // Large nearly-empty battery outline + positive terminal + single low bar + warning mark
        display_->fillRoundedRect(18, 10, 86, 44, 4, true);
        display_->fillRoundedRect(22, 14, 78, 36, 2, false);
        display_->fillRoundedRect(104, 23, 7, 18, 2, true);
        display_->fillRoundedRect(26, 18, 9, 28, 1, true);
        display_->fillRoundedRect(62, 18, 6, 19, 1, true);
        display_->fillRoundedRect(62, 41, 6, 5, 1, true);
    }

    if (display_->display()) {
        low_blink_rendered_ = state;
    }
}

}  // namespace picar_drivers

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<picar_drivers::OledNode>());
    } catch (const std::exception &e) {
        std::cerr << "OledNode error: " << e.what() << std::endl;
    }
    rclcpp::shutdown();
    return 0;
}
