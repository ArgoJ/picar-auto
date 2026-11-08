#include "picar_drivers/buzzer_node.hpp"

#include <chrono>
#include <lgpio.h>

namespace picar_drivers {

BuzzerNode::BuzzerNode()
    : Node("buzzer_node")
{
    declare_parameter<int>("gpio_chip", 0);
    declare_parameter<int>("pin_buzzer", 18);
    declare_parameter<float>("default_frequency", 1000.0f);

    gpio_chip_    = get_parameter("gpio_chip").as_int();
    pin_buzzer_   = get_parameter("pin_buzzer").as_int();
    default_freq_ = static_cast<float>(get_parameter("default_frequency").as_double());

    initHardware();

    // Subscriptions
    sub_cmd_ = create_subscription<std_msgs::msg::Bool>(
        "picar/buzzer/cmd", 10,
        std::bind(&BuzzerNode::onBuzzerCmd, this, std::placeholders::_1));

    sub_freq_ = create_subscription<std_msgs::msg::Int32>(
        "picar/buzzer/freq", 10,
        std::bind(&BuzzerNode::onBuzzerFreq, this, std::placeholders::_1));

    sub_beep_ = create_subscription<std_msgs::msg::Int32>(
        "picar/buzzer/beep", 10,
        std::bind(&BuzzerNode::onBuzzerBeep, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "BuzzerNode aktiv auf Chip %d, GPIO %d (Standard-Frequenz: %.1f Hz)",
                gpio_chip_, pin_buzzer_, default_freq_);
}

BuzzerNode::~BuzzerNode()
{
    stopTone();
    if (lg_handle_ >= 0) {
        lgGpiochipClose(lg_handle_);
        lg_handle_ = -1;
    }
}

void BuzzerNode::initHardware()
{
    lg_handle_ = lgGpiochipOpen(gpio_chip_);
    if (lg_handle_ < 0) {
        RCLCPP_ERROR(get_logger(), "Konnte gpiochip %d nicht oeffnen (Fehler: %d)",
                     gpio_chip_, lg_handle_);
        throw std::runtime_error("lgGpiochipOpen fehlgeschlagen");
    }

    // Sicherstellen, dass der Buzzer beim Start stumm ist
    stopTone();
}

void BuzzerNode::playTone(float freq_hz)
{
    if (lg_handle_ < 0) {
        return;
    }

    if (freq_hz <= 0.0f) {
        stopTone();
        return;
    }

    // 50% Tastverhaeltnis (Rechtecksignal fuer passive Tonal-Buzzer)
    int res = lgTxPwm(lg_handle_, pin_buzzer_, freq_hz, 50.0f, 0, 0);
    if (res < 0) {
        RCLCPP_WARN(get_logger(), "lgTxPwm fehlgeschlagen (Fehler: %d)", res);
    } else {
        is_playing_ = true;
    }
}

void BuzzerNode::stopTone()
{
    if (lg_handle_ >= 0 && is_playing_) {
        lgTxPwm(lg_handle_, pin_buzzer_, 0.0f, 0.0f, 0, 0);
        is_playing_ = false;
    }
}

void BuzzerNode::beep(int duration_ms, float freq_hz)
{
    playTone(freq_hz);

    if (beep_timer_) {
        beep_timer_->cancel();
    }

    beep_timer_ = create_wall_timer(
        std::chrono::milliseconds(duration_ms),
        [this]() {
            stopTone();
            if (beep_timer_) {
                beep_timer_->cancel();
            }
        });
}

void BuzzerNode::onBuzzerCmd(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data) {
        playTone(default_freq_);
    } else {
        stopTone();
    }
}

void BuzzerNode::onBuzzerFreq(const std_msgs::msg::Int32::SharedPtr msg)
{
    if (msg->data > 0) {
        playTone(static_cast<float>(msg->data));
    } else {
        stopTone();
    }
}

void BuzzerNode::onBuzzerBeep(const std_msgs::msg::Int32::SharedPtr msg)
{
    int duration = msg->data > 0 ? msg->data : 100;
    beep(duration, default_freq_);
}

}  // namespace picar_drivers

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<picar_drivers::BuzzerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
