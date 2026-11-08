#include "picar_drivers/led_node.hpp"

#include <iostream>

namespace picar_drivers {

LedNode::LedNode()
    : Node("led_node")
{
    declare_parameter<std::string>("chip_path", "/dev/gpiochip0");
    declare_parameter<int>("pin_led1", 9);
    declare_parameter<int>("pin_led2", 25);
    declare_parameter<int>("pin_led3", 11);

    chip_path_ = get_parameter("chip_path").as_string();
    pin_led1_  = get_parameter("pin_led1").as_int();
    pin_led2_  = get_parameter("pin_led2").as_int();
    pin_led3_  = get_parameter("pin_led3").as_int();

    initHardware();

    // Subscriptions
    sub_cmd_ = create_subscription<std_msgs::msg::Int32>(
        "picar/led/cmd", 10,
        std::bind(&LedNode::onLedCmd, this, std::placeholders::_1));

    sub_led1_ = create_subscription<std_msgs::msg::Bool>(
        "picar/led/led1", 10,
        std::bind(&LedNode::onLed1Cmd, this, std::placeholders::_1));

    sub_led2_ = create_subscription<std_msgs::msg::Bool>(
        "picar/led/led2", 10,
        std::bind(&LedNode::onLed2Cmd, this, std::placeholders::_1));

    sub_led3_ = create_subscription<std_msgs::msg::Bool>(
        "picar/led/led3", 10,
        std::bind(&LedNode::onLed3Cmd, this, std::placeholders::_1));

    sub_all_ = create_subscription<std_msgs::msg::Bool>(
        "picar/led/all", 10,
        std::bind(&LedNode::onAllCmd, this, std::placeholders::_1));

    // Status publisher
    pub_status_ = create_publisher<std_msgs::msg::Int32>("picar/led/status", 10);

    RCLCPP_INFO(get_logger(),
                "LedNode aktiv auf %s (LED1: GPIO%d, LED2: GPIO%d, LED3: GPIO%d)",
                chip_path_.c_str(), pin_led1_, pin_led2_, pin_led3_);
}

LedNode::~LedNode()
{
    try {
        setAll(false);
    } catch (...) {
    }
}

void LedNode::initHardware()
{
    try {
        chip_ = std::make_unique<::gpiod::chip>(chip_path_);

        auto settings = ::gpiod::line_settings()
            .set_direction(::gpiod::line::direction::OUTPUT)
            .set_output_value(::gpiod::line::value::INACTIVE);

        auto offsets = ::gpiod::line::offsets{
            static_cast<::gpiod::line::offset>(pin_led1_),
            static_cast<::gpiod::line::offset>(pin_led2_),
            static_cast<::gpiod::line::offset>(pin_led3_)
        };

        request_ = std::make_unique<::gpiod::line_request>(
            chip_->prepare_request()
                .set_consumer("picar_led_node")
                .add_line_settings(offsets, settings)
                .do_request()
        );

        setAll(false);
    } catch (const std::exception &e) {
        RCLCPP_FATAL(get_logger(), "GPIO Initialisierung fehlgeschlagen: %s", e.what());
        throw;
    }
}

void LedNode::setLed(int index, bool state)
{
    if (!request_) {
        return;
    }

    int pin = pin_led1_;
    if (index == 1) {
        pin = pin_led1_;
        led1_state_ = state;
    } else if (index == 2) {
        pin = pin_led2_;
        led2_state_ = state;
    } else if (index == 3) {
        pin = pin_led3_;
        led3_state_ = state;
    } else {
        return;
    }

    request_->set_value(
        static_cast<::gpiod::line::offset>(pin),
        state ? ::gpiod::line::value::ACTIVE : ::gpiod::line::value::INACTIVE
    );

    publishStatus();
}

void LedNode::setAll(bool state)
{
    if (!request_) {
        return;
    }

    auto val = state ? ::gpiod::line::value::ACTIVE : ::gpiod::line::value::INACTIVE;
    request_->set_value(static_cast<::gpiod::line::offset>(pin_led1_), val);
    request_->set_value(static_cast<::gpiod::line::offset>(pin_led2_), val);
    request_->set_value(static_cast<::gpiod::line::offset>(pin_led3_), val);

    led1_state_ = state;
    led2_state_ = state;
    led3_state_ = state;

    publishStatus();
}

void LedNode::setMask(int mask)
{
    setLed(1, (mask & 0x01) != 0);
    setLed(2, (mask & 0x02) != 0);
    setLed(3, (mask & 0x04) != 0);
}

void LedNode::publishStatus()
{
    if (!pub_status_) {
        return;
    }
    auto msg = std_msgs::msg::Int32();
    msg.data = (led1_state_ ? 1 : 0) | ((led2_state_ ? 1 : 0) << 1) | ((led3_state_ ? 1 : 0) << 2);
    pub_status_->publish(msg);
}

void LedNode::onLedCmd(const std_msgs::msg::Int32::SharedPtr msg)
{
    int val = msg->data;
    if (val >= 0 && val <= 7) {
        setMask(val);
    } else if (val == 10) {
        setLed(1, false);
    } else if (val == 11) {
        setLed(1, true);
    } else if (val == 20) {
        setLed(2, false);
    } else if (val == 21) {
        setLed(2, true);
    } else if (val == 30) {
        setLed(3, false);
    } else if (val == 31) {
        setLed(3, true);
    } else {
        RCLCPP_WARN(get_logger(), "Unbekanntes LED-Kommando: %d", val);
    }
}

void LedNode::onLed1Cmd(const std_msgs::msg::Bool::SharedPtr msg)
{
    setLed(1, msg->data);
}

void LedNode::onLed2Cmd(const std_msgs::msg::Bool::SharedPtr msg)
{
    setLed(2, msg->data);
}

void LedNode::onLed3Cmd(const std_msgs::msg::Bool::SharedPtr msg)
{
    setLed(3, msg->data);
}

void LedNode::onAllCmd(const std_msgs::msg::Bool::SharedPtr msg)
{
    setAll(msg->data);
}

}  // namespace picar_drivers

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<picar_drivers::LedNode>());
    } catch (const std::exception &e) {
        std::cerr << "LedNode Fehler: " << e.what() << std::endl;
    }
    rclcpp::shutdown();
    return 0;
}
