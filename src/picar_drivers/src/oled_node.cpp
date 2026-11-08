#include "picar_drivers/oled_node.hpp"

namespace picar_drivers {

OledNode::OledNode()
    : Node("oled_node")
{
    declare_parameter<int>("bus_number", 1);
    declare_parameter<int>("i2c_address", 0x3C);

    bus_number_ = get_parameter("bus_number").as_int();
    i2c_address_ = get_parameter("i2c_address").as_int();

    initHardware();

    sub_text_ = create_subscription<std_msgs::msg::String>(
        "picar/oled_text", 10,
        std::bind(&OledNode::textCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(), "OledNode initialisiert auf I2C Bus %d (0x%02X).",
                bus_number_, i2c_address_);
}

void OledNode::initHardware()
{
    // TODO: SSD1306 Display-Initialisierung über I2C
}

void OledNode::textCallback(const std_msgs::msg::String::SharedPtr msg)
{
    // TODO: Textzeilen auf dem Display rendern
    (void)msg;
}

}  // namespace picar_drivers

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<picar_drivers::OledNode>());
    rclcpp::shutdown();
    return 0;
}

