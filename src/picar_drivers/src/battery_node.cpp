#include "picar_drivers/battery_node.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace picar_drivers {

BatteryNode::BatteryNode(const rclcpp::NodeOptions &options)
    : Node("battery_node", options)
{
    initParameters();

    try {
        sensor_ = std::make_unique<ADS7830Sensor>(bus_number_, device_address_);
    } catch (const std::exception &e) {
        RCLCPP_FATAL(get_logger(), "Failed to initialize ADS7830: %s", e.what());
        throw;
    }

    pub_battery_state_ = create_publisher<sensor_msgs::msg::BatteryState>("picar/battery/state", 10);
    pub_low_alarm_     = create_publisher<std_msgs::msg::Bool>("picar/battery/low_alarm", 10);

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate_hz_));
    timer_ = create_wall_timer(period, std::bind(&BatteryNode::timerCallback, this));

    RCLCPP_INFO(
        get_logger(),
        "BatteryNode started (bus=%d, addr=0x%02X, ch=%d, rate=%.2f Hz, range=%.2f..%.2f V)",
        bus_number_, device_address_, adc_channel_, rate_hz_, empty_voltage_, full_voltage_);
}

void BatteryNode::initParameters()
{
    bus_number_     = declare_parameter("bus_number", bus_number_);
    device_address_ = declare_parameter("device_address", device_address_);
    adc_channel_    = std::clamp<int>(declare_parameter("adc_channel", adc_channel_), 0, 7);
    rate_hz_        = declare_parameter("rate_hz", rate_hz_);
    frame_id_       = declare_parameter("frame_id", frame_id_);
    if (rate_hz_ <= 0.0) {
        rate_hz_ = 2.0;
    }

    adc_ref_voltage_        = declare_parameter("adc_ref_voltage", adc_ref_voltage_);
    const double r_top      = declare_parameter("r_top", 3000.0);
    const double r_bottom   = declare_parameter("r_bottom", 1000.0);
    full_voltage_           = declare_parameter("full_voltage", full_voltage_);
    empty_voltage_          = declare_parameter("empty_voltage", empty_voltage_);
    low_battery_percentage_ = declare_parameter("low_battery_percentage", low_battery_percentage_);

    if (r_bottom <= 0.0 || (r_top + r_bottom) <= 0.0) {
        RCLCPP_WARN(get_logger(), "Invalid voltage divider resistors; falling back to R15=3000, R17=1000.");
        division_ratio_ = 0.25;
    } else {
        division_ratio_ = r_bottom / (r_top + r_bottom);
    }

    if (full_voltage_ <= empty_voltage_) {
        RCLCPP_WARN(get_logger(), "Invalid thresholds (full_voltage <= empty_voltage); using 8.4 V / 6.75 V.");
        full_voltage_ = 8.4;
        empty_voltage_ = 6.75;
    }
}

void BatteryNode::timerCallback()
{
    double a0_voltage = 0.0;
    if (!sensor_ || !sensor_->readChannelVoltage(static_cast<uint8_t>(adc_channel_), adc_ref_voltage_, a0_voltage)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Failed to read ADS7830 ADC over I2C.");
        return;
    }

    const double v_bat = a0_voltage / division_ratio_;
    const double pct_100 = std::clamp(
        ((v_bat - empty_voltage_) / (full_voltage_ - empty_voltage_)) * 100.0, 0.0, 100.0);
    const bool is_low = (pct_100 < low_battery_percentage_);

    using BatteryState = sensor_msgs::msg::BatteryState;
    constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

    BatteryState msg;
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = frame_id_;
    msg.voltage = static_cast<float>(v_bat);
    msg.temperature = kNaN;
    msg.current = kNaN;
    msg.charge = kNaN;
    msg.capacity = kNaN;
    msg.design_capacity = kNaN;
    msg.percentage = static_cast<float>(pct_100 / 100.0);
    msg.present = (v_bat > 0.5);
    msg.location = frame_id_;
    msg.power_supply_technology = BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    msg.power_supply_status = msg.present
        ? BatteryState::POWER_SUPPLY_STATUS_DISCHARGING
        : BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    msg.power_supply_health = (v_bat > full_voltage_ + 0.3) ? BatteryState::POWER_SUPPLY_HEALTH_OVERVOLTAGE
                            : (v_bat >= empty_voltage_)     ? BatteryState::POWER_SUPPLY_HEALTH_GOOD
                                                            : BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    pub_battery_state_->publish(msg);

    std_msgs::msg::Bool alarm_msg;
    alarm_msg.data = is_low;
    pub_low_alarm_->publish(alarm_msg);

    if (is_low) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 10000,
            "Low battery warning: %.1f%% (%.2f V < threshold %.1f%%)",
            pct_100, v_bat, low_battery_percentage_);
    }
}

}  // namespace picar_drivers

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<picar_drivers::BatteryNode>());
    } catch (const std::exception &e) {
        std::cerr << "BatteryNode error: " << e.what() << std::endl;
    }
    rclcpp::shutdown();
    return 0;
}
