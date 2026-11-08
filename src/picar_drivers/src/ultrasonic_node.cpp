#include "picar_drivers/ultrasonic_node.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <thread>

namespace picar_drivers {

UltrasonicNode::UltrasonicNode()
    : Node("ultrasonic_node")
{
    declare_parameter<std::string>("chip_path", "/dev/gpiochip0");
    declare_parameter<int>("pin_trig", 23);
    declare_parameter<int>("pin_echo", 24);
    declare_parameter<std::string>("frame_id", "ultrasonic_link");
    declare_parameter<double>("rate_hz", 10.0);
    declare_parameter<double>("min_range", 0.02);
    declare_parameter<double>("max_range", 4.00);

    chip_path_   = get_parameter("chip_path").as_string();
    pin_trig_    = get_parameter("pin_trig").as_int();
    pin_echo_    = get_parameter("pin_echo").as_int();
    frame_id_    = get_parameter("frame_id").as_string();
    rate_hz_     = get_parameter("rate_hz").as_double();
    min_range_m_ = get_parameter("min_range").as_double();
    max_range_m_ = get_parameter("max_range").as_double();

    initHardware();

    pub_range_ = create_publisher<sensor_msgs::msg::Range>("picar/ultrasonic/range", 10);

    int timer_period_ms = static_cast<int>(1000.0 / (rate_hz_ > 0.0 ? rate_hz_ : 10.0));
    timer_ = create_wall_timer(
        std::chrono::milliseconds(timer_period_ms),
        std::bind(&UltrasonicNode::timerCallback, this));

    RCLCPP_INFO(get_logger(),
                "UltrasonicNode aktiv auf %s (Trig: GPIO %d, Echo: GPIO %d, Rate: %.1f Hz)",
                chip_path_.c_str(), pin_trig_, pin_echo_, rate_hz_);
}

UltrasonicNode::~UltrasonicNode()
{
    try {
        if (request_) {
            request_->release();
        }
    } catch (...) {
    }
}

void UltrasonicNode::initHardware()
{
    try {
        chip_ = std::make_unique<::gpiod::chip>(chip_path_);

        auto settings_out = ::gpiod::line_settings()
            .set_direction(::gpiod::line::direction::OUTPUT)
            .set_output_value(::gpiod::line::value::INACTIVE);

        auto settings_in = ::gpiod::line_settings()
            .set_direction(::gpiod::line::direction::INPUT);

        request_ = std::make_unique<::gpiod::line_request>(
            chip_->prepare_request()
                .set_consumer("picar_ultrasonic_node")
                .add_line_settings(::gpiod::line::offsets{static_cast<::gpiod::line::offset>(pin_trig_)}, settings_out)
                .add_line_settings(::gpiod::line::offsets{static_cast<::gpiod::line::offset>(pin_echo_)}, settings_in)
                .do_request()
        );

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    } catch (const std::exception &e) {
        RCLCPP_FATAL(get_logger(), "GPIO Initialisierung fuer Ultraschall fehlgeschlagen: %s", e.what());
        throw;
    }
}

double UltrasonicNode::measureDistanceMeters()
{
    if (!request_) {
        return -1.0;
    }

    // 1. 15 Mikrosekunden Trigger-Impuls
    request_->set_value(static_cast<::gpiod::line::offset>(pin_trig_), ::gpiod::line::value::ACTIVE);
    std::this_thread::sleep_for(std::chrono::microseconds(15));
    request_->set_value(static_cast<::gpiod::line::offset>(pin_trig_), ::gpiod::line::value::INACTIVE);

    auto t_start = std::chrono::steady_clock::now();
    auto timeout_start = t_start + std::chrono::milliseconds(10);

    // 2. Warten bis Echo auf HIGH geht (max. 10 ms)
    while (request_->get_value(static_cast<::gpiod::line::offset>(pin_echo_)) == ::gpiod::line::value::INACTIVE) {
        if (std::chrono::steady_clock::now() > timeout_start) {
            return -1.0;  // Kein Echo-Start
        }
    }
    auto t_rise = std::chrono::steady_clock::now();
    auto timeout_end = t_rise + std::chrono::milliseconds(30);

    // 3. Warten bis Echo wieder auf LOW faellt (max. 30 ms = ~5.1 m)
    while (request_->get_value(static_cast<::gpiod::line::offset>(pin_echo_)) == ::gpiod::line::value::ACTIVE) {
        if (std::chrono::steady_clock::now() > timeout_end) {
            return -2.0;  // Echo-Timeout (> 400 cm)
        }
    }
    auto t_fall = std::chrono::steady_clock::now();

    // 4. Flugzeit berechnen (Schallgeschwindigkeit = 343.0 m/s bei 20°C)
    auto dt_us = std::chrono::duration_cast<std::chrono::microseconds>(t_fall - t_rise).count();
    double dist_m = (static_cast<double>(dt_us) * 1e-6 * 343.0) / 2.0;

    return dist_m;
}

void UltrasonicNode::timerCallback()
{
    double dist_m = measureDistanceMeters();

    auto msg = sensor_msgs::msg::Range();
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    msg.field_of_view = 0.523599f;  // ~30 Grad Schallkegel in Radiant
    msg.min_range = static_cast<float>(min_range_m_);
    msg.max_range = static_cast<float>(max_range_m_);

    if (dist_m < 0.0) {
        // Kein Echo oder Timeout -> Out of range (REP 117: +Inf)
        msg.range = std::numeric_limits<float>::infinity();
    } else if (dist_m < min_range_m_) {
        // Zu nah -> Unterhalb Minimalbereich (REP 117: -Inf)
        msg.range = -std::numeric_limits<float>::infinity();
    } else if (dist_m > max_range_m_) {
        // Zu weit
        msg.range = std::numeric_limits<float>::infinity();
    } else {
        msg.range = static_cast<float>(dist_m);
    }

    pub_range_->publish(msg);
}

}  // namespace picar_drivers

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<picar_drivers::UltrasonicNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
