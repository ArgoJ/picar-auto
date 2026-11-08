#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>

#include "picar_drivers/mpu6050_sensor.hpp"

namespace picar_drivers {

class Mpu6050Node : public rclcpp::Node {
public:
    explicit Mpu6050Node(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    virtual ~Mpu6050Node() = default;

private:
    void declareParameters();
    void timerCallback();

    std::unique_ptr<MPU6050Sensor> sensor_;

    // ROS 2 Kommunikationsschnittstellen
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr pub_temp_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Parameter
    int bus_number_{1};
    int device_address_{0x68};
    std::string frame_id_{"imu_link"};
    int frequency_{50};
    bool calibrate_{true};
    int calibration_samples_{50};
};

}  // namespace picar_drivers
