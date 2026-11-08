#include "picar_drivers/mpu6050_node.hpp"

#include <chrono>
#include <iostream>

namespace picar_drivers {

Mpu6050Node::Mpu6050Node(const rclcpp::NodeOptions &options)
    : Node("mpu6050_node", options)
{
    declareParameters();

    bus_number_          = get_parameter("bus_number").as_int();
    device_address_      = get_parameter("device_address").as_int();
    frame_id_            = get_parameter("frame_id").as_string();
    frequency_           = get_parameter("frequency").as_int();
    calibrate_           = get_parameter("calibrate").as_bool();
    calibration_samples_ = get_parameter("calibration_samples").as_int();

    if (frequency_ <= 0) {
        frequency_ = 50;
    }

    try {
        sensor_ = std::make_unique<MPU6050Sensor>(bus_number_, device_address_);
    } catch (const std::exception &e) {
        RCLCPP_FATAL(get_logger(), "MPU6050 Initialisierung fehlgeschlagen: %s", e.what());
        throw;
    }

    uint8_t who = sensor_->getWhoAmI();
    RCLCPP_INFO(get_logger(), "MPU6050 erkannt auf Bus %d, Addr 0x%02X (WHO_AM_I: 0x%02X)",
                bus_number_, device_address_, who);

    // Sensorbereiche konfigurieren
    sensor_->setGyroscopeRange(
        static_cast<MPU6050Sensor::GyroRange>(get_parameter("gyro_range").as_int()));
    sensor_->setAccelerometerRange(
        static_cast<MPU6050Sensor::AccelRange>(get_parameter("accel_range").as_int()));
    sensor_->setDlpfBandwidth(
        static_cast<MPU6050Sensor::DlpfBandwidth>(get_parameter("dlpf_bandwidth").as_int()));

    if (calibrate_) {
        RCLCPP_INFO(get_logger(), "Kalibriere IMU (%d Samples)... Roboter ruhig halten.", calibration_samples_);
        sensor_->calibrate(calibration_samples_);
        double offsets[6];
        sensor_->getOffsets(offsets);
        RCLCPP_INFO(get_logger(), "Kalibrierung fertig. Offsets Accel=[%.3f, %.3f, %.3f] Gyro=[%.4f, %.4f, %.4f]",
                    offsets[0], offsets[1], offsets[2], offsets[3], offsets[4], offsets[5]);
    } else {
        sensor_->setOffsets(
            get_parameter("accel_x_offset").as_double(),
            get_parameter("accel_y_offset").as_double(),
            get_parameter("accel_z_offset").as_double(),
            get_parameter("gyro_x_offset").as_double(),
            get_parameter("gyro_y_offset").as_double(),
            get_parameter("gyro_z_offset").as_double()
        );
    }

    // Publisher einrichten
    pub_imu_  = create_publisher<sensor_msgs::msg::Imu>("picar/imu/data", 10);
    pub_temp_ = create_publisher<sensor_msgs::msg::Temperature>("picar/imu/temperature", 10);

    timer_ = create_wall_timer(
        std::chrono::microseconds(1000000 / frequency_),
        std::bind(&Mpu6050Node::timerCallback, this)
    );

    RCLCPP_INFO(get_logger(), "Mpu6050Node aktiv: %d Hz, frame_id '%s'", frequency_, frame_id_.c_str());
}

void Mpu6050Node::declareParameters()
{
    declare_parameter<int>("bus_number", 1);
    declare_parameter<int>("device_address", 0x68);
    declare_parameter<std::string>("frame_id", "imu_link");
    declare_parameter<int>("frequency", 50);
    declare_parameter<bool>("calibrate", true);
    declare_parameter<int>("calibration_samples", 50);
    declare_parameter<int>("gyro_range", 0);
    declare_parameter<int>("accel_range", 0);
    declare_parameter<int>("dlpf_bandwidth", 2);
    declare_parameter<double>("accel_x_offset", 0.0);
    declare_parameter<double>("accel_y_offset", 0.0);
    declare_parameter<double>("accel_z_offset", 0.0);
    declare_parameter<double>("gyro_x_offset", 0.0);
    declare_parameter<double>("gyro_y_offset", 0.0);
    declare_parameter<double>("gyro_z_offset", 0.0);
}

void Mpu6050Node::timerCallback()
{
    if (!sensor_) {
        return;
    }

    ImuMotionData data;
    if (!sensor_->readMotionData(data)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "I2C Lesefehler beim MPU6050 Sensor!");
        return;
    }

    rclcpp::Time now = get_clock()->now();

    // 1. sensor_msgs/msg/Imu
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = now;
    imu_msg.header.frame_id = frame_id_;

    imu_msg.linear_acceleration.x = data.accel_x;
    imu_msg.linear_acceleration.y = data.accel_y;
    imu_msg.linear_acceleration.z = data.accel_z;

    imu_msg.angular_velocity.x = data.gyro_x;
    imu_msg.angular_velocity.y = data.gyro_y;
    imu_msg.angular_velocity.z = data.gyro_z;

    // REP 145: Wenn Orientierung nicht durch Sensor berechnet wird, erstes Element -1.0 setzen
    imu_msg.orientation_covariance[0] = -1.0;

    // Standard-Kovarianzen
    imu_msg.linear_acceleration_covariance[0] = 0.04;
    imu_msg.linear_acceleration_covariance[4] = 0.04;
    imu_msg.linear_acceleration_covariance[8] = 0.04;

    imu_msg.angular_velocity_covariance[0] = 0.002;
    imu_msg.angular_velocity_covariance[4] = 0.002;
    imu_msg.angular_velocity_covariance[8] = 0.002;

    pub_imu_->publish(imu_msg);

    // 2. sensor_msgs/msg/Temperature
    sensor_msgs::msg::Temperature temp_msg;
    temp_msg.header.stamp = now;
    temp_msg.header.frame_id = frame_id_;
    temp_msg.temperature = data.temp;
    temp_msg.variance = 0.25;

    pub_temp_->publish(temp_msg);
}

}  // namespace picar_drivers

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<picar_drivers::Mpu6050Node>());
    } catch (const std::exception &e) {
        std::cerr << "Mpu6050Node Fehler: " << e.what() << std::endl;
    }
    rclcpp::shutdown();
    return 0;
}
