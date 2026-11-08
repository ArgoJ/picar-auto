#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <string>

namespace picar_drivers {

struct ImuMotionData {
    double accel_x{0.0};  // m/s^2
    double accel_y{0.0};  // m/s^2
    double accel_z{0.0};  // m/s^2
    double gyro_x{0.0};   // rad/s
    double gyro_y{0.0};   // rad/s
    double gyro_z{0.0};   // rad/s
    double temp{0.0};     // deg C
};

class MPU6050Sensor {
public:
    enum AccelRange : uint8_t {
        ACC_2_G  = 0,
        ACC_4_G  = 1,
        ACC_8_G  = 2,
        ACC_16_G = 3
    };

    enum GyroRange : uint8_t {
        GYR_250_DEG  = 0,
        GYR_500_DEG  = 1,
        GYR_1000_DEG = 2,
        GYR_2000_DEG = 3
    };

    enum DlpfBandwidth : uint8_t {
        DLPF_260_HZ = 0,
        DLPF_184_HZ = 1,
        DLPF_94_HZ  = 2,
        DLPF_44_HZ  = 3,
        DLPF_21_HZ  = 4,
        DLPF_10_HZ  = 5,
        DLPF_5_HZ   = 6
    };

    explicit MPU6050Sensor(int bus_number = 1, int address = 0x68);
    ~MPU6050Sensor();

    // Konfiguration
    void setGyroscopeRange(GyroRange range);
    void setAccelerometerRange(AccelRange range);
    void setDlpfBandwidth(DlpfBandwidth bandwidth);
    void setOffsets(double ax, double ay, double az, double gx, double gy, double gz);
    void calibrate(int samples = 100);

    // Atomares Auslesen aller 14 Register
    bool readMotionData(ImuMotionData &data);

    uint8_t getWhoAmI();
    bool isCalibrated() const { return calibrated_; }
    void getOffsets(double offsets[6]) const;

private:
    int readRegister(uint8_t reg);
    bool writeRegister(uint8_t reg, uint8_t val);

    int file_{-1};
    std::string filename_;
    int address_{0x68};

    uint8_t accel_range_{0};
    uint8_t gyro_range_{0};
    uint8_t dlpf_range_{2};
    bool calibrated_{false};

    // Offsets: ax, ay, az (m/s^2), gx, gy, gz (rad/s)
    double accel_offset_[3]{0.0, 0.0, 0.0};
    double gyro_offset_[3]{0.0, 0.0, 0.0};

    // Register-Adressen
    static constexpr uint8_t REG_CONFIG       = 0x1A;
    static constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
    static constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
    static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
    static constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
    static constexpr uint8_t REG_WHO_AM_I     = 0x75;

    // Physikalische Konstanten
    static constexpr double GRAVITY    = 9.80665;
    static constexpr double DEG_TO_RAD = M_PI / 180.0;

    static constexpr std::array<double, 4> ACCEL_SCALE = {16384.0, 8192.0, 4096.0, 2048.0};
    static constexpr std::array<double, 4> GYRO_SCALE  = {131.0, 65.5, 32.8, 16.4};
};

}  // namespace picar_drivers
