#include "picar_drivers/mpu6050_sensor.hpp"

extern "C" {
#include <fcntl.h>
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
}

#include <cstring>
#include <iostream>
#include <stdexcept>

namespace picar_drivers {

MPU6050Sensor::MPU6050Sensor(int bus_number, int address)
    : filename_("/dev/i2c-" + std::to_string(bus_number)), address_(address)
{
    file_ = open(filename_.c_str(), O_RDWR);
    if (file_ < 0) {
        throw std::runtime_error("Fehler beim Oeffnen von " + filename_ + ": " + std::strerror(errno));
    }

    if (ioctl(file_, I2C_SLAVE, address_) < 0) {
        close(file_);
        file_ = -1;
        throw std::runtime_error("Konnte I2C_SLAVE 0x" + std::to_string(address_) + " nicht setzen: " + std::strerror(errno));
    }

    // Sensor aufwecken (Sleep-Bit in PWR_MGMT_1 loeschen)
    if (!writeRegister(REG_PWR_MGMT_1, 0x00)) {
        close(file_);
        file_ = -1;
        throw std::runtime_error("MPU6050 antwortet nicht auf I2C Bus " + filename_);
    }
}

MPU6050Sensor::~MPU6050Sensor()
{
    if (file_ >= 0) {
        close(file_);
        file_ = -1;
    }
}

int MPU6050Sensor::readRegister(uint8_t reg)
{
    if (file_ < 0) {
        return -1;
    }
    return i2c_smbus_read_byte_data(file_, reg);
}

bool MPU6050Sensor::writeRegister(uint8_t reg, uint8_t val)
{
    if (file_ < 0) {
        return false;
    }
    return i2c_smbus_write_byte_data(file_, reg, val) >= 0;
}

uint8_t MPU6050Sensor::getWhoAmI()
{
    int res = readRegister(REG_WHO_AM_I);
    return res >= 0 ? static_cast<uint8_t>(res) : 0x00;
}

void MPU6050Sensor::setGyroscopeRange(GyroRange range)
{
    gyro_range_ = static_cast<uint8_t>(range) & 0x03;
    writeRegister(REG_GYRO_CONFIG, gyro_range_ << 3);
}

void MPU6050Sensor::setAccelerometerRange(AccelRange range)
{
    accel_range_ = static_cast<uint8_t>(range) & 0x03;
    writeRegister(REG_ACCEL_CONFIG, accel_range_ << 3);
}

void MPU6050Sensor::setDlpfBandwidth(DlpfBandwidth bandwidth)
{
    dlpf_range_ = static_cast<uint8_t>(bandwidth) % 7;
    writeRegister(REG_CONFIG, dlpf_range_);
}

void MPU6050Sensor::setOffsets(double ax, double ay, double az, double gx, double gy, double gz)
{
    accel_offset_[0] = ax;
    accel_offset_[1] = ay;
    accel_offset_[2] = az;
    gyro_offset_[0]  = gx;
    gyro_offset_[1]  = gy;
    gyro_offset_[2]  = gz;
    calibrated_ = true;
}

void MPU6050Sensor::getOffsets(double offsets[6]) const
{
    offsets[0] = accel_offset_[0];
    offsets[1] = accel_offset_[1];
    offsets[2] = accel_offset_[2];
    offsets[3] = gyro_offset_[0];
    offsets[4] = gyro_offset_[1];
    offsets[5] = gyro_offset_[2];
}

bool MPU6050Sensor::readMotionData(ImuMotionData &data)
{
    if (file_ < 0) {
        return false;
    }

    uint8_t buf[14];
    if (i2c_smbus_read_i2c_block_data(file_, REG_ACCEL_XOUT_H, 14, buf) < 14) {
        return false;
    }

    auto to_i16 = [](uint8_t h, uint8_t l) {
        return static_cast<int16_t>((h << 8) | l);
    };

    const double a_scale = (1.0 / ACCEL_SCALE[accel_range_]) * GRAVITY;
    const double g_scale = (1.0 / GYRO_SCALE[gyro_range_]) * DEG_TO_RAD;

    data.accel_x = to_i16(buf[0], buf[1]) * a_scale - (calibrated_ ? accel_offset_[0] : 0.0);
    data.accel_y = to_i16(buf[2], buf[3]) * a_scale - (calibrated_ ? accel_offset_[1] : 0.0);
    data.accel_z = to_i16(buf[4], buf[5]) * a_scale - (calibrated_ ? accel_offset_[2] : 0.0);
    data.temp    = to_i16(buf[6], buf[7]) / 340.0 + 36.53;
    data.gyro_x  = to_i16(buf[8], buf[9]) * g_scale - (calibrated_ ? gyro_offset_[0] : 0.0);
    data.gyro_y  = to_i16(buf[10], buf[11]) * g_scale - (calibrated_ ? gyro_offset_[1] : 0.0);
    data.gyro_z  = to_i16(buf[12], buf[13]) * g_scale - (calibrated_ ? gyro_offset_[2] : 0.0);

    return true;
}

void MPU6050Sensor::calibrate(int samples)
{
    if (samples <= 0) {
        return;
    }

    calibrated_ = false;
    double sum_ax = 0.0, sum_ay = 0.0, sum_az = 0.0;
    double sum_gx = 0.0, sum_gy = 0.0, sum_gz = 0.0;

    int count = 0;
    while (count < samples) {
        ImuMotionData d;
        if (readMotionData(d)) {
            sum_ax += d.accel_x;
            sum_ay += d.accel_y;
            sum_az += d.accel_z;
            sum_gx += d.gyro_x;
            sum_gy += d.gyro_y;
            sum_gz += d.gyro_z;
            ++count;
            usleep(4000);  // 4 ms
        } else {
            usleep(10000);
        }
    }

    accel_offset_[0] = sum_ax / samples;
    accel_offset_[1] = sum_ay / samples;
    accel_offset_[2] = (sum_az / samples) - GRAVITY;  // Z misst 1g (9.80665 m/s^2) nach oben im Ruhezustand
    gyro_offset_[0]  = sum_gx / samples;
    gyro_offset_[1]  = sum_gy / samples;
    gyro_offset_[2]  = sum_gz / samples;
    calibrated_ = true;
}

}  // namespace picar_drivers
