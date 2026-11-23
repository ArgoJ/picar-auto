#include "picar_drivers/ads7830_sensor.hpp"

extern "C" {
#include <fcntl.h>
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
}

#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace picar_drivers {

ADS7830Sensor::ADS7830Sensor(int bus_number, int address)
    : filename_("/dev/i2c-" + std::to_string(bus_number)), address_(address)
{
    file_ = open(filename_.c_str(), O_RDWR);
    if (file_ < 0) {
        throw std::runtime_error(
            "ADS7830: Failed to open " + filename_ + ": " + std::strerror(errno));
    }

    if (ioctl(file_, I2C_SLAVE, address_) < 0) {
        int err = errno;
        close(file_);
        file_ = -1;
        throw std::runtime_error(
            "ADS7830: Failed to set I2C_SLAVE address: " + std::string(std::strerror(err)));
    }

    // Quick probe read on channel 0 to verify chip reachability at startup
    uint8_t dummy = 0;
    if (!readChannelRaw(0, dummy)) {
        close(file_);
        file_ = -1;
        throw std::runtime_error(
            "ADS7830 not responding on I2C bus " + filename_);
    }
}

ADS7830Sensor::~ADS7830Sensor()
{
    if (file_ >= 0) {
        close(file_);
        file_ = -1;
    }
}

uint8_t ADS7830Sensor::buildControlByte(uint8_t channel)
{
    uint8_t ch_bits = static_cast<uint8_t>(((channel << 2) | (channel >> 1)) & 0x07);
    return static_cast<uint8_t>(CMD_SINGLE_ENDED_BASE | (ch_bits << 4));
}

bool ADS7830Sensor::readChannelRaw(uint8_t channel, uint8_t &raw_value)
{
    if (file_ < 0 || channel > 7) {
        return false;
    }

    const uint8_t control_byte = buildControlByte(channel);
    const int res = i2c_smbus_read_byte_data(file_, control_byte);
    if (res < 0) {
        return false;
    }

    raw_value = static_cast<uint8_t>(res & 0xFF);
    return true;
}

bool ADS7830Sensor::readChannelVoltage(uint8_t channel, double adc_ref_voltage, double &voltage_out)
{
    uint8_t raw = 0;
    if (!readChannelRaw(channel, raw)) {
        return false;
    }

    voltage_out = (static_cast<double>(raw) / 255.0) * adc_ref_voltage;
    return true;
}

}  // namespace picar_drivers
