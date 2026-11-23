#pragma once

#include <cstdint>
#include <string>

namespace picar_drivers {

/**
 * @brief I2C driver for the 8-bit 8-channel ADC ADS7830 (default address 0x48).
 */
class ADS7830Sensor {
public:
    explicit ADS7830Sensor(int bus_number = 1, int address = 0x48);
    ~ADS7830Sensor();

    ADS7830Sensor(const ADS7830Sensor &) = delete;
    ADS7830Sensor &operator=(const ADS7830Sensor &) = delete;

    /**
     * @brief Read a single single-ended ADC channel (0..7) as an 8-bit raw value (0..255).
     */
    bool readChannelRaw(uint8_t channel, uint8_t &raw_value);

    /**
     * @brief Read an ADC channel (0..7) and convert it to the input voltage at the ADC pin [V].
     */
    bool readChannelVoltage(uint8_t channel, double adc_ref_voltage, double &voltage_out);

    static uint8_t buildControlByte(uint8_t channel);

private:
    int file_{-1};
    std::string filename_;
    int address_{0x48};

    // Single-Ended Inputs (SD=1), Internal Reference OFF, A/D Converter ON (PD1=0, PD0=1)
    static constexpr uint8_t CMD_SINGLE_ENDED_BASE = 0x84;
};

}  // namespace picar_drivers
