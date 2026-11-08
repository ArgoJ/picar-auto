#pragma once

#include <cmath>
#include <cstdint>
#include <string>

namespace picar_drivers {

class PCA9685 {
public:
    explicit PCA9685(int bus_number = 1, int address = 0x5F);
    virtual ~PCA9685();

    bool init(double frequency = 50.0);
    void setPwm(int channel, uint16_t on, uint16_t off);
    void setChannelFullOn(int channel);
    void setChannelFullOff(int channel);
    void setChannelDuty(int channel, double duty);

    // Servo-Steuerung (50 Hz)
    void setServoPulse(int channel, double pulse_us);
    void setServoAngle(int channel, double angle_deg,
                       double min_pulse = 500.0, double max_pulse = 2400.0,
                       double range_deg = 180.0);

    // DC-Motor-Steuerung (Slow Decay H-Bruecke)
    void setMotorThrottle(int pin_in1, int pin_in2, double throttle, bool slow_decay = true);

    // Notabschaltung aller Kanäle
    void allOff();
    void wakeUp();

    int getBusNumber() const { return bus_number_; }
    int getAddress() const { return address_; }

private:
    int readRegister(uint8_t reg);
    bool writeRegister(uint8_t reg, uint8_t val);

    int file_{-1};
    int bus_number_{1};
    int address_{0x5F};
    std::string filename_;
    double frequency_{50.0};

    // PCA9685 Register-Adressen
    static constexpr uint8_t REG_MODE1         = 0x00;
    static constexpr uint8_t REG_MODE2         = 0x01;
    static constexpr uint8_t REG_LED0_ON_L     = 0x06;
    static constexpr uint8_t REG_ALL_LED_ON_L  = 0xFA;
    static constexpr uint8_t REG_ALL_LED_ON_H  = 0xFB;
    static constexpr uint8_t REG_ALL_LED_OFF_L = 0xFC;
    static constexpr uint8_t REG_ALL_LED_OFF_H = 0xFD;
    static constexpr uint8_t REG_PRE_SCALE     = 0xFE;
};

}  // namespace picar_drivers
