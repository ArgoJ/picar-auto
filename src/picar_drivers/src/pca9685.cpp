#include "picar_drivers/pca9685.hpp"

extern "C" {
#include <fcntl.h>
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
}

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>

namespace picar_drivers {

PCA9685::PCA9685(int bus_number, int address)
    : bus_number_(bus_number),
      address_(address),
      filename_("/dev/i2c-" + std::to_string(bus_number))
{
}

PCA9685::~PCA9685()
{
    try {
        allOff();
    } catch (...) {
    }

    if (file_ >= 0) {
        close(file_);
        file_ = -1;
    }
}

bool PCA9685::init(double frequency)
{
    frequency_ = frequency;

    file_ = open(filename_.c_str(), O_RDWR);
    if (file_ < 0) {
        throw std::runtime_error("PCA9685: Fehler beim Oeffnen von " + filename_ + ": " + std::strerror(errno));
    }

    if (ioctl(file_, I2C_SLAVE, address_) < 0) {
        close(file_);
        file_ = -1;
        throw std::runtime_error("PCA9685: Konnte I2C_SLAVE 0x" + std::to_string(address_) + " nicht setzen");
    }

    // Auto-Increment und Sleep setzen, um Prescale konfigurieren zu koennen
    writeRegister(REG_MODE1, 0x10);  // SLEEP = 1

    // Prescale berechnen: 25 MHz interner Oszillator
    double prescale_val = 25000000.0 / (4096.0 * frequency_) - 1.0;
    uint8_t prescale = static_cast<uint8_t>(std::clamp(std::round(prescale_val), 3.0, 255.0));
    writeRegister(REG_PRE_SCALE, prescale);

    // Aufwachen (SLEEP loeschen)
    writeRegister(REG_MODE1, 0x00);
    usleep(5000);  // 5 ms fuer Stabilisierung des Oszillators

    // RESTART (Bit 7), Auto-Increment (Bit 5), ALLCALL (Bit 0) aktivieren -> 0xA1
    writeRegister(REG_MODE1, 0xA1);

    // Totem-Pole (Push-Pull) Output Drive aktivieren (MODE2 Bit 2 = 1)
    writeRegister(REG_MODE2, 0x04);

    // Alle 16 Kanaele auf Full-OFF setzen, Oszillator bleibt aktiv
    for (int i = 0; i < 16; ++i) {
        setChannelFullOff(i);
    }
    return true;
}

void PCA9685::wakeUp()
{
    if (file_ < 0) {
        return;
    }
    int m1 = readRegister(REG_MODE1);
    if (m1 < 0 || (m1 & 0x10)) {
        writeRegister(REG_MODE1, 0x00);
        usleep(5000);
        writeRegister(REG_MODE1, 0xA1);
    }
}

int PCA9685::readRegister(uint8_t reg)
{
    if (file_ < 0) {
        return -1;
    }
    return i2c_smbus_read_byte_data(file_, reg);
}

bool PCA9685::writeRegister(uint8_t reg, uint8_t val)
{
    if (file_ < 0) {
        return false;
    }
    return i2c_smbus_write_byte_data(file_, reg, val) >= 0;
}

void PCA9685::setPwm(int channel, uint16_t on, uint16_t off)
{
    if (channel < 0 || channel > 15 || file_ < 0) {
        return;
    }

    wakeUp();

    uint8_t reg = REG_LED0_ON_L + 4 * channel;
    writeRegister(reg,     static_cast<uint8_t>(on & 0xFF));
    writeRegister(reg + 1, static_cast<uint8_t>((on >> 8) & 0x0F));
    writeRegister(reg + 2, static_cast<uint8_t>(off & 0xFF));
    writeRegister(reg + 3, static_cast<uint8_t>((off >> 8) & 0x0F));
}

void PCA9685::setChannelFullOn(int channel)
{
    if (channel < 0 || channel > 15 || file_ < 0) {
        return;
    }
    // Bit 4 in ON_H (0x10) bedeutet dauerhaft HIGH
    uint8_t reg = REG_LED0_ON_L + 4 * channel;
    writeRegister(reg,     0x00);
    writeRegister(reg + 1, 0x10);
    writeRegister(reg + 2, 0x00);
    writeRegister(reg + 3, 0x00);
}

void PCA9685::setChannelFullOff(int channel)
{
    if (channel < 0 || channel > 15 || file_ < 0) {
        return;
    }
    // Bit 4 in OFF_H (0x10) bedeutet dauerhaft LOW
    uint8_t reg = REG_LED0_ON_L + 4 * channel;
    writeRegister(reg,     0x00);
    writeRegister(reg + 1, 0x00);
    writeRegister(reg + 2, 0x00);
    writeRegister(reg + 3, 0x10);
}

void PCA9685::setChannelDuty(int channel, double duty)
{
    duty = std::clamp(duty, 0.0, 1.0);
    if (duty <= 0.001) {
        setChannelFullOff(channel);
    } else if (duty >= 0.999) {
        setChannelFullOn(channel);
    } else {
        uint16_t off = static_cast<uint16_t>(std::round(duty * 4095.0));
        setPwm(channel, 0, off);
    }
}

void PCA9685::setServoPulse(int channel, double pulse_us)
{
    // Bei 50 Hz betraegt die Periodendauer 20.000 us (4096 Ticks)
    double period_us = 1000000.0 / frequency_;
    uint16_t ticks = static_cast<uint16_t>(std::clamp(
        std::round(pulse_us * 4096.0 / period_us),
        0.0, 4095.0
    ));
    setPwm(channel, 0, ticks);
}

void PCA9685::setServoAngle(int channel, double angle_deg,
                            double min_pulse, double max_pulse, double range_deg)
{
    angle_deg = std::clamp(angle_deg, 0.0, range_deg);
    double pulse = min_pulse + (angle_deg / range_deg) * (max_pulse - min_pulse);
    setServoPulse(channel, pulse);
}

void PCA9685::setMotorThrottle(int pin_in1, int pin_in2, double throttle, bool slow_decay)
{
    throttle = std::clamp(throttle, -1.0, 1.0);

    if (std::abs(throttle) < 0.01) {
        setChannelFullOff(pin_in1);
        setChannelFullOff(pin_in2);
        return;
    }

    if (throttle > 0.0) {
        // Vorwaerts
        setChannelFullOn(pin_in1);
        if (slow_decay) {
            uint16_t off = static_cast<uint16_t>(std::round((1.0 - throttle) * 4095.0));
            setPwm(pin_in2, 0, off);
        } else {
            uint16_t off = static_cast<uint16_t>(std::round(throttle * 4095.0));
            setPwm(pin_in2, 0, off);
        }
    } else {
        // Rueckwaerts
        if (slow_decay) {
            uint16_t off = static_cast<uint16_t>(std::round((1.0 - std::abs(throttle)) * 4095.0));
            setPwm(pin_in1, 0, off);
        } else {
            uint16_t off = static_cast<uint16_t>(std::round(std::abs(throttle) * 4095.0));
            setPwm(pin_in1, 0, off);
        }
        setChannelFullOn(pin_in2);
    }
}

void PCA9685::allOff()
{
    if (file_ < 0) {
        return;
    }

    // 1. Alle 16 Kanaele explizit auf Full-Off schalten
    for (int i = 0; i < 16; ++i) {
        setChannelFullOff(i);
    }

    // 2. Globales ALL_LED Register auf Full-Off (Bit 4 in OFF_H = 1)
    writeRegister(REG_ALL_LED_ON_L,  0x00);
    writeRegister(REG_ALL_LED_ON_H,  0x00);
    writeRegister(REG_ALL_LED_OFF_L, 0x00);
    writeRegister(REG_ALL_LED_OFF_H, 0x10);

    // 3. Oszillator in SLEEP versetzen (MODE1 Bit 4 = 1) -> Keine PWM-Pulse mehr
    writeRegister(REG_MODE1, 0x10);
}

}  // namespace picar_drivers
