#!/usr/bin/env python3
# File name   : all_off.py
# Description : Hardware-Not-Aus: Schaltet garantiert alle 16 PCA9685 PWM-Kanaele (Servos & DC-Motoren) stromlos.
import os
import sys
import fcntl

I2C_SLAVE = 0x0703
PCA9685_ADDR = 0x5F
I2C_DEV = "/dev/i2c-1"

def all_off():
    try:
        fd = os.open(I2C_DEV, os.O_RDWR)
        fcntl.ioctl(fd, I2C_SLAVE, PCA9685_ADDR)

        def write_reg(reg: int, val: int):
            os.write(fd, bytes([reg & 0xFF, val & 0xFF]))

        # 1. Alle 16 Kanaele einzeln auf Full-OFF setzen (Bit 4 in OFF_H = 1)
        for ch in range(16):
            base = 0x06 + 4 * ch
            write_reg(base,     0x00)  # ON_L
            write_reg(base + 1, 0x00)  # ON_H
            write_reg(base + 2, 0x00)  # OFF_L
            write_reg(base + 3, 0x10)  # OFF_H (Bit 4 = Full OFF)

        # 2. Globales Register ALL_LED auf Full-OFF
        write_reg(0xFA, 0x00)  # ALL_LED_ON_L
        write_reg(0xFB, 0x00)  # ALL_LED_ON_H
        write_reg(0xFC, 0x00)  # ALL_LED_OFF_L
        write_reg(0xFD, 0x10)  # ALL_LED_OFF_H (Full OFF)

        # 3. PCA9685 in SLEEP versetzen (MODE1 Bit 4 = 1) -> Schaltet den internen 25MHz Oszillator komplett ab!
        write_reg(0x00, 0x10)

        os.close(fd)
        print("✅ Erfolgreich: Alle 16 Kanaele (Servos & Motoren) sind 100% stromlos und Oszillator ist im SLEEP.")
        return True
    except Exception as e:
        print(f"❌ Fehler beim Abschalten: {e}", file=sys.stderr)
        return False

if __name__ == "__main__":
    success = all_off()
    sys.exit(0 if success else 1)
