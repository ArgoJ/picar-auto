#!/usr/bin/env python3
import sys
import time
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

def center_servo(channel):
    # I2C-Verbindung zum PCA9685 (Adresse 0x5f, 50 Hz)
    i2c = busio.I2C(SCL, SDA)
    pca = PCA9685(i2c, address=0x5f)
    pca.frequency = 50

    print(f"Bewege Servo auf Kanal CH{channel} in 90°-Neutralstellung...")
    # Standard-Pulsweiten des Roboters: 500 µs (0°) bis 2400 µs (180°)
    s = servo.Servo(pca.channels[channel], min_pulse=500, max_pulse=2400, actuation_range=180)
    s.angle = 90
    time.sleep(1.0)

    # Signal abschalten, damit der Servo stromlos bleibt und nicht summt/erhitzt
    pca.channels[channel].duty_cycle = 0
    pca.deinit()
    print("Fertig.")

if __name__ == "__main__":
    ch = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    center_servo(ch)