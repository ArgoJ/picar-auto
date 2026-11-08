#!/usr/bin/env python3
import sys
import time
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685


def sweep_servo(channel, min_deg=60, max_deg=120):
    i2c = busio.I2C(SCL, SDA)
    pca = PCA9685(i2c, address=0x5f)
    pca.frequency = 50

    s = servo.Servo(pca.channels[channel], min_pulse=500, max_pulse=2400, actuation_range=180)
    print(f"Starte Testlauf für Kanal CH{channel} ({min_deg}° bis {max_deg}°). Beenden mit Strg+C.")

    try:
        # Erst sanft in die Mitte des gewählten Bereichs
        mid_angle = (min_deg + max_deg) // 2
        print(f"Fahre Servo auf {mid_angle}°...")
        s.angle = mid_angle
        time.sleep(0.5)

        for cycle in range(1, 4):  # 3 Testzyklen
            print(f"Zyklus {cycle}/3...")
            # Vorwärts schwenken
            for angle in range(min_deg, max_deg + 1, 2):
                s.angle = angle
                time.sleep(0.02)
            time.sleep(0.3)

            # Rückwärts schwenken
            for angle in range(max_deg, min_deg - 1, -2):
                s.angle = angle
                time.sleep(0.02)
            time.sleep(0.3)

    except KeyboardInterrupt:
        print("\nAbbruch durch Benutzer.")
    finally:
        print("Schalte Servosignal ab...")
        pca.channels[channel].duty_cycle = 0
        pca.deinit()
        print("Kanal deaktiviert.")


if __name__ == "__main__":
    # Syntax: python3 servo_sweep.py [channel] [min_deg] [max_deg]
    ch = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    min_a = int(sys.argv[2]) if len(sys.argv) > 2 else 60
    max_a = int(sys.argv[3]) if len(sys.argv) > 3 else 120
    sweep_servo(ch, min_a, max_a)
