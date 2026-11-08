#!/usr/bin/env python3
# File name   : mpu6050_test.py
# Description : Testskript für den MPU-6050 Sensor (Beschleunigung, Gyroskop, Temperatur)
import sys
import time
from mpu6050 import mpu6050


def main():
    print("=" * 65)
    print("MPU-6050 Sensor Test (I2C Adresse: 0x68)")
    print("Beenden mit Strg+C.")
    print("=" * 65)

    try:
        sensor = mpu6050(0x68)
    except Exception as e:
        print(f"Fehler beim Verbinden mit dem MPU-6050: {e}")
        print("Bitte prüfen, ob I2C aktiviert und der Sensor angebunden ist.")
        sys.exit(1)

    print(f"{'Beschleunigung (m/s²)':<28} | {'Gyroskop (°/s)':<25} | {'Temp':<7}")
    print(f"{'X':>8} {'Y':>9} {'Z':>9}   | {'X':>7} {'Y':>8} {'Z':>8}   | {'(°C)':>6}")
    print("-" * 65)

    try:
        while True:
            # 10 Messwerte mitteln für eine ruhige Ausgabe
            accel_x, accel_y, accel_z = 0.0, 0.0, 0.0
            gyro_x, gyro_y, gyro_z = 0.0, 0.0, 0.0

            samples = 10
            for _ in range(samples):
                accel = sensor.get_accel_data()
                gyro = sensor.get_gyro_data()

                accel_x += accel['x']
                accel_y += accel['y']
                accel_z += accel['z']

                gyro_x += gyro['x']
                gyro_y += gyro['y']
                gyro_z += gyro['z']
                time.sleep(0.01)

            accel_x /= samples
            accel_y /= samples
            accel_z /= samples
            gyro_x /= samples
            gyro_y /= samples
            gyro_z /= samples

            temp = sensor.get_temp()

            print(
                f"{accel_x:8.2f} {accel_y:9.2f} {accel_z:9.2f}   | "
                f"{gyro_x:7.2f} {gyro_y:8.2f} {gyro_z:8.2f}   | "
                f"{temp:6.1f}°C"
            )
            time.sleep(0.2)

    except KeyboardInterrupt:
        print("\n\nTestlauf durch Benutzer beendet.")


if __name__ == "__main__":
    main()
