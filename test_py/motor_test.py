#!/usr/bin/env python3
# File name   : motor_test.py
# Description : Testskript für 2 DC-Motoren an der Hinterachse (PCA9685 / Adeept)
import sys
import time
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor

# Pinbelegung der Motoren am PCA9685
MOTOR_M1_IN1 = 15      # Positive pole M1
MOTOR_M1_IN2 = 14      # Negative pole M1
MOTOR_M2_IN1 = 12      # Positive pole M2
MOTOR_M2_IN2 = 13      # Negative pole M2
MOTOR_M3_IN1 = 11      # Positive pole M3
MOTOR_M3_IN2 = 10      # Negative pole M3
MOTOR_M4_IN1 = 8       # Positive pole M4
MOTOR_M4_IN2 = 9       # Negative pole M4

MOTOR_PINS = {
    1: {"in1": MOTOR_M1_IN1, "in2": MOTOR_M1_IN2, "name": "M1 (Hinterachse 1)"},
    2: {"in1": MOTOR_M2_IN1, "in2": MOTOR_M2_IN2, "name": "M2 (Hinterachse 2)"},
}

def map_speed(x, in_min=0, in_max=100, out_min=0.0, out_max=1.0):
    return (x - in_min) / (in_max - in_min) * (out_max - out_min) + out_min

class DualMotorController:
    def __init__(self, address=0x5f, frequency=50):
        self.i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(self.i2c, address=address)
        self.pca.frequency = frequency

        # Initialisiere die 2 Motoren der Hinterachse
        self.motors = {}
        for m_id, pins in MOTOR_PINS.items():
            dcm = motor.DCMotor(
                self.pca.channels[pins["in1"]],
                self.pca.channels[pins["in2"]]
            )
            dcm.decay_mode = motor.SLOW_DECAY
            self.motors[m_id] = dcm

    def set_motor(self, channel, direction, speed_pct):
        """
        channel: 1 oder 2
        direction: 1 (Vorwärts), -1 (Rückwärts), 0 (Stopp)
        speed_pct: 0 bis 100 (%)
        """
        if channel not in self.motors:
            return

        speed_pct = max(0, min(100, speed_pct))
        speed = map_speed(speed_pct, 0, 100, 0.0, 1.0)
        if direction == -1:
            speed = -speed
        elif direction == 0:
            speed = 0

        self.motors[channel].throttle = speed

    def stop_all(self):
        """Stoppt beide Motoren."""
        for m in self.motors.values():
            m.throttle = 0

    def cleanup(self):
        """Sicherheits-Stopp und Freigabe der I2C-Ressourcen."""
        try:
            self.stop_all()
        finally:
            self.pca.deinit()

def run_single_motor_test(ctrl, channel, speed=50, duration=2.0):
    name = MOTOR_PINS[channel]["name"]
    print(f"\n--- Teste {name} ---")

    print(f"[{name}] Vorwärts mit {speed}% für {duration}s...")
    ctrl.set_motor(channel, 1, speed)
    time.sleep(duration)

    ctrl.set_motor(channel, 0, 0)
    time.sleep(0.4)

    print(f"[{name}] Rückwärts mit {speed}% für {duration}s...")
    ctrl.set_motor(channel, -1, speed)
    time.sleep(duration)

    ctrl.set_motor(channel, 0, 0)
    time.sleep(0.5)

def run_both_motors_test(ctrl, speed=50, duration=2.0):
    print("\n--- Teste beide Motoren gleichzeitig (Hinterachse) ---")

    print(f"[M1 + M2] Vorwärts mit {speed}% für {duration}s...")
    ctrl.set_motor(1, 1, speed)
    ctrl.set_motor(2, 1, speed)
    time.sleep(duration)

    ctrl.stop_all()
    time.sleep(0.4)

    print(f"[M1 + M2] Rückwärts mit {speed}% für {duration}s...")
    ctrl.set_motor(1, -1, speed)
    ctrl.set_motor(2, -1, speed)
    time.sleep(duration)

    ctrl.stop_all()
    time.sleep(0.5)

def main():
    # Modi: '1', '2', 'both', 'all' (Standard)
    mode = "all"
    cycles = 1

    if len(sys.argv) > 1:
        arg = sys.argv[1].lower()
        if arg in ("1", "2", "both", "all", "alle"):
            mode = arg
        else:
            print("Verwendung: python3 motor_test.py [1 | 2 | both | all] [zyklen]")
            print("  1    : Testet nur Motor M1")
            print("  2    : Testet nur Motor M2")
            print("  both : Testet M1 und M2 gleichzeitig (Vorwärts/Rückwärts)")
            print("  all  : Testet M1 einzeln, M2 einzeln und danach beide zusammen")
            sys.exit(1)

    if len(sys.argv) > 2 and sys.argv[2].isdigit():
        cycles = int(sys.argv[2])

    ctrl = DualMotorController()
    print("=" * 60)
    print("Starte 2-Motoren-Test für Hinterachse (PCA9685 @ 0x5F)")
    print(f"Modus: {mode} | Zyklen: {cycles}")
    print("Beenden jederzeit mit Strg+C.")
    print("=" * 60)

    try:
        for c in range(1, cycles + 1):
            if cycles > 1:
                print(f"\n>>>>>> ZYKLUS {c}/{cycles} <<<<<<")

            if mode == "1":
                run_single_motor_test(ctrl, 1)
            elif mode == "2":
                run_single_motor_test(ctrl, 2)
            elif mode == "both":
                run_both_motors_test(ctrl)
            else:  # "all"
                run_single_motor_test(ctrl, 1)
                run_single_motor_test(ctrl, 2)
                run_both_motors_test(ctrl)

        print("\nTest erfolgreich abgeschlossen.")

    except KeyboardInterrupt:
        print("\nAbbruch durch Benutzer (Strg+C).")
    finally:
        print("Sicherheits-Stopp: Schalte Motoren ab und gebe PCA9685 frei...")
        ctrl.cleanup()
        print("Fertig.")

if __name__ == "__main__":
    main()
