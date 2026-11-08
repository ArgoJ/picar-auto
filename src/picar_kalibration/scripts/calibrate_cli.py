#!/usr/bin/env python3
# File name   : calibrate_cli.py
# Description : Interaktives CLI-Tool zur Kalibrierung der Servos (Lenkung, Arm, Greifer)
import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32

from picar_kalibration.srv import (
    GetServoCalibration,
    SaveCalibration,
    SetServoCalibration,
)

CHANNEL_NAMES = {
    0: "Lenkservo Vorderachse",
    1: "Roboterarm Basis (Pan)",
    2: "Roboterarm Schulter (Tilt)",
    3: "Roboterarm Ellbogen / Lift",
    4: "Greifer (AD002 Micro-Servo)",
}


class CalibrateCli(Node):
    def __init__(self):
        super().__init__("calibrate_cli")
        self.cli_set = self.create_client(SetServoCalibration, "/picar_kalibration/set_servo")
        self.cli_get = self.create_client(GetServoCalibration, "/picar_kalibration/get_servo")
        self.cli_save = self.create_client(SaveCalibration, "/picar_kalibration/save_to_yaml")
        self.pub_servo = self.create_publisher(Float32MultiArray, "picar/servo/cmd", 10)
        self.pub_release = self.create_publisher(Int32, "picar/servo/release", 10)

    def wait_services(self, timeout=3.0) -> bool:
        return (
            self.cli_set.wait_for_service(timeout_sec=timeout) and
            self.cli_get.wait_for_service(timeout_sec=timeout) and
            self.cli_save.wait_for_service(timeout_sec=timeout)
        )

    def check_motor_driver(self, timeout=1.5) -> bool:
        """Prueft, ob der Hardware-Treiber motor_node auf das Servo-Topic lauscht."""
        start = time.time()
        while time.time() - start < timeout:
            if self.pub_servo.get_subscription_count() > 0:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def release_servo(self, channel: int):
        msg = Int32()
        msg.data = channel
        self.pub_release.publish(msg)

    def get_channel_info(self, channel: int):
        req = GetServoCalibration.Request()
        req.channel = channel
        future = self.cli_get.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        return future.result()

    def set_channel(self, channel: int, offset: float, min_angle: float, max_angle: float):
        req = SetServoCalibration.Request()
        req.channel = channel
        req.offset = float(offset)
        req.min_angle = float(min_angle)
        req.max_angle = float(max_angle)
        future = self.cli_set.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        return future.result()

    def test_servo(self, channel: int, angle: float):
        msg = Float32MultiArray()
        msg.data = [float(channel), float(angle)]
        self.pub_servo.publish(msg)

    def save_calibration(self):
        req = SaveCalibration.Request()
        req.file_path = ""
        future = self.cli_save.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        return future.result()


def main():
    rclpy.init()
    cli = CalibrateCli()

    print("=" * 65)
    print("      PiCar Servo- & Greifer-Kalibrierung (picar_kalibration)")
    print("=" * 65)

    if not cli.wait_services(timeout=2.0):
        print("❌ FEHLER: /picar_kalibration Services nicht erreichbar!")
        print("Bitte starte den Kalibrierungsservice:")
        print("  ros2 run picar_kalibration kalibration_service_node")
        print("=" * 65)
        rclpy.shutdown()
        sys.exit(1)

    has_driver = cli.check_motor_driver(timeout=1.0)
    if not has_driver:
        print("\n⚠️  ACHTUNG: Der Hardware-Treiber 'motor_node' laeuft noch nicht!")
        print("   Auf 'picar/servo/cmd' lauscht aktuell kein Node.")
        print("   Die Servos bewegen sich erst, wenn der Treiber aktiv ist:")
        print("   👉 Terminal: ros2 run picar_drivers motor_node")
        print("   👉 Oder alles zusammen: ros2 launch picar_kalibration kalibration.launch.py\n")

    channel = 0
    if len(sys.argv) > 1 and sys.argv[1].isdigit():
        channel = int(sys.argv[1])

    info = cli.get_channel_info(channel)
    offset = info.offset if info else 0.0
    min_a = info.min_angle if info else 0.0
    max_a = info.max_angle if info else 180.0

    name = CHANNEL_NAMES.get(channel, f"Kanal {channel}")
    print(f"\nGewaehlter Kanal: CH{channel} ({name})")
    print(f"Aktuelle Werte  : Offset={offset:.1f}°, Min={min_a:.1f}°, Max={max_a:.1f}°")
    print("\nBefehle:")
    print("  [Zahl]      : Fahre Servo auf Sollwinkel (z. B. '90' oder '75')")
    print("  center/mid  : Fahre auf 90° Neutralstellung")
    print("  min         : Fahre auf Min-Grenze zum Pruefen")
    print("  max         : Fahre auf Max-Grenze zum Pruefen")
    print("  min [Zahl]  : Min-Grenze setzen & direkt anfahren (z. B. 'min 65')")
    print("  max [Zahl]  : Max-Grenze setzen & direkt anfahren (z. B. 'max 125')")
    print("  + / -       : Offset um +1° oder -1° anpassen (faehrt 90° Neutral)")
    print("  ++ / --     : Offset um +5° oder -5° anpassen")
    print("  off / rel   : Aktuellen Kanal stromlos schalten (PWM aus)")
    print("  alloff      : Alle 16 Kanaele stromlos schalten")
    print("  ch [0-15]   : Kanal wechseln")
    print("  s           : In YAML-Datei dauerhaft speichern")
    print("  q           : Beenden")
    print("-" * 65)

    def calc_hw(soll_deg: float) -> float:
        clamped = max(min_a, min(max_a, soll_deg))
        return max(0.0, min(180.0, clamped + offset))

    # Initial auf 90 Grad fahren
    print(f"Fahre initial auf 90.0° Neutralstellung (Hardware {calc_hw(90.0):.1f}°)...")
    cli.test_servo(channel, 90.0)

    try:
        while True:
            cmd = input(f"[CH{channel} | Off:{offset:+.1f} | {min_a:.0f}..{max_a:.0f}] > ").strip()
            if not cmd:
                continue

            if cmd == "q":
                break
            elif cmd == "s":
                res = cli.save_calibration()
                if res and res.success:
                    print(f"-> {res.message}")
                else:
                    print("-> Fehler beim Speichern.")
            elif cmd in ("center", "mid"):
                hw = calc_hw(90.0)
                print(f"-> Fahre Neutralstellung 90.0° (Hardware {hw:.1f}°)...")
                cli.test_servo(channel, 90.0)
            elif cmd == "min":
                hw = calc_hw(min_a)
                print(f"-> Fahre Min-Grenze Soll {min_a:.1f}° (Hardware {hw:.1f}°)...")
                cli.test_servo(channel, min_a)
            elif cmd == "max":
                hw = calc_hw(max_a)
                print(f"-> Fahre Max-Grenze Soll {max_a:.1f}° (Hardware {hw:.1f}°)...")
                cli.test_servo(channel, max_a)
            elif cmd in ("off", "rel", "release"):
                cli.release_servo(channel)
                print(f"-> CH{channel} stromlos geschaltet (PWM-Signal aus).")
            elif cmd in ("alloff", "off all"):
                cli.release_servo(-1)
                print("-> Alle 16 Kanaele (Servos & Motoren) stromlos geschaltet!")
            elif cmd == "+":
                offset += 1.0
                cli.set_channel(channel, offset, min_a, max_a)
                hw = calc_hw(90.0)
                print(f"Offset neu: {offset:+.1f}° -> Hardware {hw:.1f}° bei 90° Soll")
                cli.test_servo(channel, 90.0)
            elif cmd == "-":
                offset -= 1.0
                cli.set_channel(channel, offset, min_a, max_a)
                hw = calc_hw(90.0)
                print(f"Offset neu: {offset:+.1f}° -> Hardware {hw:.1f}° bei 90° Soll")
                cli.test_servo(channel, 90.0)
            elif cmd == "++":
                offset += 5.0
                cli.set_channel(channel, offset, min_a, max_a)
                hw = calc_hw(90.0)
                print(f"Offset neu: {offset:+.1f}° -> Hardware {hw:.1f}° bei 90° Soll")
                cli.test_servo(channel, 90.0)
            elif cmd == "--":
                offset -= 5.0
                cli.set_channel(channel, offset, min_a, max_a)
                hw = calc_hw(90.0)
                print(f"Offset neu: {offset:+.1f}° -> Hardware {hw:.1f}° bei 90° Soll")
                cli.test_servo(channel, 90.0)
            elif cmd.startswith("min "):
                parts = cmd.split()
                if len(parts) == 2:
                    min_a = float(parts[1])
                    cli.set_channel(channel, offset, min_a, max_a)
                    hw = calc_hw(min_a)
                    print(f"-> Min-Grenze auf {min_a:.1f}° gesetzt. Fahre zur Kontrolle (Hardware {hw:.1f}°)...")
                    cli.test_servo(channel, min_a)
            elif cmd.startswith("max "):
                parts = cmd.split()
                if len(parts) == 2:
                    max_a = float(parts[1])
                    cli.set_channel(channel, offset, min_a, max_a)
                    hw = calc_hw(max_a)
                    print(f"-> Max-Grenze auf {max_a:.1f}° gesetzt. Fahre zur Kontrolle (Hardware {hw:.1f}°)...")
                    cli.test_servo(channel, max_a)
            elif cmd.startswith("ch "):
                parts = cmd.split()
                if len(parts) == 2 and parts[1].isdigit():
                    channel = int(parts[1])
                    info = cli.get_channel_info(channel)
                    offset = info.offset if info else 0.0
                    min_a = info.min_angle if info else 0.0
                    max_a = info.max_angle if info else 180.0
                    name = CHANNEL_NAMES.get(channel, f"Kanal {channel}")
                    print(f"Gewechselt zu: CH{channel} ({name})")
                    cli.test_servo(channel, 90.0)
            else:
                try:
                    val = float(cmd)
                    hw = calc_hw(val)
                    print(f"-> Fahre CH{channel} auf Soll {val:.1f}° | Hardware {hw:.1f}° (Offset {offset:+.1f}°)...")
                    cli.test_servo(channel, val)
                except ValueError:
                    print("Unbekannter Befehl. Zahl fuer Winkel oder min, max, center, +, -, s, q eingeben.")

    except (KeyboardInterrupt, EOFError):
        print("\nBeende Kalibrierung.")
    finally:
        try:
            # Alle Servos stromlos schalten beim Verlassen der CLI
            cli.release_servo(-1)
            rclpy.spin_once(cli, timeout_sec=0.1)
        except Exception:
            pass
        cli.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
