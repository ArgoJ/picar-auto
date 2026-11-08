#!/usr/bin/env python3
# File name   : kalibration_service_node.py
# Description : Zentraler Kalibrierungs-Service fuer PiCar Servos und Motoren
import os
import re
import sys
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as MsgParameter, ParameterType, ParameterValue
from std_msgs.msg import Float32MultiArray

from picar_kalibration.srv import (
    GetServoCalibration,
    SaveCalibration,
    SetServoCalibration,
)

DEFAULT_CONFIG_PATH = "/home/pirob/ros2_ws/src/picar_drivers/config/picar_params.yaml"


class KalibrationServiceNode(Node):
    def __init__(self):
        super().__init__("kalibration_service_node")

        self.declare_parameter("config_file", DEFAULT_CONFIG_PATH)
        self.config_path = self.get_parameter("config_file").value

        # 3-fache Arrays fuer alle 16 Kanaele
        self.offsets = [0.0] * 16
        self.mins = [60.0, 10.0, 30.0, 30.0, 50.0] + [0.0] * 11
        self.maxs = [120.0, 170.0, 150.0, 150.0, 130.0] + [180.0] * 11

        self.load_from_yaml(self.config_path)

        # Services bereitstellen
        self.srv_set = self.create_service(
            SetServoCalibration,
            "/picar_kalibration/set_servo",
            self.handle_set_servo
        )
        self.srv_get = self.create_service(
            GetServoCalibration,
            "/picar_kalibration/get_servo",
            self.handle_get_servo
        )
        self.srv_save = self.create_service(
            SaveCalibration,
            "/picar_kalibration/save_to_yaml",
            self.handle_save
        )

        # Publisher fuer Test-Fahrten
        self.pub_servo = self.create_publisher(Float32MultiArray, "picar/servo/cmd", 10)

        # Parameter-Client fuer MotorNode
        self.cli_set_params = self.create_client(SetParameters, "/motor_node/set_parameters")

        self.get_logger().info(
            f"KalibrationServiceNode aktiv. Config: {self.config_path}. Services unter /picar_kalibration/* verfuegbar."
        )

    def load_from_yaml(self, path: str):
        """Laedt die bestehenden Kalibrierungs-Arrays aus der YAML-Datei."""
        if not os.path.exists(path):
            self.get_logger().warn(f"Config-Datei {path} nicht gefunden, verwende Standardwerte.")
            return

        try:
            with open(path, "r", encoding="utf-8") as f:
                content = f.read()

            def extract_list(name: str):
                match = re.search(rf"{name}\s*:\s*\[(.*?)\]", content)
                if match:
                    items = [float(x.strip()) for x in match.group(1).split(",") if x.strip()]
                    if len(items) == 16:
                        return items
                return None

            offs = extract_list("servo_offsets")
            mins = extract_list("servo_mins")
            maxs = extract_list("servo_maxs")

            if offs:
                self.offsets = offs
            if mins:
                self.mins = mins
            if maxs:
                self.maxs = maxs

            self.get_logger().info(f"Kalibrierung erfolgreich aus {path} geladen.")
        except Exception as e:
            self.get_logger().error(f"Fehler beim Laden von {path}: {e}")

    def sync_to_motor_node(self):
        """Aktualisiert die Parameter in /motor_node via ROS 2 SetParameters Service."""
        if not self.cli_set_params.service_is_ready():
            return False

        req = SetParameters.Request()
        p_off = MsgParameter()
        p_off.name = "servo_offsets"
        p_off.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=self.offsets)

        p_min = MsgParameter()
        p_min.name = "servo_mins"
        p_min.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=self.mins)

        p_max = MsgParameter()
        p_max.name = "servo_maxs"
        p_max.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=self.maxs)

        req.parameters = [p_off, p_min, p_max]
        self.cli_set_params.call_async(req)
        return True

    def handle_set_servo(self, request, response):
        ch = request.channel
        if ch < 0 or ch >= 16:
            response.success = False
            response.message = f"Ungueltiger Servokanal: {ch} (muss 0..15 sein)"
            return response

        self.offsets[ch] = float(request.offset)
        self.mins[ch] = float(request.min_angle)
        self.maxs[ch] = float(request.max_angle)

        self.sync_to_motor_node()

        response.success = True
        response.message = f"Kanal CH{ch} aktualisiert: Offset={self.offsets[ch]:.1f}, Min={self.mins[ch]:.1f}, Max={self.maxs[ch]:.1f}"
        self.get_logger().info(response.message)
        return response

    def handle_get_servo(self, request, response):
        ch = request.channel
        if ch < 0 or ch >= 16:
            response.success = False
            response.message = f"Ungueltiger Servokanal: {ch}"
            return response

        response.offset = self.offsets[ch]
        response.min_angle = self.mins[ch]
        response.max_angle = self.maxs[ch]
        response.success = True
        response.message = f"Kanal CH{ch} Werte geliefert."
        return response

    def handle_save(self, request, response):
        target_path = request.file_path if request.file_path else self.config_path
        try:
            if not os.path.exists(target_path):
                response.success = False
                response.message = f"Zieldatei {target_path} existiert nicht."
                return response

            with open(target_path, "r", encoding="utf-8") as f:
                content = f.read()

            def fmt_list(arr):
                return "[" + ", ".join(f"{x:.1f}" for x in arr) + "]"

            content = re.sub(
                r"servo_offsets\s*:\s*\[.*?\]",
                f"servo_offsets: {fmt_list(self.offsets)}",
                content
            )
            content = re.sub(
                r"servo_mins\s*:\s*\[.*?\]",
                f"servo_mins:    {fmt_list(self.mins)}",
                content
            )
            content = re.sub(
                r"servo_maxs\s*:\s*\[.*?\]",
                f"servo_maxs:    {fmt_list(self.maxs)}",
                content
            )

            with open(target_path, "w", encoding="utf-8") as f:
                f.write(content)

            response.success = True
            response.message = f"Kalibrierung erfolgreich in {target_path} gespeichert."
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Fehler beim Speichern: {e}"
            self.get_logger().error(response.message)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = KalibrationServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
