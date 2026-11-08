#!/usr/bin/env python3
# File name   : test_mpu6050_node.py
# Description : Tests fuer die picar_drivers Mpu6050Node (IMU & Temperatur)
import math
import os
import sys
import time
import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Temperature

sys.path.insert(0, os.path.dirname(__file__))
from common import (  # noqa: E402
    start_node_process,
    stop_node_process,
)


def test_mpu6050_node_lifecycle(ros_context):
    """Prueft Start und sauberes Beenden der Mpu6050Node."""
    proc = start_node_process("mpu6050_node")
    try:
        assert proc.poll() is None, "Mpu6050Node laeuft nicht!"
    finally:
        stop_node_process(proc)


def test_mpu6050_node_imu_publishing(ros_context):
    """Prueft kontinuierliches Publizieren und Datenqualitaet auf picar/imu/data."""
    proc = start_node_process("mpu6050_node")
    test_node = Node("test_mpu6050_imu_client")

    try:
        received_imus = []
        test_node.create_subscription(
            Imu,
            "picar/imu/data",
            lambda m: received_imus.append(m),
            10
        )

        # Warten auf mindestens 3 IMU-Nachrichten (Node publiziert standardmaessig mit 50 Hz)
        t_end = time.time() + 5.0
        while time.time() < t_end and len(received_imus) < 3:
            rclpy.spin_once(test_node, timeout_sec=0.1)

        assert len(received_imus) >= 3, f"Zu wenige IMU-Nachrichten empfangen: {len(received_imus)}"

        sample = received_imus[-1]
        assert sample.header.frame_id == "imu_link", f"Falsche frame_id: {sample.header.frame_id}"
        assert sample.header.stamp.sec > 0 or sample.header.stamp.nanosec > 0, "Zeitstempel nicht gesetzt!"

        # REP 145 Konformitaet: Kein interner Orientierungsfilter im 6-Achs-Treiber
        assert sample.orientation_covariance[0] == pytest.approx(-1.0), "orientation_covariance[0] muss -1.0 sein"

        # Beschleunigungswerte plausibel (im Ruhezustand nahezur 0 m/s^2 nach Kalibrierung oder ~9.81 m/s^2)
        ax = sample.linear_acceleration.x
        ay = sample.linear_acceleration.y
        az = sample.linear_acceleration.z
        accel_mag = math.sqrt(ax * ax + ay * ay + az * az)
        assert accel_mag < 25.0, f"Unplausibel hohe Beschleunigung gemessen: {accel_mag} m/s^2"

        # Kovarianzen positiv
        assert sample.linear_acceleration_covariance[0] > 0.0
        assert sample.linear_acceleration_covariance[4] > 0.0
        assert sample.linear_acceleration_covariance[8] > 0.0

        # Drehraten im Stillstand gering (< 1.0 rad/s)
        gx = sample.angular_velocity.x
        gy = sample.angular_velocity.y
        gz = sample.angular_velocity.z
        gyro_mag = math.sqrt(gx * gx + gy * gy + gz * gz)
        assert gyro_mag < 1.0, f"Unplausibel hohe Drehrate im Stillstand: {gyro_mag} rad/s"

        assert sample.angular_velocity_covariance[0] > 0.0
        assert sample.angular_velocity_covariance[4] > 0.0
        assert sample.angular_velocity_covariance[8] > 0.0

    finally:
        test_node.destroy_node()
        stop_node_process(proc)


def test_mpu6050_node_temperature_publishing(ros_context):
    """Prueft das Publizieren der Chip-Temperatur auf picar/imu/temperature."""
    proc = start_node_process("mpu6050_node")
    test_node = Node("test_mpu6050_temp_client")

    try:
        received_temps = []
        test_node.create_subscription(
            Temperature,
            "picar/imu/temperature",
            lambda m: received_temps.append(m),
            10
        )

        t_end = time.time() + 5.0
        while time.time() < t_end and len(received_temps) < 2:
            rclpy.spin_once(test_node, timeout_sec=0.1)

        assert len(received_temps) >= 2, f"Zu wenige Temperatur-Nachrichten empfangen: {len(received_temps)}"

        sample = received_temps[-1]
        assert sample.header.frame_id == "imu_link"
        # Plausibler Betriebstemperaturbereich (10 bis 85 Grad Celsius)
        assert 10.0 <= sample.temperature <= 85.0, f"Unplausible Chiptemperatur: {sample.temperature} C"
        assert sample.variance > 0.0, "Temperatur-Varianz muss groesser 0 sein"

    finally:
        test_node.destroy_node()
        stop_node_process(proc)


if __name__ == "__main__":
    pytest.main(["-v", __file__])
