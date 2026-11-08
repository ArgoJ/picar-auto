#!/usr/bin/env python3
# File name   : test_ultrasonic_node.py
# Description : Tests fuer die picar_drivers UltrasonicNode
import os
import sys
import time
import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

sys.path.insert(0, os.path.dirname(__file__))
from common import (  # noqa: E402
    start_node_process,
    stop_node_process,
)


def test_ultrasonic_node_lifecycle(ros_context):
    """Prueft Start und sauberes Beenden der UltrasonicNode."""
    proc = start_node_process("ultrasonic_node")
    try:
        assert proc.poll() is None, "UltrasonicNode laeuft nicht!"
    finally:
        stop_node_process(proc)


def test_ultrasonic_node_publishing_and_metadata(ros_context):
    """Prueft kontinuierliches Publizieren und REP-117 Konformitaet der Range-Nachrichten."""
    proc = start_node_process("ultrasonic_node")
    test_node = Node("test_ultrasonic_client")

    try:
        received_ranges = []
        test_node.create_subscription(
            Range,
            "picar/ultrasonic/range",
            lambda m: received_ranges.append(m),
            10
        )

        # Warten auf mindestens 3 Range-Messungen
        t_end = time.time() + 4.0
        while time.time() < t_end and len(received_ranges) < 3:
            rclpy.spin_once(test_node, timeout_sec=0.1)

        assert len(received_ranges) >= 3, f"Zu wenige Range-Nachrichten empfangen: {len(received_ranges)}"

        sample = received_ranges[-1]
        # REP 117 Konformitaet
        assert sample.header.frame_id == "ultrasonic_link", f"Falsche frame_id: {sample.header.frame_id}"
        assert sample.radiation_type == Range.ULTRASOUND, "radiation_type muss Range.ULTRASOUND sein!"
        assert sample.min_range == pytest.approx(0.02, abs=0.005), f"Falsche min_range: {sample.min_range}"
        assert sample.max_range == pytest.approx(4.00, abs=0.005), f"Falsche max_range: {sample.max_range}"
        assert sample.field_of_view > 0.0, "field_of_view muss positiv sein!"

        # Gueltiger Messwert (entweder positive Distanz oder +Inf fuer freie Bahn)
        is_valid = (sample.range > 0.0) or (sample.range == float("inf"))
        assert is_valid, f"Ungueltige Distanz gemessen: {sample.range}"

        # Zeitstempel vorhanden
        assert sample.header.stamp.sec > 0 or sample.header.stamp.nanosec > 0

    finally:
        test_node.destroy_node()
        stop_node_process(proc)


if __name__ == "__main__":
    pytest.main(["-v", __file__])
