#!/usr/bin/env python3
# File name   : test_battery_node.py
# Description : Tests for picar_drivers BatteryNode (ADS7830 battery status)
import os
import sys
import time
import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool

sys.path.insert(0, os.path.dirname(__file__))
from common import (  # noqa: E402
    start_node_process,
    stop_node_process,
)


def test_battery_node_lifecycle(ros_context):
    """Verify startup and clean shutdown of BatteryNode."""
    proc = start_node_process("battery_node")
    try:
        assert proc.poll() is None, "BatteryNode is not running!"
    finally:
        stop_node_process(proc)


def test_battery_node_publishing(ros_context):
    """Verify publishing on picar/battery/state and picar/battery/low_alarm."""
    proc = start_node_process("battery_node")
    test_node = Node("test_battery_client")

    try:
        received_states = []
        received_alarms = []
        test_node.create_subscription(
            BatteryState,
            "picar/battery/state",
            lambda m: received_states.append(m),
            10
        )
        test_node.create_subscription(
            Bool,
            "picar/battery/low_alarm",
            lambda m: received_alarms.append(m),
            10
        )

        # Wait for at least 2 messages (default rate: 2.0 Hz)
        t_end = time.time() + 5.0
        while time.time() < t_end and (len(received_states) < 2 or len(received_alarms) < 2):
            rclpy.spin_once(test_node, timeout_sec=0.1)

        assert len(received_states) >= 2, f"Too few BatteryState messages: {len(received_states)}"
        assert len(received_alarms) >= 2, f"Too few alarm messages: {len(received_alarms)}"

        sample = received_states[-1]
        assert sample.header.frame_id == "battery_link"
        assert sample.header.stamp.sec > 0 or sample.header.stamp.nanosec > 0
        # Voltage measurement with 5V ADC ref and 1:4 divider is always within 0.0 to 20.5 V
        assert 0.0 <= sample.voltage <= 20.5, f"Implausible voltage: {sample.voltage} V"
        assert 0.0 <= sample.percentage <= 1.0, f"Percentage out of [0, 1] range: {sample.percentage}"
        assert sample.power_supply_technology == BatteryState.POWER_SUPPLY_TECHNOLOGY_LION

    finally:
        test_node.destroy_node()
        stop_node_process(proc)


if __name__ == "__main__":
    pytest.main(["-v", __file__])
