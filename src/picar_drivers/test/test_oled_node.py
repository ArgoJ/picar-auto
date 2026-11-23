#!/usr/bin/env python3
# File name   : test_oled_node.py
# Description : Tests for picar_drivers OledNode (SSD1306 I2C OLED display & robot eyes)
import os
import sys
import time
import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

sys.path.insert(0, os.path.dirname(__file__))
from common import (  # noqa: E402
    start_node_process,
    stop_node_process,
    wait_for_subscribers,
)


def test_oled_node_lifecycle(ros_context):
    """Verify startup, continuous eye animation run, and clean shutdown of OledNode."""
    proc = start_node_process("oled_node")
    try:
        # Let eye animation run briefly
        time.sleep(2.0)
        assert proc.poll() is None, "OledNode crashed during default eye animation!"
    finally:
        stop_node_process(proc)


def test_oled_node_text_and_eyes_switch(ros_context):
    """Verify switching from default eyes to text via picar/oled_text and back to eyes."""
    proc = start_node_process("oled_node")
    test_node = Node("test_oled_client")
    pub = test_node.create_publisher(String, "picar/oled_text", 10)

    try:
        assert wait_for_subscribers(pub, timeout=3.0), "OledNode did not subscribe to picar/oled_text!"

        msg = String()
        msg.data = "PiCar ROS 2\nReady!"
        pub.publish(msg)
        rclpy.spin_once(test_node, timeout_sec=0.2)
        time.sleep(0.5)
        assert proc.poll() is None, "OledNode crashed after rendering text!"

        # Empty string switches back to eye animation
        msg.data = ""
        pub.publish(msg)
        rclpy.spin_once(test_node, timeout_sec=0.2)
        time.sleep(0.5)
        assert proc.poll() is None, "OledNode crashed when switching back to eye mode!"

    finally:
        test_node.destroy_node()
        stop_node_process(proc)


def test_oled_node_low_battery_alarm(ros_context):
    """Verify blinking low-battery display activation and recovery via picar/battery/low_alarm."""
    proc = start_node_process("oled_node")
    test_node = Node("test_oled_battery_client")
    pub = test_node.create_publisher(Bool, "picar/battery/low_alarm", 10)

    try:
        assert wait_for_subscribers(pub, timeout=3.0), "OledNode did not subscribe to picar/battery/low_alarm!"

        pub.publish(Bool(data=True))
        rclpy.spin_once(test_node, timeout_sec=0.1)
        time.sleep(1.1)
        assert proc.poll() is None, "OledNode crashed while blinking low battery warning!"

        pub.publish(Bool(data=False))
        rclpy.spin_once(test_node, timeout_sec=0.1)
        time.sleep(0.4)
        assert proc.poll() is None, "OledNode crashed when clearing low battery warning!"

    finally:
        test_node.destroy_node()
        stop_node_process(proc)


if __name__ == "__main__":
    pytest.main(["-v", __file__])
