#!/usr/bin/env python3
# File name   : test_buzzer_node.py
# Description : Tests fuer die picar_drivers BuzzerNode
import os
import sys
import time
import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32

sys.path.insert(0, os.path.dirname(__file__))
from common import (  # noqa: E402
    start_node_process,
    stop_node_process,
    wait_for_subscribers,
)


def test_buzzer_node_lifecycle(ros_context):
    """Prueft Start und sauberes Beenden der BuzzerNode."""
    proc = start_node_process("buzzer_node")
    try:
        assert proc.poll() is None, "BuzzerNode laeuft nicht!"
    finally:
        stop_node_process(proc)


def test_buzzer_node_beep_and_freq(ros_context):
    """Testet Beep-Dauer- und Frequenzkommandos."""
    proc = start_node_process("buzzer_node")
    test_node = Node("test_buzzer_client")

    try:
        pub_beep = test_node.create_publisher(Int32, "picar/buzzer/beep", 10)
        pub_freq = test_node.create_publisher(Int32, "picar/buzzer/freq", 10)
        pub_cmd = test_node.create_publisher(Bool, "picar/buzzer/cmd", 10)

        assert wait_for_subscribers(pub_beep), "buzzer_node hat pub_beep nicht abonniert"
        assert wait_for_subscribers(pub_freq), "buzzer_node hat pub_freq nicht abonniert"
        assert wait_for_subscribers(pub_cmd), "buzzer_node hat pub_cmd nicht abonniert"

        # 1. Beep-Kommando (30 ms)
        beep_msg = Int32()
        beep_msg.data = 30
        pub_beep.publish(beep_msg)
        for _ in range(5):
            rclpy.spin_once(test_node, timeout_sec=0.05)

        # 2. Frequenz setzen (440 Hz Ton A)
        freq_msg = Int32()
        freq_msg.data = 440
        pub_freq.publish(freq_msg)
        for _ in range(5):
            rclpy.spin_once(test_node, timeout_sec=0.05)

        # 3. Frequenz 0 = Stummschalten
        freq_msg.data = 0
        pub_freq.publish(freq_msg)
        for _ in range(5):
            rclpy.spin_once(test_node, timeout_sec=0.05)

        # 4. Bool Command aus
        cmd_msg = Bool()
        cmd_msg.data = False
        pub_cmd.publish(cmd_msg)
        for _ in range(5):
            rclpy.spin_once(test_node, timeout_sec=0.05)

        assert proc.poll() is None, "BuzzerNode ist unerwartet abgestuerzt!"

    finally:
        test_node.destroy_node()
        stop_node_process(proc)


if __name__ == "__main__":
    pytest.main(["-v", __file__])
