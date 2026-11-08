#!/usr/bin/env python3
# File name   : test_led_node.py
# Description : Tests fuer die picar_drivers LedNode
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
    wait_for_publishers,
    wait_for_subscribers,
)


def send_and_wait_status(test_node: Node, publisher, msg, received_status: list, expected_val: int, timeout=2.0) -> bool:
    """Sendet eine Nachricht und wartet auf den erwarteten Rueckgabewert auf /picar/led/status."""
    received_status.clear()
    publisher.publish(msg)
    t_end = time.time() + timeout
    while time.time() < t_end:
        rclpy.spin_once(test_node, timeout_sec=0.05)
        if received_status and received_status[-1] == expected_val:
            return True
    return False


def test_led_node_lifecycle(ros_context):
    """Prueft Start und sauberes Beenden der LedNode."""
    proc = start_node_process("led_node")
    try:
        assert proc.poll() is None, "LedNode laeuft nicht!"
    finally:
        stop_node_process(proc)


def test_led_node_all_cmd(ros_context):
    """Testet das Schalten aller LEDs gleichzeitig ueber picar/led/all."""
    proc = start_node_process("led_node")
    test_node = Node("test_led_all_client")

    try:
        received_status = []
        test_node.create_subscription(Int32, "picar/led/status", lambda m: received_status.append(m.data), 10)
        pub_all = test_node.create_publisher(Bool, "picar/led/all", 10)

        assert wait_for_subscribers(pub_all), "led_node hat pub_all nicht abonniert"
        assert wait_for_publishers(test_node, "picar/led/status"), "led_node publiziert nicht auf status"

        # Alle an -> Status 7
        all_on = Bool()
        all_on.data = True
        assert send_and_wait_status(test_node, pub_all, all_on, received_status, 7)

        # Alle aus -> Status 0
        all_off = Bool()
        all_off.data = False
        assert send_and_wait_status(test_node, pub_all, all_off, received_status, 0)

    finally:
        test_node.destroy_node()
        stop_node_process(proc)


def test_led_node_bitmask_and_individual(ros_context):
    """Testet Bitmasken und Einzelschaltung der Front-LEDs."""
    proc = start_node_process("led_node")
    test_node = Node("test_led_bitmask_client")

    try:
        received_status = []
        test_node.create_subscription(Int32, "picar/led/status", lambda m: received_status.append(m.data), 10)
        pub_cmd = test_node.create_publisher(Int32, "picar/led/cmd", 10)
        pub_led1 = test_node.create_publisher(Bool, "picar/led/led1", 10)
        pub_led2 = test_node.create_publisher(Bool, "picar/led/led2", 10)
        pub_led3 = test_node.create_publisher(Bool, "picar/led/led3", 10)

        assert wait_for_subscribers(pub_cmd), "led_node hat pub_cmd nicht abonniert"
        assert wait_for_subscribers(pub_led1), "led_node hat pub_led1 nicht abonniert"
        assert wait_for_subscribers(pub_led2), "led_node hat pub_led2 nicht abonniert"
        assert wait_for_subscribers(pub_led3), "led_node hat pub_led3 nicht abonniert"
        assert wait_for_publishers(test_node, "picar/led/status"), "led_node publiziert nicht auf status"

        # 1. Bitmaske 5 setzen (LED1 + LED3)
        cmd_msg = Int32()
        cmd_msg.data = 5
        assert send_and_wait_status(test_node, pub_cmd, cmd_msg, received_status, 5)

        # 2. LED2 per Einzel-Topic zuschalten -> 5 | 2 = 7
        l2_msg = Bool()
        l2_msg.data = True
        assert send_and_wait_status(test_node, pub_led2, l2_msg, received_status, 7)

        # 3. LED1 per Einzel-Topic abschalten -> 7 & ~1 = 6
        l1_msg = Bool()
        l1_msg.data = False
        assert send_and_wait_status(test_node, pub_led1, l1_msg, received_status, 6)

        # 4. LED3 per Einzel-Topic abschalten -> 6 & ~4 = 2
        l3_msg = Bool()
        l3_msg.data = False
        assert send_and_wait_status(test_node, pub_led3, l3_msg, received_status, 2)

        # 5. Alles zuruecksetzen -> 0
        cmd_msg.data = 0
        assert send_and_wait_status(test_node, pub_cmd, cmd_msg, received_status, 0)

    finally:
        test_node.destroy_node()
        stop_node_process(proc)


if __name__ == "__main__":
    pytest.main(["-v", __file__])
