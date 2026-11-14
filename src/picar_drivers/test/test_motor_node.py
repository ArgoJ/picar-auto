#!/usr/bin/env python3
# File name   : test_motor_node.py
# Description : Tests fuer die picar_drivers MotorNode (DC-Antriebsmotoren ueber PCA9685)
import os
import sys
import time
import pytest
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool

sys.path.insert(0, os.path.dirname(__file__))
from common import (  # noqa: E402
    start_node_process,
    stop_node_process,
    wait_for_publishers,
    wait_for_subscribers,
)


def test_motor_node_lifecycle(ros_context):
    """Prueft Start und sauberes Beenden der MotorNode."""
    proc = start_node_process("motor_node")
    try:
        assert proc.poll() is None, "MotorNode laeuft nicht!"
    finally:
        stop_node_process(proc)


def test_motor_node_cmd_vel_drive(ros_context):
    """Testet Geschwindigkeitsvorgaben ueber cmd_vel und picar/motor/cmd_vel."""
    proc = start_node_process("motor_node")
    test_node = Node("test_motor_client")

    try:
        received_status = []
        test_node.create_subscription(
            Twist,
            "picar/motor/status",
            lambda m: received_status.append(m),
            10
        )
        pub_cmd_vel = test_node.create_publisher(Twist, "cmd_vel", 10)
        pub_picar_cmd_vel = test_node.create_publisher(Twist, "picar/motor/cmd_vel", 10)

        assert wait_for_subscribers(pub_cmd_vel), "motor_node hat cmd_vel nicht abonniert"
        assert wait_for_subscribers(pub_picar_cmd_vel), "motor_node hat picar/motor/cmd_vel nicht abonniert"
        assert wait_for_publishers(test_node, "picar/motor/status"), "motor_node publiziert nicht auf picar/motor/status"

        def send_and_wait(pub, throttle_exp, twist_msg, timeout=2.0):
            received_status.clear()
            pub.publish(twist_msg)
            t_end = time.time() + timeout
            while time.time() < t_end:
                rclpy.spin_once(test_node, timeout_sec=0.05)
                if received_status:
                    last = received_status[-1]
                    if abs(last.linear.x - throttle_exp) < 0.05:
                        return True
            return False

        # 1. Vorwaerts (50%) ueber cmd_vel
        msg = Twist()
        msg.linear.x = 0.5
        assert send_and_wait(pub_cmd_vel, 0.5, msg), f"Vorwaerts fehlgeschlagen: {received_status}"

        # 2. Rueckwaerts (-40%) ueber picar/motor/cmd_vel
        msg.linear.x = -0.4
        assert send_and_wait(pub_picar_cmd_vel, -0.4, msg), f"Rueckwaerts fehlgeschlagen: {received_status}"

        # 3. Stopp
        msg.linear.x = 0.0
        assert send_and_wait(pub_cmd_vel, 0.0, msg), f"Stopp fehlgeschlagen: {received_status}"

    finally:
        test_node.destroy_node()
        stop_node_process(proc)


def test_motor_node_emergency_stop_and_watchdog(ros_context):
    """Testet den Notstopp ueber picar/motor/stop sowie den automatischen Watchdog-Timeout."""
    proc = start_node_process("motor_node")
    test_node = Node("test_motor_safety_client")

    try:
        received_status = []
        test_node.create_subscription(
            Twist,
            "picar/motor/status",
            lambda m: received_status.append(m),
            10
        )
        pub_cmd_vel = test_node.create_publisher(Twist, "cmd_vel", 10)
        pub_stop = test_node.create_publisher(Bool, "picar/motor/stop", 10)

        assert wait_for_subscribers(pub_cmd_vel)
        assert wait_for_subscribers(pub_stop)
        assert wait_for_publishers(test_node, "picar/motor/status")

        # 1. Fahrt starten
        msg = Twist()
        msg.linear.x = 0.6
        pub_cmd_vel.publish(msg)

        t_end = time.time() + 2.0
        while time.time() < t_end and not received_status:
            rclpy.spin_once(test_node, timeout_sec=0.05)

        assert len(received_status) > 0
        assert received_status[-1].linear.x == pytest.approx(0.6, abs=0.05)

        # 2. Notstopp senden
        stop_msg = Bool()
        stop_msg.data = True
        received_status.clear()
        pub_stop.publish(stop_msg)

        t_end = time.time() + 2.0
        stopped = False
        while time.time() < t_end:
            rclpy.spin_once(test_node, timeout_sec=0.05)
            if received_status and received_status[-1].linear.x == pytest.approx(0.0, abs=0.01):
                stopped = True
                break

        assert stopped, "Notstopp hat Motoren nicht auf 0.0 gesetzt!"

        # 3. Notstopp aufheben
        stop_msg.data = False
        pub_stop.publish(stop_msg)
        for _ in range(5):
            rclpy.spin_once(test_node, timeout_sec=0.05)

        # 4. Watchdog-Test: Fahrt senden, dann Nachricht unterbrechen (Timeout ist auf 0.5s konfiguriert)
        msg.linear.x = 0.5
        received_status.clear()
        pub_cmd_vel.publish(msg)

        t_end = time.time() + 2.0
        while time.time() < t_end and not received_status:
            rclpy.spin_once(test_node, timeout_sec=0.05)
        assert received_status[-1].linear.x == pytest.approx(0.5, abs=0.05)

        # 0.7s warten und spin, damit der 20Hz-Watchdog zuschlaegt
        received_status.clear()
        t_end = time.time() + 0.9
        watchdog_triggered = False
        while time.time() < t_end:
            rclpy.spin_once(test_node, timeout_sec=0.05)
            if received_status and abs(received_status[-1].linear.x) < 0.01:
                watchdog_triggered = True
                break

        assert watchdog_triggered, "Watchdog hat Motoren nach Timeout nicht gestoppt!"

    finally:
        test_node.destroy_node()
        stop_node_process(proc)


if __name__ == "__main__":
    pytest.main(["-v", __file__])
