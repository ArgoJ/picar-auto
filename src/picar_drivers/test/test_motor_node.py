#!/usr/bin/env python3
# File name   : test_motor_node.py
# Description : Tests fuer die picar_drivers MotorNode (Antrieb & Lenkung ueber PCA9685)
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


def test_motor_node_cmd_vel_drive_and_steering(ros_context):
    """Testet Geschwindigkeits- und Lenkvorgaben ueber cmd_vel."""
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

        assert wait_for_subscribers(pub_cmd_vel), "motor_node hat cmd_vel nicht abonniert"
        assert wait_for_publishers(test_node, "picar/motor/status"), "motor_node publiziert nicht auf picar/motor/status"

        def send_and_wait(throttle_exp, steering_exp, twist_msg, timeout=2.0):
            received_status.clear()
            pub_cmd_vel.publish(twist_msg)
            t_end = time.time() + timeout
            while time.time() < t_end:
                rclpy.spin_once(test_node, timeout_sec=0.05)
                if received_status:
                    last = received_status[-1]
                    if (abs(last.linear.x - throttle_exp) < 0.05 and
                            abs(last.angular.z - steering_exp) < 1.5):
                        return True
            return False

        # 1. Vorwaerts (50%) geradeaus
        msg = Twist()
        msg.linear.x = 0.5
        msg.angular.z = 0.0
        assert send_and_wait(0.5, 90.0, msg), f"Vorwaerts fehlgeschlagen: {received_status}"

        # 2. Rueckwaerts (-40%) geradeaus
        msg.linear.x = -0.4
        msg.angular.z = 0.0
        assert send_and_wait(-0.4, 90.0, msg), f"Rueckwaerts fehlgeschlagen: {received_status}"

        # 3. Vollanschlag Links (angular.z = +1.0 rad/s -> 90 + 30 = 120 Grad)
        msg.linear.x = 0.2
        msg.angular.z = 1.0
        assert send_and_wait(0.2, 120.0, msg), f"Lenkung Links fehlgeschlagen: {received_status}"

        # 4. Vollanschlag Rechts (angular.z = -1.0 rad/s -> 90 - 30 = 60 Grad)
        msg.linear.x = 0.2
        msg.angular.z = -1.0
        assert send_and_wait(0.2, 60.0, msg), f"Lenkung Rechts fehlgeschlagen: {received_status}"

        # 5. Stopp & Neutrallenkung
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        assert send_and_wait(0.0, 90.0, msg), f"Stopp fehlgeschlagen: {received_status}"

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


def test_motor_node_servo_and_gripper_clamping(ros_context):
    """Testet die Ansteuerung beliebiger Servokanaele (CH1) und des AD002 Greifers (CH4)."""
    proc = start_node_process("motor_node")
    test_node = Node("test_servo_client")

    try:
        from std_msgs.msg import Float32, Float32MultiArray, Int32
        pub_servo = test_node.create_publisher(Float32MultiArray, "picar/servo/cmd", 10)
        pub_gripper = test_node.create_publisher(Float32, "picar/gripper/cmd", 10)
        pub_release = test_node.create_publisher(Int32, "picar/servo/release", 10)

        assert wait_for_subscribers(pub_servo)
        assert wait_for_subscribers(pub_gripper)
        assert wait_for_subscribers(pub_release)

        # 1. Greifer auf 90 Grad
        g_msg = Float32()
        g_msg.data = 90.0
        pub_gripper.publish(g_msg)
        rclpy.spin_once(test_node, timeout_sec=0.1)

        # 2. Servo CH1 auf 80 Grad
        s_msg = Float32MultiArray()
        s_msg.data = [1.0, 80.0]
        pub_servo.publish(s_msg)
        rclpy.spin_once(test_node, timeout_sec=0.1)

        # 3. Greifer stromlos schalten
        r_msg = Int32()
        r_msg.data = 4
        pub_release.publish(r_msg)
        rclpy.spin_once(test_node, timeout_sec=0.1)

        assert proc.poll() is None, "MotorNode abgestuerzt waehrend Servo-Kommandos!"

    finally:
        test_node.destroy_node()
        stop_node_process(proc)


if __name__ == "__main__":
    pytest.main(["-v", __file__])
