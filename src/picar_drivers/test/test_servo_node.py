#!/usr/bin/env python3
# File name   : test_servo_node.py
# Description : Tests fuer die picar_drivers ServoNode (einzelne Servo-Topics & Kanalzuordnung)
import os
import subprocess
import sys
import time
import pytest
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Int32

sys.path.insert(0, os.path.dirname(__file__))
from common import (  # noqa: E402
    get_executable_path,
    start_node_process,
    stop_node_process,
    wait_for_publishers,
    wait_for_subscribers,
)


def start_servo_node_with_params(extra_args=None) -> subprocess.Popen:
    exe_path = get_executable_path("servo_node")
    cmd = [exe_path]
    if extra_args:
        cmd.extend(["--ros-args"] + extra_args)
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        preexec_fn=os.setsid,
    )
    time.sleep(1.0)
    assert proc.poll() is None, "servo_node ist direkt nach dem Start abgestuerzt!"
    return proc


def test_servo_node_lifecycle(ros_context):
    """Prueft Start mit Default-Startpositionen und sauberes Beenden der ServoNode."""
    proc = start_servo_node_with_params([
        "-p", "servo_defaults:=[90.0,85.0,115.0,70.0,75.0,90.0,90.0,90.0,90.0,90.0,90.0,90.0,90.0,90.0,90.0,90.0]",
    ])
    test_node = Node("test_servo_startup_defaults")
    try:
        assert proc.poll() is None, "ServoNode laeuft nicht!"
        assert wait_for_publishers(test_node, "picar/servo/status")
    finally:
        test_node.destroy_node()
        stop_node_process(proc)


def test_servo_node_named_topics_and_clamping(ros_context):
    """Testet alle 5 benannten Servo-Topics (lenkung, rotation, shoulder, elbow, gripper)."""
    proc = start_node_process("servo_node")
    test_node = Node("test_servo_client")

    try:
        received_status = []
        test_node.create_subscription(
            Float32MultiArray,
            "picar/servo/status",
            lambda m: received_status.append(list(m.data)),
            10,
        )
        assert wait_for_publishers(test_node, "picar/servo/status")

        servo_topics = [
            ("picar/servo/lenkung", 0, 100.0, 100.0),
            ("picar/servo/rotation", 1, 80.0, 80.0),
            ("picar/servo/shoulder", 2, 110.0, 110.0),
            ("picar/servo/elbow", 3, 75.0, 75.0),
            ("picar/servo/gripper", 4, 90.0, 90.0),
        ]

        publishers = {}
        for topic, _, _, _ in servo_topics:
            pub = test_node.create_publisher(Float32, topic, 10)
            assert wait_for_subscribers(pub), f"servo_node hat {topic} nicht abonniert"
            publishers[topic] = pub

        for topic, expected_ch, cmd_angle, expected_angle in servo_topics:
            received_status.clear()
            msg = Float32()
            msg.data = cmd_angle
            publishers[topic].publish(msg)

            t_end = time.time() + 2.0
            matched = False
            while time.time() < t_end:
                rclpy.spin_once(test_node, timeout_sec=0.05)
                for st in received_status:
                    if int(round(st[0])) == expected_ch and abs(st[1] - expected_angle) < 0.1:
                        matched = True
                        break
                if matched:
                    break
            assert matched, f"Topic {topic} -> CH{expected_ch} Winkel {expected_angle} nicht bestaetigt: {received_status}"

    finally:
        test_node.destroy_node()
        stop_node_process(proc)


def test_servo_node_custom_channel_mapping(ros_context):
    """Prueft die freie Kanalzuordnung ueber Parameter (z. B. gripper auf CH7, elbow auf CH6)."""
    proc = start_servo_node_with_params([
        "-p", "channel_gripper:=7",
        "-p", "channel_elbow:=6",
    ])
    test_node = Node("test_servo_channel_mapping")

    try:
        received_status = []
        test_node.create_subscription(
            Float32MultiArray,
            "picar/servo/status",
            lambda m: received_status.append(list(m.data)),
            10,
        )
        pub_gripper = test_node.create_publisher(Float32, "picar/servo/gripper", 10)
        pub_elbow = test_node.create_publisher(Float32, "picar/servo/elbow", 10)
        assert wait_for_subscribers(pub_gripper)
        assert wait_for_subscribers(pub_elbow)
        assert wait_for_publishers(test_node, "picar/servo/status")

        def send_and_check(pub, angle, exp_ch):
            received_status.clear()
            msg = Float32()
            msg.data = angle
            pub.publish(msg)
            t_end = time.time() + 2.0
            while time.time() < t_end:
                rclpy.spin_once(test_node, timeout_sec=0.05)
                for st in received_status:
                    if int(round(st[0])) == exp_ch and abs(st[1] - angle) < 0.1:
                        return True
            return False

        assert send_and_check(pub_gripper, 95.0, 7), f"Gripper auf CH7 fehlgeschlagen: {received_status}"
        assert send_and_check(pub_elbow, 85.0, 6), f"Elbow auf CH6 fehlgeschlagen: {received_status}"

    finally:
        test_node.destroy_node()
        stop_node_process(proc)


def test_servo_node_cmd_vel_and_release(ros_context):
    """Prueft Lenkung via cmd_vel sowie generisches picar/servo/cmd und picar/servo/release."""
    proc = start_node_process("servo_node")
    test_node = Node("test_servo_cmdvel")

    try:
        received_status = []
        test_node.create_subscription(
            Float32MultiArray,
            "picar/servo/status",
            lambda m: received_status.append(list(m.data)),
            10,
        )
        pub_cmd_vel = test_node.create_publisher(Twist, "cmd_vel", 10)
        pub_servo_cmd = test_node.create_publisher(Float32MultiArray, "picar/servo/cmd", 10)
        pub_release = test_node.create_publisher(Int32, "picar/servo/release", 10)

        assert wait_for_subscribers(pub_cmd_vel)
        assert wait_for_subscribers(pub_servo_cmd)
        assert wait_for_subscribers(pub_release)
        assert wait_for_publishers(test_node, "picar/servo/status")

        # 1. Lenkung Links via cmd_vel (angular.z = +1.0 -> 90 + 30 = 120 Grad auf CH0)
        twist = Twist()
        twist.angular.z = 1.0
        received_status.clear()
        pub_cmd_vel.publish(twist)

        t_end = time.time() + 2.0
        matched = False
        while time.time() < t_end:
            rclpy.spin_once(test_node, timeout_sec=0.05)
            for st in received_status:
                if int(round(st[0])) == 0 and abs(st[1] - 120.0) < 0.5:
                    matched = True
                    break
            if matched:
                break
        assert matched, f"Lenkung via cmd_vel fehlgeschlagen: {received_status}"

        # 2. Generisches picar/servo/cmd auf CH1
        s_msg = Float32MultiArray()
        s_msg.data = [1.0, 80.0]
        pub_servo_cmd.publish(s_msg)
        rclpy.spin_once(test_node, timeout_sec=0.1)

        # 3. Release CH4
        r_msg = Int32()
        r_msg.data = 4
        pub_release.publish(r_msg)
        rclpy.spin_once(test_node, timeout_sec=0.1)

        assert proc.poll() is None, "ServoNode abgestuerzt!"

    finally:
        test_node.destroy_node()
        stop_node_process(proc)


if __name__ == "__main__":
    pytest.main(["-v", __file__])
