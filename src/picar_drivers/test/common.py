#!/usr/bin/env python3
# File name   : common.py
# Description : Gemeinsame Hilfsfunktionen und Fixtures fuer picar_drivers Tests
import os
import shutil
import signal
import subprocess
import time
import pytest
import rclpy
from rclpy.node import Node

DEFAULT_BIN_DIR = "/home/pirob/ros2_ws/install/picar_drivers/lib/picar_drivers"


@pytest.fixture(scope="module")
def ros_context():
    """Initialisiert und bereinigt den globalen ROS 2 Kontext."""
    rclpy.init()
    yield
    rclpy.shutdown()


def get_executable_path(name: str) -> str:
    """Ermittelt den Pfad zur ROS 2 Node Executable."""
    path = os.path.join(DEFAULT_BIN_DIR, name)
    if os.path.exists(path):
        return path
    which_path = shutil.which(name)
    if which_path:
        return which_path
    raise FileNotFoundError(f"Executable {name} nicht gefunden in {DEFAULT_BIN_DIR} oder PATH!")


def start_node_process(executable_name: str) -> subprocess.Popen:
    """Startet eine Node als isolierten Subprozess."""
    exe_path = get_executable_path(executable_name)
    proc = subprocess.Popen(
        [exe_path],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        preexec_fn=os.setsid
    )
    time.sleep(1.0)
    assert proc.poll() is None, f"{executable_name} ist direkt nach dem Start abgestuerzt!"
    return proc


def stop_node_process(proc: subprocess.Popen):
    """Beendet den Subprozess sicher und wartet auf dessen Beendigung."""
    if proc and proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=2.0)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                proc.wait(timeout=1.0)
            except Exception:
                pass


def wait_for_subscribers(pub, count=1, timeout=3.0) -> bool:
    """Wartet auf DDS-Discovery: Publisher -> Subscriber."""
    t_end = time.time() + timeout
    while time.time() < t_end:
        if pub.get_subscription_count() >= count:
            return True
        time.sleep(0.05)
    return False


def wait_for_publishers(node: Node, topic: str, count=1, timeout=3.0) -> bool:
    """Wartet auf DDS-Discovery: Subscriber -> Publisher."""
    t_end = time.time() + timeout
    while time.time() < t_end:
        if node.count_publishers(topic) >= count:
            return True
        time.sleep(0.05)
    return False
