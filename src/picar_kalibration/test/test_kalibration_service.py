#!/usr/bin/env python3
# File name   : test_kalibration_service.py
# Description : Tests fuer das picar_kalibration Service-Paket
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import pytest
import rclpy
from rclpy.node import Node

INSTALL_PY_PATH = "/home/pirob/ros2_ws/install/picar_kalibration/lib/python3.14/site-packages"
if INSTALL_PY_PATH not in sys.path:
    sys.path.insert(0, INSTALL_PY_PATH)

from picar_kalibration.srv import (  # noqa: E402
    GetServoCalibration,
    SaveCalibration,
    SetServoCalibration,
)

SCRIPT_PATH = "/home/pirob/ros2_ws/src/picar_kalibration/scripts/kalibration_service_node.py"


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def start_service_process(config_file: str) -> subprocess.Popen:
    env = os.environ.copy()
    if "PYTHONPATH" in env:
        env["PYTHONPATH"] = f"{INSTALL_PY_PATH}:{env['PYTHONPATH']}"
    else:
        env["PYTHONPATH"] = INSTALL_PY_PATH

    proc = subprocess.Popen(
        [sys.executable, SCRIPT_PATH, "--ros-args", "-p", f"config_file:={config_file}"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        env=env,
        preexec_fn=os.setsid
    )
    time.sleep(1.2)
    assert proc.poll() is None, "kalibration_service_node ist abgestuerzt!"
    return proc


def stop_service_process(proc: subprocess.Popen):
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


def wait_service(client, node: Node, timeout=5.0) -> bool:
    t_end = time.time() + timeout
    while time.time() < t_end:
        if client.service_is_ready():
            return True
        rclpy.spin_once(node, timeout_sec=0.05)
    return False


def test_kalibration_services(ros_context):
    """Testet die Services /picar_kalibration/set_servo, get_servo und save_to_yaml."""
    tmp_dir = tempfile.mkdtemp()
    test_yaml = os.path.join(tmp_dir, "test_params.yaml")
    with open(test_yaml, "w", encoding="utf-8") as f:
        f.write(
            "motor_node:\n"
            "    ros__parameters:\n"
            "        servo_offsets: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]\n"
            "        servo_mins:    [60.0, 10.0, 30.0, 30.0, 50.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]\n"
            "        servo_maxs:    [120.0, 170.0, 150.0, 150.0, 130.0, 180.0, 180.0, 180.0, 180.0, 180.0, 180.0, 180.0, 180.0, 180.0, 180.0, 180.0]\n"
        )

    proc = start_service_process(test_yaml)
    client_node = Node("test_kalib_client")

    try:
        cli_set = client_node.create_client(SetServoCalibration, "/picar_kalibration/set_servo")
        cli_get = client_node.create_client(GetServoCalibration, "/picar_kalibration/get_servo")
        cli_save = client_node.create_client(SaveCalibration, "/picar_kalibration/save_to_yaml")

        assert wait_service(cli_set, client_node)
        assert wait_service(cli_get, client_node)
        assert wait_service(cli_save, client_node)

        # 1. Setze Kalibrierung fuer CH4 (AD002 Greifer)
        req_set = SetServoCalibration.Request()
        req_set.channel = 4
        req_set.offset = 3.5
        req_set.min_angle = 45.0
        req_set.max_angle = 135.0

        future_set = cli_set.call_async(req_set)
        rclpy.spin_until_future_complete(client_node, future_set, timeout_sec=2.0)
        res_set = future_set.result()
        assert res_set is not None
        assert res_set.success is True

        # 2. Frage Werte von CH4 ab
        req_get = GetServoCalibration.Request()
        req_get.channel = 4
        future_get = cli_get.call_async(req_get)
        rclpy.spin_until_future_complete(client_node, future_get, timeout_sec=2.0)
        res_get = future_get.result()
        assert res_get is not None
        assert res_get.success is True
        assert res_get.offset == pytest.approx(3.5, abs=0.01)
        assert res_get.min_angle == pytest.approx(45.0, abs=0.01)
        assert res_get.max_angle == pytest.approx(135.0, abs=0.01)

        # 3. In Datei speichern
        req_save = SaveCalibration.Request()
        req_save.file_path = test_yaml
        future_save = cli_save.call_async(req_save)
        rclpy.spin_until_future_complete(client_node, future_save, timeout_sec=2.0)
        res_save = future_save.result()
        assert res_save is not None
        assert res_save.success is True

        # 4. Inhalt der gespeicherten Datei pruefen
        with open(test_yaml, "r", encoding="utf-8") as f:
            saved_content = f.read()
        assert "3.5" in saved_content
        assert "45.0" in saved_content
        assert "135.0" in saved_content

    finally:
        client_node.destroy_node()
        stop_service_process(proc)
        shutil.rmtree(tmp_dir, ignore_errors=True)


if __name__ == "__main__":
    pytest.main(["-v", __file__])
