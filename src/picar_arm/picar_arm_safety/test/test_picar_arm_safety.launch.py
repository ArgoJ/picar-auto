import re
import os
import rclpy
import unittest
import launch
import time
import launch_testing
import pytest
import subprocess
import numpy as np

from typing import Union
from unittest import result
from launch_ros.actions import Node
from rclpy.parameter import Parameter, parameter_value_to_python
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.publisher import Publisher

from picar_arm_safety_interface.msg import State, Control, References, Parameters


class AsyncParametersClient:
    """Simple async parameter client."""
    def __init__(self, node, remote_node_name):
        self.node = node
        self.remote_node_name = remote_node_name
        self._get_client = self.node.create_client(
            GetParameters,
            f'/{remote_node_name}/get_parameters')
        self._set_client = self.node.create_client(
            SetParameters,
            f'/{remote_node_name}/set_parameters')

    def wait_for_service(self, timeout_sec=1.0):
        ok1 = self._get_client.wait_for_service(timeout_sec=timeout_sec)
        ok2 = self._set_client.wait_for_service(timeout_sec=timeout_sec)
        return ok1 and ok2

    def get_parameters(self, names):
        request = GetParameters.Request()
        request.names = names
        return self._get_client.call_async(request)
    
    def set_parameters(self, params: list[tuple[str, object]]):
        """Set parameters synchron via service (returns Future)."""
        request = SetParameters.Request()
        for name, value in params:
            if isinstance(value, (list, tuple)):
                value = [float(v) for v in value]
            if isinstance(value, (int, float)):
                value = float(value)
            p = Parameter(name, value=value)
            request.parameters.append(p.to_parameter_msg())
        return self._set_client.call_async(request)


@pytest.mark.launch_test
def generate_test_description():
    """Generate launch description for node testing."""
    start_picar_arm_safety_node = Node(
        package='picar_arm_safety',
        executable='picar_arm_safety_node',
        name='picar_arm_safety_node'
    )

    return launch.LaunchDescription([
        start_picar_arm_safety_node,
        launch.actions.TimerAction(
                    period=5.0, actions=[launch_testing.actions.ReadyToTest()]),
    ])


class GeneratedNodeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('generated_node_test')
        self.param_client = AsyncParametersClient(self.node, 'picar_arm_safety_node')

    def tearDown(self):
        self.node.destroy_node()

    def test_set_constraints(self, proc_info):
        """Test if constraints compile-time declared default parameters."""
        param_name = "constraints.lbu"
        expected_value = [-3, -3, -3]
        self._check_parameter_set(param_name, expected_value)
        param_name = "constraints.lbx"
        expected_value = [-1.4, -0.8, -1.6, -3, -3, -3]
        self._check_parameter_set(param_name, expected_value)
        param_name = "constraints.lh"
        expected_value = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        self._check_parameter_set(param_name, expected_value)
        param_name = "constraints.lh_e"
        expected_value = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        self._check_parameter_set(param_name, expected_value)
        param_name = "constraints.lsh"
        expected_value = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        self._check_parameter_set(param_name, expected_value)
        param_name = "constraints.lsh_e"
        expected_value = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        self._check_parameter_set(param_name, expected_value)
        param_name = "constraints.ubu"
        expected_value = [3, 3, 3]
        self._check_parameter_set(param_name, expected_value)
        param_name = "constraints.ubx"
        expected_value = [1.4, 1.6, 1.6, 3, 3, 3]
        self._check_parameter_set(param_name, expected_value)
        param_name = "constraints.uh"
        expected_value = [1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000]
        self._check_parameter_set(param_name, expected_value)
        param_name = "constraints.uh_e"
        expected_value = [1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000]
        self._check_parameter_set(param_name, expected_value)
        param_name = "constraints.ush"
        expected_value = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        self._check_parameter_set(param_name, expected_value)
        param_name = "constraints.ush_e"
        expected_value = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        self._check_parameter_set(param_name, expected_value)

    def test_set_cost(self, proc_info):
        """Test if cost compile-time declared default parameters."""
        # --- Weights ---
        param_name = "cost.W"
        expected_value = [5, 5, 5, 0.1, 0.1, 0.1, 0.2, 0.2, 0.2]
        self._check_parameter_set(param_name, expected_value)
        param_name = "cost.W_0"
        expected_value = [5, 5, 5, 0.1, 0.1, 0.1, 0.2, 0.2, 0.2]
        self._check_parameter_set(param_name, expected_value)
        param_name = "cost.W_e"
        expected_value = [5, 5, 5]
        self._check_parameter_set(param_name, expected_value)

    def test_set_solver_options(self, proc_info):
        """Test if solver options compile-time declared default parameters."""
        param_name = "ts"
        expected_value = 0.05
        self._check_parameter_set(param_name, expected_value)

    def test_subscribing(self, proc_info):
        """Test if the node subscribes to all expected topics."""
        self._wait_for_subscription('/arm_state')
        self._wait_for_subscription('/arm_reference')
        self._wait_for_subscription('/arm_params')

    def test_publishing(self, proc_info):
        """Test if the node publishes to all expected topics."""
        self._wait_for_publisher('/arm_control')
    
    

    def _wait_for_subscription(self, topic: str, timeout: float = 2.0):
        end_time = time.time() + timeout
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            subs = self.node.get_subscriptions_info_by_topic(topic)
            if subs:
                return True
        self.fail(f"Node has NOT subscribed to '{topic}'.")

    def _wait_for_publisher(self, topic: str, timeout: float = 2.0):
        end_time = time.time() + timeout
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            pubs = self.node.get_publishers_info_by_topic(topic)
            if pubs:
                return True
        self.fail(f"Node has NOT published to '{topic}'.")

    def _wait_for_condition(self, condition_check: callable, timeout: float = 10.0) -> bool:
        """Spin the node until a condition is met or a timeout occurs."""
        end_time = time.time() + timeout
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if condition_check():
                return True
        return False

    def _check_parameter_get(self, param_name: str, expected_value: Union[list[float], float]):
        """Run a subprocess command and return its output."""
        if not self.param_client.wait_for_service(timeout_sec=2.0):
            self.fail(f"Parameter service for 'picar_arm_safety_node' not available.")

        future = self.param_client.get_parameters([param_name])
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
        if not future.done():
            self.fail(f"Timeout while getting parameter {param_name}")
        resp = future.result()
        if not resp.values:
            self.fail(f"Parameter {param_name} does not exist.")
        actual_value = parameter_value_to_python(resp.values[0])

        if isinstance(expected_value, list):
            self.assertListEqual(
                list(actual_value), expected_value, 
                f"Parameter {param_name} has the wrong value! Got {actual_value}")
        else:
            self.assertEqual(
                actual_value, expected_value, 
                f"Parameter {param_name} has the wrong value! Got {actual_value}")

    def _check_parameter_set(self, param_name: str, new_value: Union[list[float], float]):
        """Run a subprocess command and return its output."""
        if not self.param_client.wait_for_service(timeout_sec=2.0):
            self.fail(f"Parameter service for 'picar_arm_safety_node' not available.")

        future = self.param_client.set_parameters([(param_name, new_value)])
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=3.0)
        if not future.done() or future.result() is None:
            self.fail(f"Timeout while setting parameter {param_name}")

        results = future.result().results
        if not results or not results[0].successful:
            self.fail(f"Failed to set parameter {param_name}: {results[0].reason if results else 'unknown'}")

        self._check_parameter_get(param_name, new_value)