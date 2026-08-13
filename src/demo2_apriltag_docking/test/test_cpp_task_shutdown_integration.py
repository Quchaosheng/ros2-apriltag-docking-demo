from pathlib import Path
import signal
from threading import Event, Thread
import time
import unittest

import launch
from launch.events.process import SignalProcess
from launch_ros.actions import Node as LaunchNode
import launch_testing
from launch_testing.actions import ReadyToTest
import launch_testing.markers
from nav2_msgs.action import DockRobot
import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    mapping_file = Path(__file__).parents[1] / 'config' / 'docks.yaml'
    task_bridge = LaunchNode(
        package='demo2_apriltag_docking_cpp',
        executable='docking_task_bridge_cpp',
        name='docking_task_bridge',
        parameters=[{
            'dock_mapping_file': str(mapping_file),
            'dock_action_name': '/test_shutdown/dock_robot',
            'start_service': '/test_shutdown/start_docking',
            'cancel_service': '/test_shutdown/cancel_docking',
            'guard_required': False,
            'state_topic': '/test_shutdown/docking_state',
            'shutdown_cancel_timeout': 2.0,
        }],
        output='screen',
    )
    return launch.LaunchDescription([task_bridge, ReadyToTest()]), {
        'task_bridge': task_bridge,
    }


class TestCppTaskShutdownIntegration(unittest.TestCase):
    """Verify that SIGINT cancels an active goal before bridge exit."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('cpp_task_shutdown_integration_test')
        cls.executor = MultiThreadedExecutor(num_threads=4)
        cls.executor.add_node(cls.node)
        cls.executor_thread = Thread(target=cls.executor.spin, daemon=True)

        cls.goal_executing = Event()
        cls.cancel_seen = Event()
        cls.release_result = Event()
        cls.result_done = Event()

        cls.action_server = ActionServer(
            cls.node,
            DockRobot,
            '/test_shutdown/dock_robot',
            execute_callback=cls._execute_goal,
            cancel_callback=cls._on_cancel,
            callback_group=ReentrantCallbackGroup(),
        )
        cls.action_probe = ActionClient(
            cls.node,
            DockRobot,
            '/test_shutdown/dock_robot',
        )
        cls.start_client = cls.node.create_client(
            Trigger,
            '/test_shutdown/start_docking',
        )
        cls.executor_thread.start()

    @classmethod
    def _on_cancel(cls, _goal_handle):
        cls.cancel_seen.set()
        return CancelResponse.ACCEPT

    @classmethod
    def _execute_goal(cls, goal_handle):
        cls.goal_executing.set()
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not goal_handle.is_cancel_requested:
            time.sleep(0.01)

        result = DockRobot.Result()
        result.success = False
        if goal_handle.is_cancel_requested:
            cls.release_result.wait(timeout=10.0)
            goal_handle.canceled()
        else:
            goal_handle.abort()
        cls.result_done.set()
        return result

    @classmethod
    def tearDownClass(cls):
        cls.release_result.set()
        cls.result_done.wait(timeout=5.0)
        cls.action_server.destroy()
        cls.executor.shutdown(timeout_sec=5.0)
        cls.executor_thread.join(timeout=5.0)
        cls.node.destroy_node()
        rclpy.shutdown()

    def _call_start(self):
        self.assertTrue(self.start_client.wait_for_service(timeout_sec=10.0))
        future = self.start_client.call_async(Trigger.Request())
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not future.done():
            time.sleep(0.01)
        self.assertTrue(future.done())
        return future.result()

    def test_sigint_waits_for_cancel_ack(
        self,
        launch_service,
        proc_info,
        task_bridge,
    ):
        self.assertTrue(self.action_probe.wait_for_server(timeout_sec=10.0))
        time.sleep(0.5)
        response = self._call_start()
        self.assertTrue(response.success, response.message)
        self.assertTrue(self.goal_executing.wait(timeout=10.0))

        launch_service.emit_event(SignalProcess(
            signal_number=signal.SIGINT,
            process_matcher=lambda action: action is task_bridge,
        ))

        self.assertTrue(self.cancel_seen.wait(timeout=5.0))
        proc_info.assertWaitForShutdown(process=task_bridge, timeout=5.0)
        self.assertFalse(self.result_done.is_set())
        launch_testing.asserts.assertExitCodes(proc_info, process=task_bridge)


@launch_testing.post_shutdown_test()
class TestCppTaskShutdownExit(unittest.TestCase):
    """Verify the signaled bridge exits successfully."""

    def test_process_exits_cleanly(self, proc_info, task_bridge):
        launch_testing.asserts.assertExitCodes(proc_info, process=task_bridge)
