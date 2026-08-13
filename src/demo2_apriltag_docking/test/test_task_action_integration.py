from pathlib import Path
from threading import Event, Thread
import time
import unittest

from diagnostic_msgs.msg import DiagnosticArray
import launch
from launch_ros.actions import Node as LaunchNode
import launch_testing
from launch_testing.actions import ReadyToTest
from nav2_msgs.action import DockRobot
import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


@pytest.mark.launch_test
def generate_test_description():
    mapping_file = Path(__file__).parents[1] / 'config' / 'docks.yaml'
    task_bridge = LaunchNode(
        package='demo2_apriltag_docking',
        executable='docking_task_bridge',
        name='docking_task_bridge',
        parameters=[{
            'dock_mapping_file': str(mapping_file),
            'dock_action_name': '/test/dock_robot',
            'start_service': '/test/start_docking',
            'cancel_service': '/test/cancel_docking',
            'guard_topic': '/test/docking_allowed',
            'guard_required': True,
            'guard_timeout': 10.0,
            'state_topic': '/test/docking_state',
        }],
        output='screen',
    )
    return launch.LaunchDescription([task_bridge, ReadyToTest()]), {
        'task_bridge': task_bridge,
    }


class TestTaskActionIntegration(unittest.TestCase):
    """Exercise real service, Guard, and DockRobot action interactions."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('task_action_integration_test')
        cls.executor = MultiThreadedExecutor(num_threads=4)
        cls.executor.add_node(cls.node)
        cls.executor_thread = Thread(target=cls.executor.spin, daemon=True)

        cls.received_goals = []
        cls.states = []
        cls.controlling_seen = Event()
        cls.guard_denied_diagnostics = 0
        cls.first_goal_done = Event()
        cls.pending_goal_received = Event()
        cls.release_pending_goal = Event()
        cls.cancel_seen = Event()
        cls.second_goal_done = Event()
        cls.goal_count = 0

        callback_group = ReentrantCallbackGroup()
        cls.action_server = ActionServer(
            cls.node,
            DockRobot,
            '/test/dock_robot',
            execute_callback=cls._execute_goal,
            goal_callback=cls._on_goal,
            cancel_callback=cls._on_cancel,
            callback_group=callback_group,
        )
        cls.action_probe = ActionClient(cls.node, DockRobot, '/test/dock_robot')
        cls.start_client = cls.node.create_client(Trigger, '/test/start_docking')
        cls.cancel_client = cls.node.create_client(Trigger, '/test/cancel_docking')
        cls.guard_publisher = cls.node.create_publisher(
            Bool,
            '/test/docking_allowed',
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )
        cls.node.create_subscription(
            String,
            '/test/docking_state',
            cls._on_state,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        cls.node.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            cls._on_diagnostics,
            10,
        )
        cls.executor_thread.start()

    @classmethod
    def _on_state(cls, message):
        cls.states.append(message.data)
        if message.data == 'CONTROLLING':
            cls.controlling_seen.set()

    @classmethod
    def _on_diagnostics(cls, message):
        cls.guard_denied_diagnostics += sum(
            status.name == 'docking_task_bridge'
            and status.message == 'GUARD_DENIED'
            for status in message.status
        )

    @classmethod
    def _on_goal(cls, goal_request):
        cls.goal_count += 1
        cls.received_goals.append(goal_request)
        if cls.goal_count == 2:
            cls.pending_goal_received.set()
            if not cls.release_pending_goal.wait(timeout=10.0):
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @classmethod
    def _on_cancel(cls, _goal_handle):
        cls.cancel_seen.set()
        return CancelResponse.ACCEPT

    @classmethod
    def _execute_goal(cls, goal_handle):
        result = DockRobot.Result()
        if len(cls.received_goals) == 1:
            feedback = DockRobot.Feedback()
            feedback.state = DockRobot.Feedback.CONTROLLING
            feedback.num_retries = 1
            goal_handle.publish_feedback(feedback)
            cls.controlling_seen.wait(timeout=10.0)
            result.success = True
            result.num_retries = 1
            goal_handle.succeed()
            cls.first_goal_done.set()
            return result

        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                result.success = False
                result.error_code = DockRobot.Result.FAILED_TO_CONTROL
                result.error_msg = 'guard canceled pending goal'
                goal_handle.canceled()
                cls.second_goal_done.set()
                return result
            time.sleep(0.01)
        result.success = False
        goal_handle.abort()
        cls.second_goal_done.set()
        return result

    @classmethod
    def tearDownClass(cls):
        cls.action_server.destroy()
        cls.executor.shutdown(timeout_sec=5.0)
        cls.executor_thread.join(timeout=5.0)
        cls.node.destroy_node()
        rclpy.shutdown()

    def _call(self, client):
        self.assertTrue(client.wait_for_service(timeout_sec=10.0))
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not future.done():
            time.sleep(0.01)
        self.assertTrue(future.done())
        return future.result()

    def _publish_guard(self, allowed):
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            if self.guard_publisher.get_subscription_count() > 0:
                break
            time.sleep(0.01)
        self.assertGreater(self.guard_publisher.get_subscription_count(), 0)

        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            self.guard_publisher.publish(Bool(data=allowed))
            time.sleep(0.05)

    def _wait_for_state(self, state, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if state in self.states:
                return
            time.sleep(0.01)
        self.fail(f'task bridge did not publish {state}; states={self.states}')

    def _wait_for_guard_diagnostics(self, count, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.guard_denied_diagnostics >= count:
                return
            time.sleep(0.01)
        self.fail(
            'task bridge did not publish terminal GUARD_DENIED diagnostic; '
            f'count={self.guard_denied_diagnostics}'
        )

    def test_action_success_and_pending_guard_cancel(self):
        self.assertTrue(self.action_probe.wait_for_server(timeout_sec=10.0))
        time.sleep(0.5)
        self._publish_guard(True)
        response = self._call(self.start_client)
        self.assertTrue(response.success, response.message)
        self.assertEqual(response.message, 'DOCKING_REQUESTED')
        self.assertTrue(self.first_goal_done.wait(timeout=10.0))
        self._wait_for_state('CONTROLLING')
        self._wait_for_state('SUCCEEDED')

        first_goal = self.received_goals[0]
        self.assertTrue(first_goal.use_dock_id)
        self.assertEqual(first_goal.dock_id, 'demo_charge_dock')
        self.assertEqual(first_goal.max_staging_time, 60.0)
        self.assertTrue(first_goal.navigate_to_staging_pose)

        self._publish_guard(True)
        response = self._call(self.start_client)
        self.assertTrue(response.success)
        self.assertTrue(self.pending_goal_received.wait(timeout=10.0))

        self._publish_guard(False)
        self._wait_for_state('GUARD_DENIED')
        self._publish_guard(True)
        self.release_pending_goal.set()

        self.assertTrue(self.cancel_seen.wait(timeout=10.0))
        self.assertTrue(self.second_goal_done.wait(timeout=10.0))
        self._wait_for_guard_diagnostics(2)
        response = self._call(self.cancel_client)
        self.assertFalse(response.success)
        self.assertEqual(response.message, 'NO_ACTIVE_DOCKING')


@launch_testing.post_shutdown_test()
class TestTaskActionShutdown(unittest.TestCase):
    """Verify that the Python task bridge exits cleanly."""

    def test_process_exits_cleanly(self, proc_info, task_bridge):
        launch_testing.asserts.assertExitCodes(proc_info, process=task_bridge)
