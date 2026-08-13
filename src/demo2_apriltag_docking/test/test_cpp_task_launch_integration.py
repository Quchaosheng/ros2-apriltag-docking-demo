from threading import Event
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
from launch_testing.actions import ReadyToTest
import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


@pytest.mark.launch_test
def generate_test_description():
    package_share = get_package_share_directory('demo2_apriltag_docking')
    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{package_share}/launch/demo.launch.py'
        ),
        launch_arguments={
            'headless': 'true',
            'rviz': 'false',
            'guard_required': 'false',
            'tag_bridge_implementation': 'python',
            'task_bridge_implementation': 'cpp',
        }.items(),
    )
    return launch.LaunchDescription([demo_launch, ReadyToTest()]), {}


class TestCppTaskBridgeSimulation(unittest.TestCase):
    """Exercise the Python tag bridge with the C++ task bridge."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('cpp_task_bridge_launch_test')
        cls.pose_seen = Event()
        cls.accepted_seen = Event()
        cls.docking_idle_seen = Event()
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        cls.node.create_subscription(
            PoseStamped,
            '/detected_dock_pose',
            lambda _message: cls.pose_seen.set(),
            10,
        )
        cls.node.create_subscription(
            String,
            '/demo2/docking_state',
            lambda message: cls.docking_idle_seen.set()
            if message.data == 'IDLE'
            else None,
            state_qos,
        )
        cls.node.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            cls._on_diagnostics,
            10,
        )

    @classmethod
    def _on_diagnostics(cls, message):
        if any(
            status.name == 'tag_pose_bridge'
            and status.message == 'ACCEPTED'
            for status in message.status
        ):
            cls.accepted_seen.set()

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_cpp_task_bridge_joins_full_graph(self):
        deadline = time.monotonic() + 180.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.2)
            if (
                self.pose_seen.is_set()
                and self.accepted_seen.is_set()
                and self.docking_idle_seen.is_set()
            ):
                return

        self.fail(
            'C++ task bridge did not join a graph with admitted dock poses '
            'within 180 seconds'
        )


@launch_testing.post_shutdown_test()
class TestCppTaskBridgeShutdown(unittest.TestCase):
    """Verify that the selected project-owned bridges exit cleanly."""

    def test_processes_exit_cleanly(self, proc_info):
        for process in ('tag_pose_bridge', 'docking_task_bridge_cpp'):
            launch_testing.asserts.assertExitCodes(
                proc_info,
                process=process,
            )
