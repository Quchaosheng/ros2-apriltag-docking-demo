import math

from apriltag_msgs.msg import AprilTagDetection
from demo2_apriltag_docking import monitor
from demo2_apriltag_docking import tag_pose_bridge
from demo2_apriltag_docking.monitor import make_status
from demo2_apriltag_docking.tag_pose_bridge import to_policy_detection
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import TransformStamped


def test_make_status_serializes_monitor_values():
    status = make_status(
        'tag_pose_bridge',
        DiagnosticStatus.WARN,
        'LOW_MARGIN',
        {'tag_id': 0, 'margin': 42.5},
    )

    assert status.name == 'tag_pose_bridge'
    assert status.hardware_id == 'demo2_apriltag_docking'
    assert status.level == DiagnosticStatus.WARN
    assert status.message == 'LOW_MARGIN'
    assert [(item.key, item.value) for item in status.values] == [
        ('tag_id', '0'),
        ('margin', '42.5'),
    ]


def test_to_policy_detection_uses_tf_translation_and_wrapped_yaw():
    message = AprilTagDetection()
    message.id = 7
    message.hamming = 0
    message.decision_margin = 71.5

    transform = TransformStamped()
    transform.transform.translation.x = 1.2
    transform.transform.translation.y = -0.3
    transform.transform.rotation.z = math.sin(math.radians(45.0))
    transform.transform.rotation.w = math.cos(math.radians(45.0))

    detection = to_policy_detection(message, transform, stamp=12.25)

    assert detection.tag_id == 7
    assert detection.hamming == 0
    assert detection.decision_margin == 71.5
    assert detection.stamp == 12.25
    assert detection.x == 1.2
    assert detection.y == -0.3
    assert math.isclose(detection.yaw, math.pi / 2.0)


def test_lookup_tag_transform_uses_cached_latest_tf():
    class Buffer:

        def lookup_transform(self, *args, **kwargs):
            self.args = args
            self.kwargs = kwargs
            return 'transform'

    buffer = Buffer()

    transform = tag_pose_bridge.lookup_tag_transform(
        buffer,
        'camera_rgb_frame',
        'tag36h11:0',
    )

    assert transform == 'transform'
    assert buffer.args[:2] == ('camera_rgb_frame', 'tag36h11:0')
    assert buffer.args[2].nanoseconds == 0
    assert buffer.kwargs == {}


def test_pose_message_uses_transform_header_and_pose():
    transform = TransformStamped()
    transform.header.frame_id = 'camera_rgb_frame'
    transform.header.stamp.sec = 7
    transform.transform.translation.x = 1.2
    transform.transform.rotation.w = 1.0

    pose = tag_pose_bridge.to_pose_message(transform)

    assert pose.header == transform.header
    assert pose.pose.position.x == 1.2
    assert pose.pose.orientation.w == 1.0


def test_shutdown_only_runs_for_active_context(monkeypatch):
    calls = []
    monkeypatch.setattr(monitor.rclpy, 'ok', lambda: False)
    monkeypatch.setattr(monitor.rclpy, 'shutdown', lambda: calls.append('shutdown'))

    monitor.shutdown_if_running()

    assert calls == []

    monkeypatch.setattr(monitor.rclpy, 'ok', lambda: True)
    monitor.shutdown_if_running()
    assert calls == ['shutdown']
