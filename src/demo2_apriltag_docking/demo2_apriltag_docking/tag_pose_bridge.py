import math

from apriltag_msgs.msg import AprilTagDetectionArray
from demo2_apriltag_docking.monitor import make_status, shutdown_if_running
from demo2_apriltag_docking.tag_policy import Detection, load_dock_specs, TagGate
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


def to_policy_detection(message, transform, stamp):
    rotation = transform.transform.rotation
    sin_yaw = 2.0 * (rotation.w * rotation.z + rotation.x * rotation.y)
    cos_yaw = 1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z)
    translation = transform.transform.translation
    return Detection(
        tag_id=int(message.id),
        hamming=int(message.hamming),
        decision_margin=float(message.decision_margin),
        stamp=stamp,
        x=float(translation.x),
        y=float(translation.y),
        yaw=math.atan2(sin_yaw, cos_yaw),
    )


def metadata_only(message, stamp):
    return Detection(
        tag_id=int(message.id),
        hamming=int(message.hamming),
        decision_margin=float(message.decision_margin),
        stamp=stamp,
        x=0.0,
        y=0.0,
        yaw=0.0,
    )


def lookup_tag_transform(buffer, target_frame, tag_frame):
    return buffer.lookup_transform(target_frame, tag_frame, Time())


def to_pose_message(transform):
    pose = PoseStamped()
    pose.header = transform.header
    pose.pose.position.x = transform.transform.translation.x
    pose.pose.position.y = transform.transform.translation.y
    pose.pose.position.z = transform.transform.translation.z
    pose.pose.orientation = transform.transform.rotation
    return pose


class TagPoseBridge(Node):

    def __init__(self):
        super().__init__('tag_pose_bridge')
        self._declare_parameters()

        mapping_file = self.get_parameter('dock_mapping_file').value
        if not mapping_file:
            raise ValueError('dock_mapping_file must be set')
        with open(mapping_file, encoding='utf-8') as stream:
            self.specs = load_dock_specs(yaml.safe_load(stream))

        publish_rate = float(self.get_parameter('publish_rate_hz').value)
        self.gate = TagGate(
            specs=self.specs,
            min_margin=float(self.get_parameter('min_decision_margin').value),
            max_hamming=int(self.get_parameter('max_hamming').value),
            confirmations=int(self.get_parameter('confirmations').value),
            confirmation_window=float(
                self.get_parameter('confirmation_window').value
            ),
            publish_period=1.0 / publish_rate,
            loss_timeout=float(self.get_parameter('loss_timeout').value),
            max_translation_jump=float(
                self.get_parameter('max_translation_jump').value
            ),
            max_yaw_jump=math.radians(
                float(self.get_parameter('max_yaw_jump_deg').value)
            ),
        )
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            self.get_parameter('output_pose_topic').value,
            10,
        )
        self.state_publisher = self.create_publisher(
            String,
            self.get_parameter('state_topic').value,
            state_qos,
        )
        self.diagnostic_publisher = self.create_publisher(
            DiagnosticArray,
            '/diagnostics',
            10,
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            self.get_parameter('detections_topic').value,
            self._on_detections,
            10,
        )
        self.loss_timer = self.create_timer(0.1, self._check_loss)
        self._last_state = None
        self._last_diagnostic_time = None

    def _declare_parameters(self):
        defaults = {
            'dock_mapping_file': '',
            'detections_topic': '/apriltag/detections',
            'output_pose_topic': '/detected_dock_pose',
            'state_topic': '/demo2/tag_state',
            'min_decision_margin': 50.0,
            'max_hamming': 0,
            'confirmations': 3,
            'confirmation_window': 0.5,
            'publish_rate_hz': 10.0,
            'loss_timeout': 0.5,
            'max_translation_jump': 0.25,
            'max_yaw_jump_deg': 20.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _on_detections(self, message):
        now = self._now_seconds()
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        detections = list(message.detections)

        if len(detections) != 1:
            result = self.gate.evaluate(
                [metadata_only(item, stamp) for item in detections],
                now,
            )
            self._report(result.reason, DiagnosticStatus.WARN)
            return

        tag_message = detections[0]
        dock = self.specs.get(int(tag_message.id))
        if dock is None:
            result = self.gate.evaluate([metadata_only(tag_message, stamp)], now)
            self._report(
                result.reason,
                DiagnosticStatus.WARN,
                {'tag_id': tag_message.id},
            )
            return

        try:
            transform = lookup_tag_transform(
                self.tf_buffer,
                message.header.frame_id,
                dock.tag_frame,
            )
        except TransformException as exc:
            self._report(
                'TF_UNAVAILABLE',
                DiagnosticStatus.WARN,
                {'tag_id': tag_message.id, 'error': exc},
            )
            return

        transform_stamp = (
            transform.header.stamp.sec
            + transform.header.stamp.nanosec * 1e-9
        )
        detection = to_policy_detection(tag_message, transform, transform_stamp)
        result = self.gate.evaluate([detection], now)
        if result.accepted:
            self.pose_publisher.publish(to_pose_message(transform))
            self._report(
                'ACCEPTED',
                DiagnosticStatus.OK,
                {
                    'tag_id': detection.tag_id,
                    'dock_id': result.dock.dock_id,
                    'margin': detection.decision_margin,
                },
                refresh_seconds=1.0,
            )
        elif result.reason != 'RATE_LIMITED':
            self._report(
                result.reason,
                DiagnosticStatus.WARN,
                {'tag_id': detection.tag_id},
            )

    def _check_loss(self):
        reason = self.gate.loss_reason(self._now_seconds())
        if reason:
            self._report(reason, DiagnosticStatus.WARN)

    def _report(self, state, level, values=None, refresh_seconds=None):
        now = self._now_seconds()
        repeated = state == self._last_state
        refresh_due = (
            refresh_seconds is not None
            and self._last_diagnostic_time is not None
            and now - self._last_diagnostic_time >= refresh_seconds
        )
        if repeated and not refresh_due:
            return

        if not repeated:
            self.state_publisher.publish(String(data=state))
            self._last_state = state

        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        diagnostics.status.append(
            make_status(self.get_name(), level, state, values or {})
        )
        self.diagnostic_publisher.publish(diagnostics)
        self._last_diagnostic_time = now

    def _now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TagPoseBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except (OSError, ValueError, yaml.YAMLError) as exc:
        if node is not None:
            node.get_logger().fatal(str(exc))
        else:
            print(f'tag_pose_bridge: {exc}')
    finally:
        if node is not None:
            node.destroy_node()
        shutdown_if_running()


if __name__ == '__main__':
    main()
