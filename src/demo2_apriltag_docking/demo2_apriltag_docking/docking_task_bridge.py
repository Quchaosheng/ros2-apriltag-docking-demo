from action_msgs.msg import GoalStatus
from demo2_apriltag_docking.monitor import make_status, shutdown_if_running
from demo2_apriltag_docking.tag_policy import load_dock_specs
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
import yaml

try:
    from nav2_msgs.action import DockRobot
except ImportError:  # Humble compatibility; Jazzy carries this action in nav2_msgs.
    from opennav_docking_msgs.action import DockRobot

FEEDBACK_STATES = {
    1: 'NAV_TO_STAGING',
    2: 'INITIAL_PERCEPTION',
    3: 'CONTROLLING',
    4: 'WAIT_FOR_CHARGE',
    5: 'RETRY',
}


def feedback_state_name(value):
    return FEEDBACK_STATES.get(int(value), 'UNKNOWN')


def guard_qos_profile():
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


class TaskPolicy:
    def __init__(self, *, guard_required, guard_timeout):
        self.guard_required = guard_required
        self.guard_timeout = guard_timeout
        self.guard_allowed = None
        self.guard_stamp = None
        self.action_active = False

    def update_guard(self, allowed, stamp):
        self.guard_allowed = bool(allowed)
        self.guard_stamp = stamp

    def can_start(self, now):
        if self.action_active:
            return False, 'ALREADY_ACTIVE'
        reason = self._guard_reason(now)
        if reason:
            return False, reason
        return True, 'READY'

    def cancel_reason(self, now):
        if not self.action_active:
            return None
        return self._guard_reason(now)

    def _guard_reason(self, now):
        if not self.guard_required:
            return None
        if self.guard_allowed is not True or self.guard_stamp is None:
            return 'GUARD_DENIED'
        if now - self.guard_stamp > self.guard_timeout:
            return 'GUARD_STALE'
        return None


class DockingTaskBridge(Node):
    def __init__(self):
        super().__init__('docking_task_bridge')
        self._declare_parameters()

        mapping_file = self.get_parameter('dock_mapping_file').value
        if not mapping_file:
            raise ValueError('dock_mapping_file must be set')
        with open(mapping_file, encoding='utf-8') as stream:
            self.specs = load_dock_specs(yaml.safe_load(stream))

        target_tag_id = int(self.get_parameter('target_tag_id').value)
        try:
            self.target_dock = self.specs[target_tag_id]
        except KeyError as exc:
            raise ValueError(f'target_tag_id {target_tag_id} is not mapped') from exc

        self.policy = TaskPolicy(
            guard_required=bool(self.get_parameter('guard_required').value),
            guard_timeout=float(self.get_parameter('guard_timeout').value),
        )
        self.max_staging_time = float(
            self.get_parameter('max_staging_time').value
        )
        self.navigate_to_staging_pose = bool(
            self.get_parameter('navigate_to_staging_pose').value
        )

        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
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
        self.guard_subscription = self.create_subscription(
            Bool,
            self.get_parameter('guard_topic').value,
            self._on_guard,
            guard_qos_profile(),
        )
        self.action_client = ActionClient(
            self,
            DockRobot,
            self.get_parameter('dock_action_name').value,
        )
        self.start_service = self.create_service(
            Trigger,
            self.get_parameter('start_service').value,
            self._on_start,
        )
        self.cancel_service = self.create_service(
            Trigger,
            self.get_parameter('cancel_service').value,
            self._on_cancel,
        )
        self._goal_handle = None
        self._guard_cancel_reason = None
        self._last_state = None
        self.guard_timer = self.create_timer(0.1, self._check_guard)
        self._publish_state('IDLE', DiagnosticStatus.OK)

    def _declare_parameters(self):
        defaults = {
            'dock_mapping_file': '',
            'target_tag_id': 0,
            'dock_action_name': '/dock_robot',
            'start_service': '/demo2/start_docking',
            'cancel_service': '/demo2/cancel_docking',
            'guard_topic': '/guard/docking_allowed',
            'guard_required': False,
            'guard_timeout': 2.0,
            'state_topic': '/demo2/docking_state',
            'max_staging_time': 60.0,
            'navigate_to_staging_pose': True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _on_start(self, _request, response):
        allowed, reason = self.policy.can_start(self._now_seconds())
        if not allowed:
            response.success = False
            response.message = reason
            self._publish_state(reason, DiagnosticStatus.WARN)
            return response
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            response.success = False
            response.message = 'DOCK_ACTION_UNAVAILABLE'
            self._publish_state(response.message, DiagnosticStatus.ERROR)
            return response

        goal = DockRobot.Goal()
        goal.use_dock_id = True
        goal.dock_id = self.target_dock.dock_id
        goal.max_staging_time = self.max_staging_time
        goal.navigate_to_staging_pose = self.navigate_to_staging_pose

        self.policy.action_active = True
        self._guard_cancel_reason = None
        future = self.action_client.send_goal_async(
            goal,
            feedback_callback=self._on_feedback,
        )
        future.add_done_callback(self._on_goal_response)
        response.success = True
        response.message = 'DOCKING_REQUESTED'
        self._publish_state(
            'WAITING_FOR_ACTION',
            DiagnosticStatus.OK,
            {'dock_id': self.target_dock.dock_id},
        )
        return response

    def _on_cancel(self, _request, response):
        if not self.policy.action_active or self._goal_handle is None:
            response.success = False
            response.message = 'NO_ACTIVE_DOCKING'
            return response
        self._goal_handle.cancel_goal_async()
        response.success = True
        response.message = 'CANCEL_REQUESTED'
        return response

    def _on_guard(self, message):
        self.policy.update_guard(message.data, self._now_seconds())
        self._check_guard()

    def _check_guard(self):
        reason = self.policy.cancel_reason(self._now_seconds())
        if reason and self._goal_handle is not None and self._guard_cancel_reason is None:
            self._guard_cancel_reason = reason
            self._goal_handle.cancel_goal_async()
            self._publish_state(reason, DiagnosticStatus.ERROR)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.policy.action_active = False
            self._publish_state('FAILED', DiagnosticStatus.ERROR, {'reason': 'REJECTED'})
            return
        self._goal_handle = goal_handle
        self._check_guard()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_feedback(self, feedback_message):
        feedback = feedback_message.feedback
        state = feedback_state_name(feedback.state)
        self._publish_state(
            state,
            DiagnosticStatus.OK,
            {'num_retries': feedback.num_retries},
        )

    def _on_result(self, future):
        wrapped = future.result()
        result = wrapped.result
        if wrapped.status == GoalStatus.STATUS_CANCELED:
            state = self._guard_cancel_reason or 'CANCELED'
            level = DiagnosticStatus.WARN
        elif result.success:
            state = 'SUCCEEDED'
            level = DiagnosticStatus.OK
        else:
            state = 'FAILED'
            level = DiagnosticStatus.ERROR

        self.policy.action_active = False
        self._goal_handle = None
        self._publish_state(
            state,
            level,
            {
                'error_code': result.error_code,
                'error_msg': getattr(result, 'error_msg', ''),
                'num_retries': result.num_retries,
            },
        )

    def _publish_state(self, state, level, values=None):
        if state != self._last_state:
            self.state_publisher.publish(String(data=state))
            self._last_state = state

        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        diagnostics.status.append(
            make_status(self.get_name(), level, state, values or {})
        )
        self.diagnostic_publisher.publish(diagnostics)

    def _now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DockingTaskBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except (OSError, ValueError, yaml.YAMLError) as exc:
        if node is not None:
            node.get_logger().fatal(str(exc))
        else:
            print(f'docking_task_bridge: {exc}')
    finally:
        if node is not None:
            node.destroy_node()
        shutdown_if_running()


if __name__ == '__main__':
    main()
