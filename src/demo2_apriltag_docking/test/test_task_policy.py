import math
import sys

from action_msgs.msg import GoalStatus
from demo2_apriltag_docking import docking_task_bridge
from demo2_apriltag_docking.docking_task_bridge import (
    feedback_state_name,
    FLOAT32_MAX,
    TaskPolicy,
    validate_max_staging_time,
)
from diagnostic_msgs.msg import DiagnosticStatus
import pytest
from rclpy.qos import (
    DurabilityPolicy,
    qos_check_compatible,
    QoSCompatibility,
    QoSProfile,
    ReliabilityPolicy,
)


def test_required_guard_denies_without_message():
    policy = TaskPolicy(guard_required=True, guard_timeout=2.0)

    assert policy.can_start(now=1.0) == (False, 'GUARD_DENIED')


def test_optional_guard_allows_without_message():
    policy = TaskPolicy(guard_required=False, guard_timeout=2.0)

    assert policy.can_start(now=1.0) == (True, 'READY')


def test_fresh_true_guard_allows_start():
    policy = TaskPolicy(guard_required=True, guard_timeout=2.0)
    policy.update_guard(True, stamp=1.0)

    assert policy.can_start(now=2.0) == (True, 'READY')


def test_false_guard_denies_start():
    policy = TaskPolicy(guard_required=True, guard_timeout=2.0)
    policy.update_guard(False, stamp=1.0)

    assert policy.can_start(now=1.0) == (False, 'GUARD_DENIED')


def test_stale_guard_denies_start():
    policy = TaskPolicy(guard_required=True, guard_timeout=2.0)
    policy.update_guard(True, stamp=1.0)

    assert policy.can_start(now=3.1) == (False, 'GUARD_STALE')


def test_active_action_rejects_duplicate_start():
    policy = TaskPolicy(guard_required=False, guard_timeout=2.0)
    policy.mark_active()

    assert policy.can_start(now=1.0) == (False, 'ALREADY_ACTIVE')


def test_active_action_cancels_when_guard_turns_false():
    policy = TaskPolicy(guard_required=True, guard_timeout=2.0)
    policy.mark_active()
    policy.update_guard(False, stamp=1.0)

    assert policy.cancel_reason(now=1.0) == 'GUARD_DENIED'


def test_active_action_cancels_when_guard_becomes_stale():
    policy = TaskPolicy(guard_required=True, guard_timeout=2.0)
    policy.mark_active()
    policy.update_guard(True, stamp=1.0)

    assert policy.cancel_reason(now=3.1) == 'GUARD_STALE'


def test_optional_guard_never_cancels_action():
    policy = TaskPolicy(guard_required=False, guard_timeout=2.0)
    policy.mark_active()

    assert policy.cancel_reason(now=100.0) is None


def test_action_state_is_managed_by_explicit_methods():
    policy = TaskPolicy(guard_required=False, guard_timeout=2.0)

    policy.mark_active()
    assert policy.action_active is True

    policy.mark_idle()
    assert policy.action_active is False
    assert TaskPolicy.action_active.fset is None


@pytest.mark.parametrize(
    'guard_timeout',
    [0.0, -1.0, float('nan'), float('inf'), -float('inf')],
)
def test_task_policy_rejects_invalid_guard_timeout(guard_timeout):
    with pytest.raises(ValueError, match='guard_timeout must be > 0'):
        TaskPolicy(guard_required=False, guard_timeout=guard_timeout)


@pytest.mark.parametrize(
    'max_staging_time',
    [
        0.0,
        -1.0,
        float('nan'),
        float('inf'),
        -float('inf'),
        sys.float_info.max,
        5e-324,
        math.nextafter(FLOAT32_MAX, math.inf),
    ],
)
def test_rejects_max_staging_time_outside_positive_float32(max_staging_time):
    message = (
        'max_staging_time must be finite and representable as a positive float32'
    )
    with pytest.raises(ValueError, match=message):
        validate_max_staging_time(max_staging_time)


@pytest.mark.parametrize(
    'max_staging_time',
    [60.0, FLOAT32_MAX, 2.0 ** -149],
)
def test_accepts_positive_float32_max_staging_time(max_staging_time):
    assert validate_max_staging_time(max_staging_time) == max_staging_time


class FailingFuture:

    def __init__(self, error):
        self.error = error

    def result(self):
        raise self.error


def make_fake_bridge():
    bridge = docking_task_bridge.DockingTaskBridge.__new__(
        docking_task_bridge.DockingTaskBridge
    )
    bridge.policy = TaskPolicy(guard_required=False, guard_timeout=2.0)
    bridge.policy.mark_active()
    bridge._goal_handle = object()
    bridge._guard_cancel_reason = None
    bridge._cancel_sent = False
    bridge.published_states = []
    bridge._publish_state = lambda *args: bridge.published_states.append(args)
    return bridge


def test_goal_response_future_error_clears_action_and_reports_failure():
    bridge = make_fake_bridge()

    bridge._on_goal_response(FailingFuture(RuntimeError('send failed')))

    assert bridge.policy.action_active is False
    assert bridge._goal_handle is None
    assert bridge.published_states == [
        (
            'FAILED',
            DiagnosticStatus.ERROR,
            {'reason': 'GOAL_RESPONSE_ERROR', 'error_msg': 'send failed'},
        )
    ]


def test_result_future_error_clears_action_and_reports_failure():
    bridge = make_fake_bridge()

    bridge._on_result(FailingFuture(RuntimeError('result failed')))

    assert bridge.policy.action_active is False
    assert bridge._goal_handle is None
    assert bridge.published_states == [
        (
            'FAILED',
            DiagnosticStatus.ERROR,
            {'reason': 'RESULT_ERROR', 'error_msg': 'result failed'},
        )
    ]


class FakeGoalHandle:

    def __init__(self, *, accepted=True):
        self.accepted = accepted
        self.cancel_calls = 0

    def cancel_goal_async(self):
        self.cancel_calls += 1

    def get_result_async(self):
        return CallbackFuture()


class CallbackFuture:

    def add_done_callback(self, callback):
        self.callback = callback


class ValueFuture:

    def __init__(self, value):
        self.value = value

    def result(self):
        return self.value


class FakeResult:

    def __init__(self, *, success):
        self.success = success
        self.error_code = 0
        self.error_msg = ''
        self.num_retries = 0


class FakeWrappedResult:

    def __init__(self, *, status, success):
        self.status = status
        self.result = FakeResult(success=success)


def test_aborted_action_cannot_succeed_from_payload_alone():
    bridge = make_fake_bridge()

    bridge._on_result(ValueFuture(FakeWrappedResult(
        status=GoalStatus.STATUS_ABORTED,
        success=True,
    )))

    assert bridge.policy.action_active is False
    assert bridge.published_states == [
        (
            'FAILED',
            DiagnosticStatus.ERROR,
            {'error_code': 0, 'error_msg': '', 'num_retries': 0},
        )
    ]


def test_guard_cancel_is_latched_while_goal_response_is_pending():
    bridge = make_fake_bridge()
    bridge.policy = TaskPolicy(guard_required=True, guard_timeout=2.0)
    bridge.policy.mark_active()
    bridge._goal_handle = None
    bridge._now_seconds = lambda: 1.0
    bridge.policy.update_guard(False, stamp=1.0)

    bridge._check_guard()

    assert bridge._guard_cancel_reason == 'GUARD_DENIED'
    assert bridge._cancel_sent is False
    assert bridge.published_states == [
        ('GUARD_DENIED', DiagnosticStatus.ERROR)
    ]

    bridge.policy.update_guard(True, stamp=1.1)
    goal_handle = FakeGoalHandle()
    bridge._on_goal_response(ValueFuture(goal_handle))

    assert goal_handle.cancel_calls == 1
    assert bridge._cancel_sent is True

    bridge._check_guard()
    assert goal_handle.cancel_calls == 1


@pytest.mark.parametrize(
    ('value', 'name'),
    [
        (1, 'NAV_TO_STAGING'),
        (2, 'INITIAL_PERCEPTION'),
        (3, 'CONTROLLING'),
        (4, 'WAIT_FOR_CHARGE'),
        (5, 'RETRY'),
        (99, 'UNKNOWN'),
    ],
)
def test_feedback_state_mapping(value, name):
    assert feedback_state_name(value) == name


def test_guard_subscription_accepts_volatile_publishers():
    publisher_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )

    subscription_qos = docking_task_bridge.guard_qos_profile()
    compatibility, _ = qos_check_compatible(
        publisher_qos,
        subscription_qos,
    )

    assert subscription_qos.durability == DurabilityPolicy.VOLATILE
    assert compatibility != QoSCompatibility.ERROR
