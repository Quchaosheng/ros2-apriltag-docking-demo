from demo2_apriltag_docking import docking_task_bridge
from demo2_apriltag_docking.docking_task_bridge import (
    feedback_state_name,
    TaskPolicy,
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
    bridge._cancel_requested = False
    bridge._request_id = 1
    bridge.published_states = []
    bridge._publish_state = lambda *args: bridge.published_states.append(args)
    return bridge


class FakeFuture:

    def __init__(self, value=None, error=None):
        self._value = value
        self._error = error

    def result(self):
        if self._error is not None:
            raise self._error
        return self._value


class FakeGoalHandle:

    def __init__(self, accepted=True):
        self.accepted = accepted
        self.cancel_calls = 0
        self.result_callbacks = 0

    def cancel_goal_async(self):
        self.cancel_calls += 1

    def get_result_async(self):
        self.result_callbacks += 1
        return FakeFuture(value=None)


class FakeResponse:

    def __init__(self):
        self.success = None
        self.message = None


def make_pending_bridge(guard_required=False):
    """Bridge in the window between mark_active() and goal acceptance."""
    bridge = docking_task_bridge.DockingTaskBridge.__new__(
        docking_task_bridge.DockingTaskBridge
    )
    bridge.policy = TaskPolicy(guard_required=guard_required, guard_timeout=2.0)
    bridge.policy.mark_active()
    bridge._goal_handle = None
    bridge._guard_cancel_reason = None
    bridge._cancel_requested = False
    bridge._request_id = 1
    bridge.published_states = []
    bridge._publish_state = lambda *args: bridge.published_states.append(args)
    bridge._now_seconds = lambda: 1.0
    return bridge


def test_goal_response_future_error_clears_action_and_reports_failure():
    bridge = make_fake_bridge()

    bridge._on_goal_response(FailingFuture(RuntimeError('send failed')), 1)

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

    bridge._on_result(FailingFuture(RuntimeError('result failed')), 1)

    assert bridge.policy.action_active is False
    assert bridge._goal_handle is None
    assert bridge.published_states == [
        (
            'FAILED',
            DiagnosticStatus.ERROR,
            {'reason': 'RESULT_ERROR', 'error_msg': 'result failed'},
        )
    ]


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


def test_cancel_before_acceptance_is_retained():
    """Issue #14: the intent must survive the goal acceptance window."""
    bridge = make_pending_bridge()

    response = bridge._on_cancel(None, FakeResponse())

    assert response.success is True
    assert response.message == 'CANCEL_REQUESTED'
    assert bridge._cancel_requested is True
    assert bridge.policy.action_active is True


def test_retained_cancel_cancels_the_eventually_accepted_goal():
    bridge = make_pending_bridge()
    bridge._on_cancel(None, FakeResponse())
    handle = FakeGoalHandle()

    bridge._on_goal_response(FakeFuture(value=handle), 1)

    assert handle.cancel_calls == 1
    assert handle.result_callbacks == 0
    assert bridge._cancel_requested is False


def test_retained_cancel_does_not_survive_rejection():
    bridge = make_pending_bridge()
    bridge._on_cancel(None, FakeResponse())

    bridge._on_goal_response(FakeFuture(value=FakeGoalHandle(accepted=False)), 1)

    assert bridge.policy.action_active is False
    assert bridge._cancel_requested is False
    assert bridge._goal_handle is None


def test_duplicate_cancel_is_idempotent():
    bridge = make_pending_bridge()

    first = bridge._on_cancel(None, FakeResponse())
    second = bridge._on_cancel(None, FakeResponse())

    assert (first.success, second.success) == (True, True)
    assert bridge._cancel_requested is True


def test_cancel_without_active_action_is_rejected():
    bridge = make_pending_bridge()
    bridge.policy.mark_idle()

    response = bridge._on_cancel(None, FakeResponse())

    assert response.success is False
    assert response.message == 'NO_ACTIVE_DOCKING'
    assert bridge._cancel_requested is False


def test_guard_denial_before_acceptance_stays_latched_after_recovery():
    bridge = make_pending_bridge(guard_required=True)
    bridge.policy.update_guard(False, stamp=1.0)

    bridge._check_guard()

    assert bridge._guard_cancel_reason == 'GUARD_DENIED'

    # The guard recovers while the goal response is still in flight.
    bridge.policy.update_guard(True, stamp=1.0)
    handle = FakeGoalHandle()

    bridge._on_goal_response(FakeFuture(value=handle), 1)

    assert handle.cancel_calls == 1
    assert handle.result_callbacks == 0


def test_stale_goal_response_does_not_affect_current_request():
    bridge = make_pending_bridge()
    bridge._request_id = 2
    handle = FakeGoalHandle()

    bridge._on_goal_response(FakeFuture(value=handle), 1)

    assert bridge._goal_handle is None
    assert handle.cancel_calls == 0
    assert handle.result_callbacks == 0
    assert bridge.published_states == []


def test_stale_result_is_ignored():
    bridge = make_pending_bridge()
    bridge._request_id = 2

    bridge._on_result(FailingFuture(RuntimeError('late')), 1)

    assert bridge.policy.action_active is True
    assert bridge.published_states == []
