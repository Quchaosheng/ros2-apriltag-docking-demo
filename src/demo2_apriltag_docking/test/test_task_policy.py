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


def test_cancel_before_the_goal_response_is_remembered():
    """Issue #14: a cancel with no handle yet must not be forgotten."""
    policy = TaskPolicy(guard_required=False, guard_timeout=2.0)
    policy.mark_active()
    assert policy.pending_cancel() is None

    assert policy.request_cancel() is True

    pending = policy.pending_cancel()
    assert pending is not None
    assert pending['request_id'] == policy.request_id


def test_cancel_while_idle_is_refused():
    policy = TaskPolicy(guard_required=False, guard_timeout=2.0)

    assert policy.request_cancel() is False
    assert policy.pending_cancel() is None


def test_cancel_intent_belongs_to_one_request():
    policy = TaskPolicy(guard_required=False, guard_timeout=2.0)
    first = policy.mark_active()
    policy.request_cancel()

    # A new request starts: the old intent must not carry over and cancel it.
    second = policy.mark_active()
    assert second != first
    assert policy.pending_cancel() is None

    policy.request_cancel()
    assert policy.pending_cancel()['request_id'] == second


def test_clear_cancel_ignores_a_stale_request_id():
    policy = TaskPolicy(guard_required=False, guard_timeout=2.0)
    policy.mark_active()
    policy.request_cancel()
    current = policy.request_id

    # A late callback from an older request must not clear the current intent.
    policy.clear_cancel(current - 1)
    assert policy.pending_cancel() is not None

    policy.clear_cancel(current)
    assert policy.pending_cancel() is None


def test_mark_idle_drops_a_pending_cancel():
    policy = TaskPolicy(guard_required=False, guard_timeout=2.0)
    policy.mark_active()
    policy.request_cancel()

    policy.mark_idle()

    assert policy.pending_cancel() is None
    assert policy.action_active is False


class FakeGoalHandle:
    """Stands in for the handle the action client produces on acceptance."""

    def __init__(self):
        self.cancelled = 0

    def cancel_goal_async(self):
        self.cancelled += 1

    def get_result_async(self):
        class _Future:
            def add_done_callback(self, _callback):
                pass

        return _Future()


def test_bridge_applies_a_latched_cancel_when_the_goal_is_accepted():
    """The path the issue describes, without a live ROS graph.

    _on_goal_response() is the only place a late acceptance turns into a handle,
    so that is what is exercised: a cancel latched while the response was
    outstanding has to become a cancel_goal_async() on the accepted handle.
    """
    bridge = docking_task_bridge.DockingTaskBridge.__new__(
        docking_task_bridge.DockingTaskBridge)
    bridge.policy = TaskPolicy(guard_required=False, guard_timeout=2.0)
    bridge.policy.mark_active()
    bridge._goal_handle = None
    bridge._guard_cancel_reason = None
    bridge._publish_state = lambda *args, **kwargs: None
    bridge._now_seconds = lambda: 1.0

    # The caller asks to cancel before the server has answered.
    assert bridge.policy.request_cancel() is True

    handle = FakeGoalHandle()
    future = _accepted_future(handle)
    docking_task_bridge.DockingTaskBridge._on_goal_response(bridge, future)

    assert bridge._goal_handle is handle
    assert handle.cancelled == 1, 'the accepted goal was not cancelled'
    assert bridge.policy.pending_cancel() is None


def test_bridge_does_not_cancel_a_rejected_goal():
    bridge = docking_task_bridge.DockingTaskBridge.__new__(
        docking_task_bridge.DockingTaskBridge)
    bridge.policy = TaskPolicy(guard_required=False, guard_timeout=2.0)
    bridge.policy.mark_active()
    bridge._goal_handle = None
    bridge._guard_cancel_reason = None
    bridge._publish_state = lambda *args, **kwargs: None
    bridge._now_seconds = lambda: 1.0
    bridge.policy.request_cancel()

    handle = FakeGoalHandle()
    handle.accepted = False
    docking_task_bridge.DockingTaskBridge._on_goal_response(
        bridge, _accepted_future(handle))

    assert handle.cancelled == 0
    assert bridge.policy.action_active is False


def test_bridge_cancel_service_without_a_handle_latches_instead_of_failing():
    bridge = docking_task_bridge.DockingTaskBridge.__new__(
        docking_task_bridge.DockingTaskBridge)
    bridge.policy = TaskPolicy(guard_required=False, guard_timeout=2.0)
    bridge.policy.mark_active()
    bridge._goal_handle = None

    class _Response:
        success = False
        message = ''

    response = _Response()
    docking_task_bridge.DockingTaskBridge._on_cancel(bridge, None, response)

    assert response.success is True
    assert response.message == 'CANCEL_REQUESTED'
    assert bridge.policy.pending_cancel() is not None


def _accepted_future(handle):
    class _Future:
        def result(self):
            return handle

    return _Future()


def test_guard_denial_survives_a_recovery_before_the_response():
    """A guard that recovers during the wait must not un-cancel the request.

    _check_guard() cancels through the handle, and there is no handle yet, so
    the denial has to be latched the same way an operator's cancel is.
    """
    bridge = docking_task_bridge.DockingTaskBridge.__new__(
        docking_task_bridge.DockingTaskBridge)
    bridge.policy = TaskPolicy(guard_required=True, guard_timeout=2.0)
    bridge.policy.mark_active()
    bridge._goal_handle = None
    bridge._guard_cancel_reason = None
    bridge._publish_state = lambda *args, **kwargs: None
    bridge._now_seconds = lambda: 1.0

    # The guard denies while the goal response is still outstanding ...
    bridge.policy.update_guard(False, 1.0)
    docking_task_bridge.DockingTaskBridge._check_guard(bridge)
    assert bridge.policy.pending_cancel() is not None
    assert bridge._guard_cancel_reason == 'GUARD_DENIED'

    # ... and recovers before the server answers. The denial still applies.
    bridge.policy.update_guard(True, 1.0)

    handle = FakeGoalHandle()
    docking_task_bridge.DockingTaskBridge._on_goal_response(
        bridge, _accepted_future(handle))

    assert bridge._goal_handle is handle
    assert handle.cancelled == 1, 'a recovered guard un-cancelled the request'
