from demo2_apriltag_docking import docking_task_bridge
from demo2_apriltag_docking.docking_task_bridge import (
    feedback_state_name,
    TaskPolicy,
)
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
    policy.action_active = True

    assert policy.can_start(now=1.0) == (False, 'ALREADY_ACTIVE')


def test_active_action_cancels_when_guard_turns_false():
    policy = TaskPolicy(guard_required=True, guard_timeout=2.0)
    policy.action_active = True
    policy.update_guard(False, stamp=1.0)

    assert policy.cancel_reason(now=1.0) == 'GUARD_DENIED'


def test_active_action_cancels_when_guard_becomes_stale():
    policy = TaskPolicy(guard_required=True, guard_timeout=2.0)
    policy.action_active = True
    policy.update_guard(True, stamp=1.0)

    assert policy.cancel_reason(now=3.1) == 'GUARD_STALE'


def test_optional_guard_never_cancels_action():
    policy = TaskPolicy(guard_required=False, guard_timeout=2.0)
    policy.action_active = True

    assert policy.cancel_reason(now=100.0) is None


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
