import math

from demo2_apriltag_docking.tag_policy import (
    Detection,
    DockSpec,
    load_dock_specs,
    TagGate,
)
import pytest


def make_detection(
    tag_id=0,
    *,
    hamming=0,
    margin=60.0,
    stamp=1.0,
    x=1.0,
    y=0.0,
    yaw=0.0,
):
    return Detection(tag_id, hamming, margin, stamp, x, y, yaw)


@pytest.fixture
def dock():
    return DockSpec(0, 'demo_charge_dock', 'charging_dock', 'tag36h11:0')


@pytest.fixture
def gate(dock):
    return TagGate(
        specs={0: dock},
        min_margin=50.0,
        max_hamming=0,
        confirmations=3,
        confirmation_window=0.5,
        publish_period=0.1,
        loss_timeout=0.5,
        max_translation_jump=0.25,
        max_yaw_jump=math.radians(20.0),
    )


def test_load_dock_specs_converts_yaml_keys():
    specs = load_dock_specs(
        {
            'docks': {
                '0': {
                    'dock_id': 'demo_charge_dock',
                    'dock_type': 'charging_dock',
                    'tag_frame': 'tag36h11:0',
                }
            }
        }
    )

    assert specs == {
        0: DockSpec(0, 'demo_charge_dock', 'charging_dock', 'tag36h11:0')
    }


@pytest.mark.parametrize(
    'raw',
    [
        {},
        {'docks': []},
        {'docks': {-1: {'dock_id': 'a', 'dock_type': 'b', 'tag_frame': 'c'}}},
        {'docks': {0: {'dock_id': '', 'dock_type': 'b', 'tag_frame': 'c'}}},
        {
            'docks': {
                0: {'dock_id': 'same', 'dock_type': 'a', 'tag_frame': 'tag:0'},
                1: {'dock_id': 'same', 'dock_type': 'b', 'tag_frame': 'tag:1'},
            }
        },
        {
            'docks': {
                0: {'dock_id': 'a', 'dock_type': 'a', 'tag_frame': 'tag:0'},
                '0': {'dock_id': 'b', 'dock_type': 'b', 'tag_frame': 'tag:1'},
            }
        },
    ],
)
def test_load_dock_specs_rejects_invalid_mapping(raw):
    with pytest.raises(ValueError):
        load_dock_specs(raw)


def test_rejects_no_tag(gate):
    assert gate.evaluate([], now=1.0).reason == 'NO_TAG'


def test_rejects_unknown_tag(gate):
    result = gate.evaluate([make_detection(tag_id=99)], now=1.0)
    assert result.reason == 'UNKNOWN_TAG'


def test_rejects_low_margin(gate):
    result = gate.evaluate([make_detection(margin=49.9)], now=1.0)
    assert result.reason == 'LOW_MARGIN'


def test_rejects_bad_hamming(gate):
    result = gate.evaluate([make_detection(hamming=1)], now=1.0)
    assert result.reason == 'HAMMING'


def test_rejects_multiple_tags(gate):
    result = gate.evaluate(
        [make_detection(tag_id=0), make_detection(tag_id=1)],
        now=1.0,
    )
    assert result.reason == 'MULTI_TAG'


def test_requires_three_consecutive_frames(gate, dock):
    first = gate.evaluate([make_detection(stamp=1.0)], now=1.0)
    second = gate.evaluate([make_detection(stamp=1.1)], now=1.1)
    third = gate.evaluate([make_detection(stamp=1.2)], now=1.2)

    assert first.reason == 'CONFIRMING'
    assert second.reason == 'CONFIRMING'
    assert third.accepted is True
    assert third.reason == 'ACCEPTED'
    assert third.dock == dock


def test_confirmation_window_restarts(gate):
    assert gate.evaluate([make_detection(stamp=1.0)], now=1.0).reason == 'CONFIRMING'
    assert gate.evaluate([make_detection(stamp=1.1)], now=1.1).reason == 'CONFIRMING'
    assert gate.evaluate([make_detection(stamp=1.7)], now=1.7).reason == 'CONFIRMING'
    assert gate.evaluate([make_detection(stamp=1.8)], now=1.8).reason == 'CONFIRMING'
    assert gate.evaluate([make_detection(stamp=1.9)], now=1.9).accepted is True


def test_rate_limits_confirmed_pose(gate):
    gate.evaluate([make_detection(stamp=1.0)], now=1.0)
    gate.evaluate([make_detection(stamp=1.1)], now=1.1)
    assert gate.evaluate([make_detection(stamp=1.2)], now=1.2).accepted is True

    limited = gate.evaluate([make_detection(stamp=1.25)], now=1.25)
    refreshed = gate.evaluate([make_detection(stamp=1.31)], now=1.31)

    assert limited.reason == 'RATE_LIMITED'
    assert refreshed.accepted is True


def test_translation_jump_resets_confirmation(gate):
    gate.evaluate([make_detection(stamp=1.0)], now=1.0)
    gate.evaluate([make_detection(stamp=1.1)], now=1.1)
    gate.evaluate([make_detection(stamp=1.2)], now=1.2)

    jumped = gate.evaluate(
        [make_detection(stamp=1.4, x=1.30)],
        now=1.4,
    )
    next_sample = gate.evaluate(
        [make_detection(stamp=1.5, x=1.02)],
        now=1.5,
    )

    assert jumped.reason == 'POSE_JUMP'
    assert next_sample.reason == 'CONFIRMING'


def test_yaw_jump_uses_wrapped_angle(gate):
    gate.evaluate([make_detection(stamp=1.0, yaw=math.radians(179.0))], now=1.0)
    gate.evaluate([make_detection(stamp=1.1, yaw=math.radians(179.0))], now=1.1)
    gate.evaluate([make_detection(stamp=1.2, yaw=math.radians(179.0))], now=1.2)

    wrapped = gate.evaluate(
        [make_detection(stamp=1.4, yaw=math.radians(-179.0))],
        now=1.4,
    )

    assert wrapped.reason == 'ACCEPTED'


def test_reports_tag_loss_after_timeout(gate):
    gate.evaluate([make_detection(stamp=1.0)], now=1.0)

    assert gate.loss_reason(now=1.5) is None
    assert gate.loss_reason(now=1.51) == 'TAG_LOST'


def test_tag_loss_requires_confirmation_after_recovery(gate):
    for now in (1.0, 1.1, 1.2):
        gate.evaluate([make_detection(stamp=now)], now=now)

    assert gate.loss_reason(now=1.8) == 'TAG_LOST'
    assert gate.evaluate(
        [make_detection(stamp=1.8)], now=1.8
    ).reason == 'CONFIRMING'


def test_recovery_resets_confirmation_before_loss_timer_runs(gate):
    for now in (1.0, 1.1, 1.2):
        gate.evaluate([make_detection(stamp=now)], now=now)

    recovered = gate.evaluate([make_detection(stamp=1.8)], now=1.8)

    assert recovered.reason == 'CONFIRMING'
