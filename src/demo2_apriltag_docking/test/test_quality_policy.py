import math

from demo2_apriltag_docking.quality_policy import (
    pose_delta,
    validate_camera_calibration,
    validate_observation_timing,
)
import pytest


def test_accepts_fresh_detection_and_transform():
    result = validate_observation_timing(
        now=10.0,
        detection_stamp=9.9,
        tf_stamp=9.85,
        max_detection_age=0.25,
        max_tf_age=0.25,
        max_future_skew=0.05,
    )

    assert result.accepted is True
    assert result.detection_age_ms == pytest.approx(100.0)
    assert result.tf_age_ms == pytest.approx(150.0)


def test_rejects_stale_and_future_temporal_inputs():
    stale = validate_observation_timing(
        now=10.0,
        detection_stamp=9.7,
        tf_stamp=9.9,
        max_detection_age=0.25,
        max_tf_age=0.25,
        max_future_skew=0.05,
    )
    future = validate_observation_timing(
        now=10.0,
        detection_stamp=10.1,
        tf_stamp=10.0,
        max_detection_age=0.25,
        max_tf_age=0.25,
        max_future_skew=0.05,
    )

    assert stale.reason == 'DETECTION_STALE'
    assert future.reason == 'DETECTION_FUTURE'


def test_validates_metric_camera_intrinsics():
    valid = validate_camera_calibration(
        640,
        480,
        (500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0),
        'plumb_bob',
    )
    invalid = validate_camera_calibration(
        640,
        480,
        (0.0, 0.0, 320.0, 0.0, 0.0, 240.0, 0.0, 0.0, 1.0),
        'plumb_bob',
    )

    assert valid.valid is True
    assert valid.fx == 500.0
    assert invalid.reason == 'FOCAL_LENGTH_INVALID'


def test_rejects_missing_intrinsics_without_raising():
    result = validate_camera_calibration(640, 480, None, 'plumb_bob')

    assert result.valid is False
    assert result.reason == 'INTRINSICS_INVALID'


def test_pose_delta_wraps_yaw_across_pi():
    translation, yaw = pose_delta(
        (0.0, 0.0, 0.0, math.radians(179.0)),
        (0.1, 0.0, 0.0, math.radians(-179.0)),
    )

    assert translation == 0.1
    assert math.isclose(yaw, math.radians(2.0))
