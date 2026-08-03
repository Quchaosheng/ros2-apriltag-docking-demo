import math

from demo2_apriltag_docking.bag_quality import (
    CalibrationSample,
    DetectionSample,
    PoseSample,
    summarize_observations,
    TransformSample,
)
from demo2_apriltag_docking.quality_policy import validate_camera_calibration


def _valid_calibration(stamp=1.0):
    return CalibrationSample(
        recorded_at=stamp,
        stamp=stamp,
        calibration=validate_camera_calibration(
            640,
            480,
            (500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0),
            'plumb_bob',
        ),
    )


def test_summary_matches_fresh_tf_and_detects_published_pose_jump():
    report = summarize_observations(
        detections=[DetectionSample(1.05, 1.0, 0)],
        transforms=[TransformSample(1.01, 'tag36h11:0')],
        calibrations=[_valid_calibration()],
        poses=[
            PoseSample(1.0, 1.0, (0.0, 0.0, 0.0, 0.0)),
            PoseSample(1.1, 1.1, (0.4, 0.0, 0.0, math.radians(30.0))),
        ],
        tag_frame_prefix='tag36h11',
        max_detection_age=0.25,
        max_tf_age=0.25,
        max_future_skew=0.05,
        max_translation_jump=0.25,
        max_yaw_jump=math.radians(20.0),
    )

    assert report['observation_quality']['by_reason'] == {'ACCEPTED': 1}
    assert report['observation_quality']['valid_camera_info_samples'] == 1
    assert report['published_pose_quality']['jump_count'] == 1
    assert report['published_pose_quality']['jumps'][0]['translation_m'] == 0.4


def test_summary_reports_missing_tf_without_calling_it_accepted():
    report = summarize_observations(
        detections=[DetectionSample(1.05, 1.0, 2)],
        transforms=[],
        calibrations=[_valid_calibration()],
        poses=[],
        tag_frame_prefix='tag36h11',
        max_detection_age=0.25,
        max_tf_age=0.25,
        max_future_skew=0.05,
        max_translation_jump=0.25,
        max_yaw_jump=math.radians(20.0),
    )

    assert report['observation_quality']['by_reason'] == {'TF_UNAVAILABLE': 1}


def test_summary_reports_stale_tf():
    report = summarize_observations(
        detections=[DetectionSample(2.0, 1.9, 0)],
        transforms=[TransformSample(1.5, 'tag36h11:0')],
        calibrations=[_valid_calibration()],
        poses=[],
        tag_frame_prefix='tag36h11',
        max_detection_age=0.25,
        max_tf_age=0.25,
        max_future_skew=0.05,
        max_translation_jump=0.25,
        max_yaw_jump=math.radians(20.0),
    )

    assert report['observation_quality']['by_reason'] == {'TF_STALE': 1}
