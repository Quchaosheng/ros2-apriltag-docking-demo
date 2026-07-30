"""Offline quality report for a recorded AprilTag docking rosbag.

The reader is deliberately observational. It reports timing, TF association,
camera-intrinsic validity, and published-pose jumps without claiming that a
recorded DockRobot request completed on real hardware.
"""

import argparse
from bisect import bisect_left
from collections import Counter, defaultdict
from dataclasses import dataclass
import json
import math
from pathlib import Path
import re

from demo2_apriltag_docking.quality_policy import (
    CameraCalibration,
    camera_info_to_calibration,
    pose_delta,
    validate_observation_timing,
    yaw_from_quaternion,
)


@dataclass(frozen=True)
class DetectionSample:
    recorded_at: float
    stamp: float
    tag_id: int


@dataclass(frozen=True)
class TransformSample:
    stamp: float
    child_frame_id: str


@dataclass(frozen=True)
class CalibrationSample:
    recorded_at: float
    stamp: float
    calibration: CameraCalibration


@dataclass(frozen=True)
class PoseSample:
    recorded_at: float
    stamp: float
    pose: tuple[float, float, float, float]


def _finite(value):
    return isinstance(value, (int, float)) and math.isfinite(value)


def _message_seconds(message):
    stamp = message.header.stamp
    return stamp.sec + stamp.nanosec * 1e-9


def _nearest_sample(samples, stamp, key):
    """Return the nearest timestamped item, including a following TF sample."""
    if not samples or not _finite(stamp):
        return None
    values = [key(sample) for sample in samples]
    index = bisect_left(values, stamp)
    candidates = []
    if index:
        candidates.append(samples[index - 1])
    if index < len(samples):
        candidates.append(samples[index])
    return min(candidates, key=lambda sample: abs(key(sample) - stamp))


def _percentile(values, percentile):
    if not values:
        return None
    sorted_values = sorted(values)
    index = round((len(sorted_values) - 1) * percentile / 100.0)
    return round(sorted_values[index], 3)


def _summary(values):
    return {
        'count': len(values),
        'min': _percentile(values, 0),
        'p50': _percentile(values, 50),
        'p95': _percentile(values, 95),
        'p99': _percentile(values, 99),
        'max': _percentile(values, 100),
    }


def summarize_observations(
    *,
    detections,
    transforms,
    calibrations,
    poses,
    tag_frame_prefix,
    max_detection_age,
    max_tf_age,
    max_future_skew,
    max_translation_jump,
    max_yaw_jump,
):
    """Build a deterministic quality summary from decoded bag observations."""
    transforms_by_frame = defaultdict(list)
    for transform in transforms:
        transforms_by_frame[transform.child_frame_id].append(transform)
    for samples in transforms_by_frame.values():
        samples.sort(key=lambda sample: sample.stamp)
    calibrations = sorted(calibrations, key=lambda sample: sample.stamp)

    quality_reasons = Counter()
    calibration_reasons = Counter()
    detection_recording_age_ms = []
    tf_recording_age_ms = []
    tf_to_detection_delta_ms = []

    for detection in detections:
        if _finite(detection.recorded_at) and _finite(detection.stamp):
            detection_recording_age_ms.append(
                (detection.recorded_at - detection.stamp) * 1000.0
            )

        calibration = _nearest_sample(
            calibrations,
            detection.stamp,
            lambda sample: sample.stamp,
        )
        if calibration is None:
            quality_reasons['CALIBRATION_UNAVAILABLE'] += 1
            continue
        if not calibration.calibration.valid:
            quality_reasons['CALIBRATION_INVALID'] += 1
            calibration_reasons[calibration.calibration.reason] += 1
            continue

        frame_id = f'{tag_frame_prefix}:{detection.tag_id}'
        transform = _nearest_sample(
            transforms_by_frame.get(frame_id, []),
            detection.stamp,
            lambda sample: sample.stamp,
        )
        if transform is None:
            quality_reasons['TF_UNAVAILABLE'] += 1
            continue

        timing = validate_observation_timing(
            now=detection.recorded_at,
            detection_stamp=detection.stamp,
            tf_stamp=transform.stamp,
            max_detection_age=max_detection_age,
            max_tf_age=max_tf_age,
            max_future_skew=max_future_skew,
        )
        quality_reasons[timing.reason] += 1
        if timing.tf_age_ms is not None:
            tf_recording_age_ms.append(timing.tf_age_ms)
        if _finite(detection.stamp) and _finite(transform.stamp):
            tf_to_detection_delta_ms.append(
                abs(detection.stamp - transform.stamp) * 1000.0
            )

    pose_jumps = []
    ordered_poses = sorted(
        poses,
        key=lambda sample: (
            sample.stamp if _finite(sample.stamp) else sample.recorded_at
        ),
    )
    for previous, current in zip(ordered_poses, ordered_poses[1:]):
        translation, yaw = pose_delta(previous.pose, current.pose)
        if translation > max_translation_jump or yaw > max_yaw_jump:
            pose_jumps.append(
                {
                    'translation_m': round(translation, 6),
                    'yaw_deg': round(math.degrees(yaw), 6),
                }
            )

    valid_calibrations = sum(
        sample.calibration.valid for sample in calibrations
    )
    return {
        'schema_version': 1,
        'counts': {
            'detection_samples': len(detections),
            'tf_samples': len(transforms),
            'camera_info_samples': len(calibrations),
            'published_pose_samples': len(poses),
        },
        'thresholds': {
            'max_detection_age_ms': max_detection_age * 1000.0,
            'max_tf_age_ms': max_tf_age * 1000.0,
            'max_future_skew_ms': max_future_skew * 1000.0,
            'max_translation_jump_m': max_translation_jump,
            'max_yaw_jump_deg': math.degrees(max_yaw_jump),
        },
        'observation_quality': {
            'by_reason': dict(sorted(quality_reasons.items())),
            'calibration_reasons': dict(sorted(calibration_reasons.items())),
            'valid_camera_info_samples': valid_calibrations,
            'detection_to_recording_ms': _summary(detection_recording_age_ms),
            'tf_to_recording_ms': _summary(tf_recording_age_ms),
            'tf_to_detection_absolute_ms': _summary(tf_to_detection_delta_ms),
            'meaning': (
                'Recording-time deltas are observability signals, not camera, '
                'inference, control, or physical docking latency.'
            ),
        },
        'published_pose_quality': {
            'jump_count': len(pose_jumps),
            'jumps': pose_jumps,
        },
    }


def _storage_identifier(bag_path):
    metadata_path = bag_path / 'metadata.yaml'
    if not metadata_path.is_file():
        return 'sqlite3'
    match = re.search(
        r'^\s*storage_identifier:\s*([^\s#]+)',
        metadata_path.read_text(encoding='utf-8'),
        flags=re.MULTILINE,
    )
    return match.group(1) if match else 'sqlite3'


def read_rosbag(
    bag_path,
    detections_topic,
    tf_topic,
    camera_info_topic,
    pose_topic,
):
    """Decode only the four topics needed by the offline report."""
    try:
        from rclpy.serialization import deserialize_message
        from rosbag2_py import (
            ConverterOptions,
            SequentialReader,
            StorageOptions,
        )
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise RuntimeError(
            'rosbag2_py, rclpy, and rosidl_runtime_py are required to read a '
            'bag'
        ) from exc

    reader = SequentialReader()
    reader.open(
        StorageOptions(
            uri=str(bag_path),
            storage_id=_storage_identifier(bag_path),
        ),
        ConverterOptions('cdr', 'cdr'),
    )
    topic_types = {
        item.name: item.type
        for item in reader.get_all_topics_and_types()
    }
    selected_topics = {
        detections_topic,
        tf_topic,
        camera_info_topic,
        pose_topic,
    }
    message_types = {
        topic: get_message(topic_types[topic])
        for topic in selected_topics
        if topic in topic_types
    }

    detections = []
    transforms = []
    calibrations = []
    poses = []
    while reader.has_next():
        topic, serialized, recorded_ns = reader.read_next()
        if topic not in message_types:
            continue
        message = deserialize_message(serialized, message_types[topic])
        recorded_at = recorded_ns * 1e-9
        if topic == detections_topic:
            stamp = _message_seconds(message)
            detections.extend(
                DetectionSample(recorded_at, stamp, int(item.id))
                for item in message.detections
            )
        elif topic == tf_topic:
            transforms.extend(
                TransformSample(
                    transform.header.stamp.sec
                    + transform.header.stamp.nanosec * 1e-9,
                    transform.child_frame_id,
                )
                for transform in message.transforms
            )
        elif topic == camera_info_topic:
            calibrations.append(
                CalibrationSample(
                    recorded_at,
                    _message_seconds(message),
                    camera_info_to_calibration(message),
                )
            )
        elif topic == pose_topic:
            orientation = message.pose.orientation
            poses.append(
                PoseSample(
                    recorded_at,
                    _message_seconds(message),
                    (
                        float(message.pose.position.x),
                        float(message.pose.position.y),
                        float(message.pose.position.z),
                        yaw_from_quaternion(
                            orientation.x,
                            orientation.y,
                            orientation.z,
                            orientation.w,
                        ),
                    ),
                )
            )
    return detections, transforms, calibrations, poses


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--detections-topic', default='/apriltag/detections')
    parser.add_argument('--tf-topic', default='/tf')
    parser.add_argument('--camera-info-topic', default='/camera/camera_info')
    parser.add_argument('--pose-topic', default='/detected_dock_pose')
    parser.add_argument('--tag-frame-prefix', default='tag36h11')
    parser.add_argument('--max-detection-age', type=float, default=0.25)
    parser.add_argument('--max-tf-age', type=float, default=0.25)
    parser.add_argument('--max-future-skew', type=float, default=0.05)
    parser.add_argument('--max-translation-jump', type=float, default=0.25)
    parser.add_argument('--max-yaw-jump-deg', type=float, default=20.0)
    args = parser.parse_args(argv)

    detections, transforms, calibrations, poses = read_rosbag(
        args.bag,
        args.detections_topic,
        args.tf_topic,
        args.camera_info_topic,
        args.pose_topic,
    )
    report = summarize_observations(
        detections=detections,
        transforms=transforms,
        calibrations=calibrations,
        poses=poses,
        tag_frame_prefix=args.tag_frame_prefix,
        max_detection_age=args.max_detection_age,
        max_tf_age=args.max_tf_age,
        max_future_skew=args.max_future_skew,
        max_translation_jump=args.max_translation_jump,
        max_yaw_jump=math.radians(args.max_yaw_jump_deg),
    )
    report['source'] = {'bag_name': args.bag.name}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
