"""Pure validation helpers shared by the live bridge and bag analysis."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class TemporalQuality:
    accepted: bool
    reason: str
    detection_age_ms: float | None
    tf_age_ms: float | None


@dataclass(frozen=True)
class CameraCalibration:
    valid: bool
    reason: str
    width: int | None
    height: int | None
    fx: float | None
    fy: float | None


def _finite_nonnegative(value):
    return isinstance(value, (int, float)) and math.isfinite(value) and value >= 0.0


def validate_observation_timing(
    *,
    now,
    detection_stamp,
    tf_stamp,
    max_detection_age,
    max_tf_age,
    max_future_skew,
):
    """Fail closed when a detection or its transform is stale or future dated."""
    values = (now, detection_stamp, tf_stamp)
    limits = (max_detection_age, max_tf_age, max_future_skew)
    if not all(_finite_nonnegative(value) for value in values):
        return TemporalQuality(False, 'INVALID_TIMESTAMP', None, None)
    if not all(_finite_nonnegative(value) for value in limits):
        raise ValueError('temporal limits must be finite and non-negative')

    detection_age = (now - detection_stamp) * 1000.0
    tf_age = (now - tf_stamp) * 1000.0
    if detection_age < -max_future_skew * 1000.0:
        return TemporalQuality(False, 'DETECTION_FUTURE', detection_age, tf_age)
    if tf_age < -max_future_skew * 1000.0:
        return TemporalQuality(False, 'TF_FUTURE', detection_age, tf_age)
    if detection_age > max_detection_age * 1000.0:
        return TemporalQuality(False, 'DETECTION_STALE', detection_age, tf_age)
    if tf_age > max_tf_age * 1000.0:
        return TemporalQuality(False, 'TF_STALE', detection_age, tf_age)
    return TemporalQuality(True, 'ACCEPTED', detection_age, tf_age)


def validate_camera_calibration(width, height, matrix, distortion_model):
    """Validate the subset of CameraInfo needed for metric pose estimation."""
    if not isinstance(width, int) or not isinstance(height, int):
        return CameraCalibration(False, 'IMAGE_SIZE_INVALID', None, None, None, None)
    if width <= 0 or height <= 0:
        return CameraCalibration(False, 'IMAGE_SIZE_INVALID', width, height, None, None)
    if not isinstance(distortion_model, str) or not distortion_model.strip():
        return CameraCalibration(
            False,
            'DISTORTION_MODEL_MISSING',
            width,
            height,
            None,
            None,
        )
    try:
        matrix = tuple(matrix)
    except TypeError:
        return CameraCalibration(
            False,
            'INTRINSICS_INVALID',
            width,
            height,
            None,
            None,
        )
    if len(matrix) != 9 or not all(
        isinstance(value, (int, float)) and math.isfinite(value) for value in matrix
    ):
        return CameraCalibration(False, 'INTRINSICS_INVALID', width, height, None, None)

    fx = float(matrix[0])
    fy = float(matrix[4])
    cx = float(matrix[2])
    cy = float(matrix[5])
    if fx <= 0.0 or fy <= 0.0:
        return CameraCalibration(False, 'FOCAL_LENGTH_INVALID', width, height, fx, fy)
    if not 0.0 <= cx <= width or not 0.0 <= cy <= height:
        return CameraCalibration(
            False,
            'PRINCIPAL_POINT_INVALID',
            width,
            height,
            fx,
            fy,
        )
    return CameraCalibration(True, 'ACCEPTED', width, height, fx, fy)


def camera_info_to_calibration(message):
    return validate_camera_calibration(
        int(message.width),
        int(message.height),
        tuple(message.k),
        str(message.distortion_model),
    )


def yaw_from_quaternion(x, y, z, w):
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


def pose_delta(previous, current):
    """Return translation and wrapped yaw deltas for two (x, y, z, yaw) poses."""
    translation = math.sqrt(
        (current[0] - previous[0]) ** 2
        + (current[1] - previous[1]) ** 2
        + (current[2] - previous[2]) ** 2
    )
    yaw = (current[3] - previous[3] + math.pi) % (2.0 * math.pi) - math.pi
    return translation, abs(yaw)
