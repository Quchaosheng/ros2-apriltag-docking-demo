from dataclasses import dataclass
import math


@dataclass(frozen=True)
class DockSpec:
    tag_id: int
    dock_id: str
    dock_type: str
    tag_frame: str


@dataclass(frozen=True)
class Detection:
    tag_id: int
    hamming: int
    decision_margin: float
    stamp: float
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class GateResult:
    accepted: bool
    reason: str
    detection: Detection | None = None
    dock: DockSpec | None = None


def load_dock_specs(raw):
    docks = raw.get('docks') if isinstance(raw, dict) else None
    if not isinstance(docks, dict) or not docks:
        raise ValueError('docks must be a non-empty mapping')

    specs = {}
    dock_ids = set()
    for raw_tag_id, value in docks.items():
        try:
            tag_id = int(raw_tag_id)
        except (TypeError, ValueError) as exc:
            raise ValueError(f'invalid tag id: {raw_tag_id!r}') from exc

        if tag_id < 0 or tag_id in specs or not isinstance(value, dict):
            raise ValueError(f'invalid or duplicate tag id: {raw_tag_id!r}')

        fields = [value.get(name) for name in ('dock_id', 'dock_type', 'tag_frame')]
        if any(not isinstance(field, str) or not field.strip() for field in fields):
            raise ValueError(f'tag {tag_id} has missing or empty fields')

        dock_id, dock_type, tag_frame = fields
        if dock_id in dock_ids:
            raise ValueError(f'duplicate dock id: {dock_id}')

        specs[tag_id] = DockSpec(tag_id, dock_id, dock_type, tag_frame)
        dock_ids.add(dock_id)

    return specs


class TagGate:
    def __init__(
        self,
        *,
        specs,
        min_margin,
        max_hamming,
        confirmations,
        confirmation_window,
        publish_period,
        loss_timeout,
        max_translation_jump,
        max_yaw_jump,
    ):
        self.specs = specs
        self.min_margin = min_margin
        self.max_hamming = max_hamming
        self.confirmations = confirmations
        self.confirmation_window = confirmation_window
        self.publish_period = publish_period
        self.loss_timeout = loss_timeout
        self.max_translation_jump = max_translation_jump
        self.max_yaw_jump = max_yaw_jump

        self._active_tag_id = None
        self._confirmation_count = 0
        self._confirmation_started = None
        self._confirmed = False
        self._last_accepted = None
        self._last_publication = None
        self._last_seen = None

    def evaluate(self, detections, now):
        if not detections:
            return self._reject('NO_TAG')
        if len(detections) != 1:
            return self._reject('MULTI_TAG')

        detection = detections[0]
        dock = self.specs.get(detection.tag_id)
        if dock is None:
            return self._reject('UNKNOWN_TAG')
        if detection.hamming > self.max_hamming:
            return self._reject('HAMMING')
        if detection.decision_margin < self.min_margin:
            return self._reject('LOW_MARGIN')

        if (
            self._last_seen is not None
            and now - self._last_seen > self.loss_timeout
        ):
            self._last_accepted = None
            self._reset_confirmation()
        self._last_seen = now
        if self._is_pose_jump(detection):
            return self._reject('POSE_JUMP')

        if not self._confirmed:
            expired = (
                self._confirmation_started is None
                or now - self._confirmation_started > self.confirmation_window
            )
            if self._active_tag_id != detection.tag_id or expired:
                self._active_tag_id = detection.tag_id
                self._confirmation_count = 1
                self._confirmation_started = now
            else:
                self._confirmation_count += 1

            if self._confirmation_count < self.confirmations:
                return GateResult(False, 'CONFIRMING', detection, dock)
            self._confirmed = True

        if (
            self._last_publication is not None
            and now - self._last_publication < self.publish_period
        ):
            return GateResult(False, 'RATE_LIMITED', detection, dock)

        self._last_accepted = detection
        self._last_publication = now
        return GateResult(True, 'ACCEPTED', detection, dock)

    def loss_reason(self, now):
        if self._last_seen is None or now - self._last_seen <= self.loss_timeout:
            return None
        self._last_seen = None
        self._last_accepted = None
        self._reset_confirmation()
        return 'TAG_LOST'

    def _reject(self, reason):
        self._reset_confirmation()
        return GateResult(False, reason)

    def _reset_confirmation(self):
        self._active_tag_id = None
        self._confirmation_count = 0
        self._confirmation_started = None
        self._confirmed = False

    def _is_pose_jump(self, detection):
        previous = self._last_accepted
        if previous is None or previous.tag_id != detection.tag_id:
            return False

        translation = math.hypot(detection.x - previous.x, detection.y - previous.y)
        yaw_delta = (detection.yaw - previous.yaw + math.pi) % (2.0 * math.pi) - math.pi
        return (
            translation > self.max_translation_jump
            or abs(yaw_delta) > self.max_yaw_jump
        )
