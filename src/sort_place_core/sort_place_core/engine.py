"""Backend-independent sequence. No automatic release when a motion fails."""
import math
from dataclasses import dataclass
from .kinematics import Pose, ik, fk, trajectory, angle_error


class CellError(RuntimeError):
    pass


class Cancelled(CellError):
    pass


@dataclass(frozen=True)
class Part:
    object_id: str
    class_id: str
    pose: Pose
    confidence: float = 1.0
    attached: bool = False


@dataclass(frozen=True)
class Slot:
    slot_id: str
    class_id: str
    pose: Pose  # desired PART CENTER, not TCP


class Engine:
    def __init__(self, config, backend, feedback=lambda *args: None):
        self.config, self.backend, self.feedback = config, backend, feedback
        self.slots = [Slot(s['id'], s['class_id'], Pose(*s['pose'])) for s in config['slots']]
        if len({s.slot_id for s in self.slots}) != len(self.slots):
            raise ValueError('Duplicate slot IDs')
        self.occupied, self.quarantined, self.completed = set(), set(), set()
        self.held = None
        for slot in self.slots:
            ik(self.tcp(slot.pose))
        if config['safe_z'] <= config['source_z_max'] + 0.05:
            raise ValueError('Insufficient travel clearance')

    def tcp(self, p):
        return Pose(p.x, p.y, p.z + self.config['tcp_part_offset'], p.yaw)

    def check(self, cancelled):
        if cancelled():
            raise Cancelled('Task cancelled; held part remains attached')

    def move(self, pose, stage, part, count, cancelled):
        self.check(cancelled)
        self.feedback(stage, part.object_id, count)
        self.backend.move(ik(pose), cancelled)

    def source_candidate(self, part):
        c = self.config
        return (part.object_id not in self.completed and not part.attached
                and math.isfinite(part.confidence) and part.confidence >= c['minimum_confidence']
                and c['source_x'][0] <= part.pose.x <= c['source_x'][1]
                and c['source_y'][0] <= part.pose.y <= c['source_y'][1]
                and c['source_z_min'] <= part.pose.z <= c['source_z_max'])

    def reconcile(self, parts):
        """Rebuild occupied slots from measured poses, including after node restart."""
        for s in self.slots:
            for p in parts:
                if (not p.attached and math.dist((p.pose.x, p.pose.y, p.pose.z),
                        (s.pose.x, s.pose.y, s.pose.z)) < self.config['slot_exclusion_radius']):
                    self.occupied.add(s.slot_id)

    def run(self, max_parts=0, cancelled=lambda: False):
        count = 0
        parts = self.backend.parts()
        self.reconcile(parts)
        if self.held or any(p.attached for p in parts):
            raise CellError('A part is already attached; recover the cell before a new batch')
        candidates = sorted((p for p in parts if self.source_candidate(p)), key=lambda p: p.object_id)
        for original in candidates:
            if max_parts and count >= max_parts:
                break
            self.check(cancelled)
            fresh = {p.object_id: p for p in self.backend.parts()}
            part = fresh.get(original.object_id)
            if part is None or not self.source_candidate(part):
                raise CellError('Source part disappeared or moved out of the source area')
            slot = next((s for s in self.slots if s.class_id == part.class_id
                         and s.slot_id not in self.occupied | self.quarantined), None)
            if slot is None:
                raise CellError(f'No free slot for class {part.class_id}')
            current = fk(self.backend.joints())
            safe_z = self.config['safe_z']
            if current.z < self.config['minimum_tcp_z']:
                raise CellError('TCP starts below the permitted cell height')
            pick, place = self.tcp(part.pose), self.tcp(slot.pose)
            sequence = [Pose(current.x, current.y, safe_z, current.yaw),
                        Pose(pick.x, pick.y, safe_z, pick.yaw), pick,
                        Pose(pick.x, pick.y, safe_z, pick.yaw),
                        Pose(place.x, place.y, safe_z, place.yaw), place]
            # Validate all endpoints and interpolated links' TCP heights before pickup.
            q = self.backend.joints()
            for pose in sequence:
                target = ik(pose)
                for _, sample, _, _ in trajectory(q, target):
                    if fk(sample).z < self.config['minimum_tcp_z']:
                        raise CellError('Path violates TCP/table clearance')
                q = target
            picked = False
            try:
                for pose, stage in zip(sequence[:3], ('RAISE', 'APPROACH_PICK', 'DESCEND_PICK')):
                    self.move(pose, stage, part, count, cancelled)
                # Reserve BEFORE service: a lost response may still mean attachment occurred.
                self.quarantined.add(slot.slot_id)
                picked = True
                self.feedback('SUCTION_ON', part.object_id, count)
                self.check(cancelled)
                self.backend.suction(part.object_id, True)
                self.held = part.object_id
                for pose, stage in zip(sequence[3:], ('LIFT', 'TRANSFER', 'DESCEND_PLACE')):
                    self.move(pose, stage, part, count, cancelled)
                    self.backend.dwell(0.12, cancelled)
                    observed = next((p for p in self.backend.parts()
                                     if p.object_id == part.object_id), None)
                    expected = Pose(pose.x, pose.y, pose.z - self.config['tcp_part_offset'], pose.yaw)
                    if (observed is None or not observed.attached or
                            math.dist((observed.pose.x, observed.pose.y, observed.pose.z),
                                      (expected.x, expected.y, expected.z)) > 0.012):
                        raise CellError('Held part did not follow the TCP; vacuum constraint may have failed')
                self.backend.dwell(self.config['press_dwell'], cancelled)
                self.check(cancelled)
                self.feedback('SUCTION_OFF', part.object_id, count)
                self.backend.suction(part.object_id, False)
                self.held = None
                self.move(Pose(place.x, place.y, safe_z, place.yaw),
                          'RETRACT', part, count, cancelled)
                self.feedback('VERIFY', part.object_id, count)
                self.backend.dwell(self.config['settle_time'], cancelled)
                measured = next((p for p in self.backend.parts() if p.object_id == part.object_id), None)
                if measured is None or measured.attached:
                    raise CellError('No detached-part feedback after release')
                error = math.dist((measured.pose.x, measured.pose.y, measured.pose.z),
                                  (slot.pose.x, slot.pose.y, slot.pose.z))
                if (error > self.config['position_tolerance'] or
                        abs(angle_error(measured.pose.yaw, slot.pose.yaw)) > self.config['yaw_tolerance']):
                    raise CellError(f'Placement verification failed: position error={error:.5f} m')
                self.occupied.add(slot.slot_id)
                self.quarantined.discard(slot.slot_id)
                self.completed.add(part.object_id)
                count += 1
                self.feedback('PLACED', part.object_id, count)
            except Exception:
                if not picked:
                    self.quarantined.discard(slot.slot_id)
                # Any uncertain pickup/place keeps the destination unavailable.
                raise
        self.check(cancelled)
        self.feedback('COMPLETE', '', count)
        return count
