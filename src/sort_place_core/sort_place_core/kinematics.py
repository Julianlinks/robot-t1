"""SCARA R-R-P-R kinematics. Distances: m; angles: rad; times: s."""
import math
from dataclasses import dataclass

JOINTS = ('joint1', 'joint2', 'joint3', 'joint4')
LIMITS = ((-2.6, 2.6), (0.15, 2.85), (0.0, 0.28), (-math.pi, math.pi))
VELOCITIES = (0.7, 0.7, 0.10, 1.0)
ACCELERATIONS = (1.4, 1.4, 0.25, 2.0)
L1, L2, TCP_ZERO_Z = 0.32, 0.28, 0.96


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    z: float
    yaw: float = 0.0


def angle_error(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def validate_joints(q):
    if len(q) != 4 or not all(math.isfinite(v) for v in q):
        raise ValueError('Expected four finite joint positions')
    for name, value, (low, high) in zip(JOINTS, q, LIMITS):
        if not low - 1e-9 <= value <= high + 1e-9:
            raise ValueError(f'{name}={value:.5f} outside [{low}, {high}]')
    return tuple(q)


def fk(q):
    a, b, slide, wrist = validate_joints(q)
    return Pose(L1 * math.cos(a) + L2 * math.cos(a + b),
                L1 * math.sin(a) + L2 * math.sin(a + b),
                TCP_ZERO_Z - slide, angle_error(a + b + wrist, 0.0))


def ik(pose):
    if not all(math.isfinite(v) for v in (pose.x, pose.y, pose.z, pose.yaw)):
        raise ValueError('Non-finite target pose')
    c = (pose.x**2 + pose.y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    if abs(c) > 1.0 + 1e-9:
        raise ValueError('Target outside SCARA reach')
    b = math.acos(max(-1.0, min(1.0, c)))  # fixed elbow branch
    a = math.atan2(pose.y, pose.x) - math.atan2(L2 * math.sin(b), L1 + L2 * math.cos(b))
    wrist = angle_error(pose.yaw - a - b, 0.0)
    return validate_joints((a, b, TCP_ZERO_Z - pose.z, wrist))


def trajectory(start, end, minimum_duration=0.5, step=0.02):
    """Quintic interpolation, bounded analytically in velocity and acceleration."""
    start, end = validate_joints(start), validate_joints(end)
    if minimum_duration <= 0 or step <= 0:
        raise ValueError('Duration and step must be positive')
    delta = [b - a for a, b in zip(start, end)]
    duration = max(minimum_duration,
                   *(1.875 * abs(d) / v for d, v in zip(delta, VELOCITIES)),
                   *(math.sqrt(5.774 * abs(d) / a) for d, a in zip(delta, ACCELERATIONS)))
    count = max(2, math.ceil(duration / step))
    points = []
    for i in range(1, count + 1):
        t = duration * i / count
        s = t / duration
        h = 10*s**3 - 15*s**4 + 6*s**5
        dh = (30*s**2 - 60*s**3 + 30*s**4) / duration
        ddh = (60*s - 180*s**2 + 120*s**3) / duration**2
        points.append((t, tuple(a + d*h for a, d in zip(start, delta)),
                       tuple(d*dh for d in delta), tuple(d*ddh for d in delta)))
    return points
