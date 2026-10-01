"""Validation and interpolation helpers for Piper joint trajectories."""

from bisect import bisect_right
from dataclasses import dataclass
import math


ARM_JOINTS = (
    'joint1',
    'joint2',
    'joint3',
    'joint4',
    'joint5',
    'joint6',
)
GRIPPER_JOINTS = ('joint7',)
COMMAND_JOINTS = ARM_JOINTS + GRIPPER_JOINTS
# Complete joint1-joint7 target used by /initialize_arm.
READY_POSITIONS = (
    -1.557051440,
    0.234953236,
    -0.324353736,
    0.013745872,
    0.203885472,
    -0.018106872,
    0.0020,
)

#限制角度
JOINT_LIMITS = {
    'joint1': (-2.618, 2.168),
    'joint2': (0.0, 3.14),
    'joint3': (-2.967, 0.0),
    'joint4': (-1.745, 1.745),
    'joint5': (-1.22, 1.22),
    'joint6': (-2.0944, 2.0944),
    'joint7': (0.0, 0.035),
}

def startup_state_error(arm_positions, boundary_recovery_tolerance):
    """Return why a measured startup state cannot be safely normalized."""
    positions = tuple(float(value) for value in arm_positions)
    if len(positions) != len(ARM_JOINTS):
        return 'startup state must contain all six arm joints'
    if not all(math.isfinite(value) for value in positions):
        return 'startup state contains a non-finite joint position'

    tolerance = float(boundary_recovery_tolerance)
    if not math.isfinite(tolerance) or tolerance < 0.0:
        return 'boundary recovery tolerance must be finite and non-negative'

    for joint_name, value in zip(ARM_JOINTS, positions):
        lower, upper = JOINT_LIMITS[joint_name]
        allowed_lower = (
            lower - tolerance if joint_name == 'joint2' else lower
        )
        allowed_upper = (
            upper + tolerance if joint_name == 'joint3' else upper
        )
        if value < allowed_lower or value > allowed_upper:
            return (
                f'{joint_name} measured {value:.6f} is outside the '
                f'startup recovery range '
                f'[{allowed_lower:.6f}, {allowed_upper:.6f}]'
            )
    return None


class TrajectoryValidationError(ValueError):
    """Raised when a FollowJointTrajectory goal is unsafe or malformed."""


@dataclass(frozen=True)
class NormalizedTrajectory:
    """Trajectory points reordered into the controller's joint order."""

    joint_names: tuple
    times: tuple
    positions: tuple
    velocities: tuple = ()


def duration_seconds(duration):
    """Convert a ROS-like duration object to seconds."""
    return float(duration.sec) + float(duration.nanosec) * 1e-9


def normalize_trajectory(trajectory, expected_joints):
    """Validate and reorder a ROS JointTrajectory without mutating it."""
    expected = tuple(expected_joints)
    supplied = tuple(trajectory.joint_names)
    if len(supplied) != len(set(supplied)):
        raise TrajectoryValidationError('trajectory contains duplicate joints')
    if set(supplied) != set(expected):
        raise TrajectoryValidationError(
            f'expected joints {expected}, received {supplied}'
        )
    if not trajectory.points:
        raise TrajectoryValidationError('trajectory contains no points')

    source_indices = tuple(supplied.index(name) for name in expected)
    times = []
    positions = []
    velocities = []
    previous_time = -math.inf

    for point_index, point in enumerate(trajectory.points):
        if len(point.positions) != len(supplied):
            raise TrajectoryValidationError(
                f'point {point_index} has an invalid position count'
            )
        for values, field_name in (
            (point.velocities, 'velocity'),
            (point.accelerations, 'acceleration'),
            (point.effort, 'effort'),
        ):
            if values and len(values) != len(supplied):
                raise TrajectoryValidationError(
                    f'point {point_index} has an invalid {field_name} count'
                )

        point_time = duration_seconds(point.time_from_start)
        if point_time < 0.0 or point_time <= previous_time:
            raise TrajectoryValidationError(
                'trajectory times must be non-negative and strictly increasing'
            )

        reordered = tuple(
            float(point.positions[index])
            for index in source_indices
        )
        if not all(math.isfinite(value) for value in reordered):
            raise TrajectoryValidationError(
                f'point {point_index} contains a non-finite position'
            )

        for joint_name, value in zip(expected, reordered):
            lower, upper = JOINT_LIMITS[joint_name]
            if value < lower or value > upper:
                raise TrajectoryValidationError(
                    f'{joint_name} target {value:.6f} is outside '
                    f'[{lower:.6f}, {upper:.6f}]'
                )

        times.append(point_time)
        positions.append(reordered)
        velocity = tuple(float(point.velocities[index]) for index in source_indices) if point.velocities else ()
        if not all(math.isfinite(value) for value in velocity):
            raise TrajectoryValidationError('trajectory contains non-finite velocity')
        velocities.append(velocity)
        previous_time = point_time

    return NormalizedTrajectory(
        joint_names=expected,
        times=tuple(times),
        positions=tuple(positions),
        velocities=tuple(velocities) if all(velocities) else (),
    )


def sample_linear_trajectory(trajectory, start_positions, elapsed):
    """Sample a normalized trajectory with bounded linear interpolation."""
    start = tuple(float(value) for value in start_positions)
    if len(start) != len(trajectory.joint_names):
        raise ValueError('start position count does not match trajectory')

    sample_time = max(0.0, float(elapsed))
    times = trajectory.times
    positions = trajectory.positions

    if sample_time >= times[-1]:
        return positions[-1], (0.0,) * len(start)

    right_index = bisect_right(times, sample_time)
    if right_index == 0:
        left_time = 0.0
        left_positions = start
        right_time = times[0]
        right_positions = positions[0]
    else:
        left_time = times[right_index - 1]
        left_positions = positions[right_index - 1]
        right_time = times[right_index]
        right_positions = positions[right_index]

    segment_duration = right_time - left_time
    if segment_duration <= 0.0:
        return right_positions, (0.0,) * len(start)

    ratio = (sample_time - left_time) / segment_duration
    ratio = min(max(ratio, 0.0), 1.0)
    sampled = tuple(
        left + ratio * (right - left)
        for left, right in zip(left_positions, right_positions)
    )
    velocities = tuple(
        (right - left) / segment_duration
        for left, right in zip(left_positions, right_positions)
    )
    return sampled, velocities



def prepare_smooth_trajectory(trajectory, start_positions):
    """Bound shared cubic tangents to local secants; no point overshoot."""
    times = list(trajectory.times)
    positions = list(trajectory.positions)
    velocities = list(trajectory.velocities)
    if times[0] > 0:
        times.insert(0, 0.0)
        positions.insert(0, tuple(start_positions))
        if velocities:
            velocities.insert(0, (0.0,) * len(start_positions))
    slopes = [tuple((b-a)/(times[i+1]-times[i]) for a,b in zip(positions[i],positions[i+1])) for i in range(len(times)-1)]
    tangents=[]
    for i in range(len(times)):
        values=[]
        for j in range(len(trajectory.joint_names)):
            if i == 0 or i == len(times)-1:
                value=0.0
            else:
                left,right=slopes[i-1][j],slopes[i][j]
                requested=velocities[i][j] if velocities else (left+right)/2
                if left*right <= 0 or requested*left <= 0:
                    value=0.0
                else:
                    value=math.copysign(min(abs(requested),abs(left),abs(right)),left)
            values.append(value)
        tangents.append(tuple(values))
    return NormalizedTrajectory(trajectory.joint_names,tuple(times),tuple(positions),tuple(tangents))


def sample_smooth_trajectory(trajectory, start_positions, elapsed):
    """C1 cubic interpolation with tangents bounded during preparation."""
    t=max(0.0,float(elapsed))
    if t >= trajectory.times[-1]:
        return trajectory.positions[-1],(0.0,)*len(trajectory.joint_names)
    i=max(0,bisect_right(trajectory.times,t)-1)
    h=trajectory.times[i+1]-trajectory.times[i]
    u=(t-trajectory.times[i])/h
    p0,p1=trajectory.positions[i:i+2]
    v0,v1=trajectory.velocities[i:i+2]
    pos=tuple((2*u**3-3*u**2+1)*a+(u**3-2*u**2+u)*h*va+(-2*u**3+3*u**2)*b+(u**3-u**2)*h*vb for a,b,va,vb in zip(p0,p1,v0,v1))
    vel=tuple((6*u*u-6*u)*a/h+(3*u*u-4*u+1)*va+(-6*u*u+6*u)*b/h+(3*u*u-2*u)*vb for a,b,va,vb in zip(p0,p1,v0,v1))
    return pos,vel

def position_tolerances(requested, joint_names, default_tolerance):
    """Return safe positive per-joint tolerances."""
    tolerances = {
        joint_name: float(default_tolerance)
        for joint_name in joint_names
    }
    for tolerance in requested:
        if tolerance.name not in tolerances:
            raise TrajectoryValidationError(
                f'unknown tolerance joint {tolerance.name!r}'
            )
        if tolerance.position > 0.0:
            tolerances[tolerance.name] = float(tolerance.position)
    return tuple(tolerances[name] for name in joint_names)


def violating_joint(joint_names, errors, tolerances):
    """Return the first joint exceeding its absolute position tolerance."""
    for joint_name, error, tolerance in zip(
        joint_names,
        errors,
        tolerances,
    ):
        if abs(error) > tolerance:
            return joint_name
    return None


def updated_settle_count(
    joint_names,
    errors,
    tolerances,
    current_count,
):
    """Count consecutive in-tolerance feedback cycles, resetting on error."""
    count = int(current_count)
    if count < 0:
        raise ValueError('current settle count cannot be negative')
    if violating_joint(joint_names, errors, tolerances) is not None:
        return 0
    return count + 1


class PiperCanFeedbackState:
    """Track fresh joint and enable frames observed directly on SocketCAN."""

    ARM_POSITION_CAN_IDS = (0x2A5, 0x2A6, 0x2A7)
    GRIPPER_POSITION_CAN_ID = 0x2A8
    LOW_SPEED_CAN_IDS = tuple(range(0x261, 0x267))

    def __init__(self):
        self.received_at = {}
        self.enabled = {}

    def observe(self, can_id, data, received_at):
        """Record one Piper CAN feedback frame."""
        if can_id in (
            *self.ARM_POSITION_CAN_IDS,
            self.GRIPPER_POSITION_CAN_ID,
        ):
            self.received_at[can_id] = float(received_at)
        elif can_id in self.LOW_SPEED_CAN_IDS and len(data) >= 6:
            self.received_at[can_id] = float(received_at)
            self.enabled[can_id] = bool(data[5] & 0x40)

    def arm_ready(self, now, timeout):
        """Return whether position and enabled feedback are all fresh."""
        required = self.ARM_POSITION_CAN_IDS + self.LOW_SPEED_CAN_IDS
        return (
            all(
                now - self.received_at.get(can_id, -math.inf) <= timeout
                for can_id in required
            )
            and all(
                self.enabled.get(can_id, False)
                for can_id in self.LOW_SPEED_CAN_IDS
            )
        )

    def gripper_ready(self, now, timeout):
        """Return whether arm enable and gripper feedback are fresh."""
        return (
            self.arm_ready(now, timeout)
            and now - self.received_at.get(
                self.GRIPPER_POSITION_CAN_ID,
                -math.inf,
            ) <= timeout
        )
