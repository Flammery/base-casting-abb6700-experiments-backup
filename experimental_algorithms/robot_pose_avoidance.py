from __future__ import annotations

"""Small experimental TCP-roll library for robot-arm collision trials.

The contact point and TCP +Z direction stay unchanged.  Only a constant local
TCP-Z roll is applied to a complete region/patch path.  Candidate paths are
screened with the reusable ``src`` IK, robot FK segments, and workpiece
collision mesh.  Tool geometry is intentionally excluded during this phase.
"""

import math
from dataclasses import dataclass, replace

from robot_studio_qt.kinematics.model import JointState, RobotConfiguration, WorkpiecePlacement
from robot_studio_qt.kinematics.orientation import Quaternion
from robot_studio_qt.kinematics.robot_models import SerialRobotModel
from robot_studio_qt.kinematics.solvers import SerialKinematicsSolver, serial_configuration_from_joint_values
from robot_studio_qt.kinematics.targets import CartesianTarget, Pose
from robot_studio_qt.path_planning.models import PathResult, Waypoint
from robot_studio_qt.polishing_tool import PolishingToolData
from robot_studio_qt.tools.reachability.collision import CollisionMesh, CollisionSettings


POSE_ROLL_DEGREES = (0.0, 15.0, -15.0, 30.0, -30.0)
EXPERIMENT_LINK_RADIUS_MM = 5.0
EXPERIMENT_CLEARANCE_MM = 0.0
EXPERIMENT_USE_SEGMENT_RADIUS = False
DEFAULT_SAMPLE_LIMIT = 7
DEFAULT_MAX_JOINT_JUMP_DEGREES = 40.0
DEFAULT_MIN_ABS_J5_DEGREES = 6.0


@dataclass(frozen=True)
class PoseTrial:
    name: str
    roll_degrees: float
    accepted: bool
    sampled_waypoints: int
    ik_failures: int
    collision_count: int
    collision_links: tuple[str, ...]
    configuration_count: int
    max_joint_jump_degrees: float
    min_abs_j5_degrees: float | None
    message: str

    def as_dict(self) -> dict:
        return {
            "pose_name": self.name,
            "roll_degrees": self.roll_degrees,
            "accepted": self.accepted,
            "sampled_waypoints": self.sampled_waypoints,
            "ik_failures": self.ik_failures,
            "collision_count": self.collision_count,
            "collision_links": ",".join(self.collision_links),
            "configuration_count": self.configuration_count,
            "max_joint_jump_degrees": self.max_joint_jump_degrees,
            "min_abs_j5_degrees": self.min_abs_j5_degrees,
            "message": self.message,
        }


@dataclass(frozen=True)
class PoseSelection:
    path: PathResult
    status: str
    selected_name: str
    selected_roll_degrees: float
    trials: tuple[PoseTrial, ...]

    @property
    def validated(self) -> bool:
        return self.status in {"baseline-validated", "alternative-validated"}


def apply_local_tcp_z_roll(path: PathResult, roll_degrees: float) -> PathResult:
    """Rotate TCP X/Y around local TCP Z without changing position or tool axis."""

    half_angle = math.radians(roll_degrees) * 0.5
    local_roll = Quaternion(math.cos(half_angle), 0.0, 0.0, math.sin(half_angle))
    previous: Quaternion | None = None
    waypoints: list[Waypoint] = []
    for waypoint in path.waypoints:
        quaternion = waypoint.quaternion.multiplied(local_roll).normalized()
        if previous is not None and quaternion.dot(previous) < 0.0:
            quaternion = quaternion.negated()
        previous = quaternion
        waypoints.append(replace(waypoint, quaternion=quaternion))
    return replace(path, waypoints=waypoints, message=f"{path.message} TCP local-Z roll={roll_degrees:g} deg.")


def select_robot_pose(
    path: PathResult,
    polydata,
    placement: WorkpiecePlacement,
    robot_config: RobotConfiguration,
    initial_state: JointState,
    polishing_tool: PolishingToolData | None,
    *,
    sample_limit: int = DEFAULT_SAMPLE_LIMIT,
    collision_settings: CollisionSettings | None = None,
    collision_mesh: CollisionMesh | None = None,
) -> PoseSelection:
    """Return the first sampled-validated library pose, or unchanged baseline.

    ``fallback-unverified`` deliberately keeps the former orientation when the
    internal numerical model cannot validate any candidate.  This avoids
    silently replacing a path on the strength of an incomplete experiment; the
    generated diagnostics are then used for RobotStudio follow-up.
    """

    # The final MDH segment is part of the robot wrist, not the separately
    # modelled polishing tool, so robot-only analysis must include it.
    settings = collision_settings or CollisionSettings(
        link_radius=EXPERIMENT_LINK_RADIUS_MM,
        clearance=EXPERIMENT_CLEARANCE_MM,
        ignore_tcp_segment=False,
        use_segment_radius=EXPERIMENT_USE_SEGMENT_RADIUS,
    )
    collision_mesh = collision_mesh or CollisionMesh.from_polydata(polydata, placement, settings.max_triangles)
    trials: list[PoseTrial] = []
    candidate_paths: list[PathResult] = []
    for roll_degrees in POSE_ROLL_DEGREES:
        candidate = apply_local_tcp_z_roll(path, roll_degrees)
        candidate_paths.append(candidate)
        trials.append(
            evaluate_robot_pose(
                candidate,
                robot_config,
                initial_state,
                polishing_tool,
                collision_mesh,
                settings,
                roll_degrees,
                sample_limit=sample_limit,
            )
        )

    accepted_index = next((index for index, trial in enumerate(trials) if trial.accepted), None)
    if accepted_index is None:
        return PoseSelection(path, "fallback-unverified", trials[0].name, 0.0, tuple(trials))
    selected_trial = trials[accepted_index]
    status = "baseline-validated" if accepted_index == 0 else "alternative-validated"
    return PoseSelection(
        candidate_paths[accepted_index],
        status,
        selected_trial.name,
        selected_trial.roll_degrees,
        tuple(trials),
    )


def evaluate_robot_pose(
    path: PathResult,
    robot_config: RobotConfiguration,
    initial_state: JointState,
    polishing_tool: PolishingToolData | None,
    collision_mesh: CollisionMesh,
    collision_settings: CollisionSettings,
    roll_degrees: float,
    *,
    sample_limit: int = DEFAULT_SAMPLE_LIMIT,
    max_joint_jump_degrees: float = DEFAULT_MAX_JOINT_JUMP_DEGREES,
    min_abs_j5_degrees: float = DEFAULT_MIN_ABS_J5_DEGREES,
) -> PoseTrial:
    sampled = sample_waypoints(path.waypoints, sample_limit)
    free_solver = SerialKinematicsSolver(max_iterations=90, lock_configuration_to_seed=False)
    locked_solver = SerialKinematicsSolver(max_iterations=90, lock_configuration_to_seed=True)
    seed = initial_state.as_list()
    previous: list[float] | None = None
    ik_failures = 0
    collision_count = 0
    collision_links: set[str] = set()
    max_jump = 0.0
    min_j5_abs = math.inf
    configurations: set[tuple[int, int, int, int]] = set()
    first_failure = ""

    for index, waypoint in enumerate(sampled):
        solver = free_solver if index == 0 else locked_solver
        result = solver.inverse(
            SerialRobotModel(JointState(seed), robot_config, polishing_tool),
            target_for_waypoint(waypoint),
            seed,
        )
        if not result.success or result.joint_target is None:
            ik_failures += 1
            first_failure = first_failure or result.message
            continue
        values = result.joint_target.values
        configurations.add(serial_configuration_from_joint_values(values))
        if previous is not None:
            max_jump = max(max_jump, max(abs(current - prior) for current, prior in zip(values, previous)))
        if len(values) > 4:
            min_j5_abs = min(min_j5_abs, abs(values[4]))
        robot = SerialRobotModel(JointState(values), robot_config, polishing_tool)
        waypoint_collision_links = collision_mesh.colliding_segment_names(robot.forward().pose.segments, collision_settings)
        if waypoint_collision_links:
            collision_count += 1
            collision_links.update(waypoint_collision_links)
        seed = values
        previous = values

    min_j5 = None if min_j5_abs == math.inf else min_j5_abs
    posture_ok = max_jump <= max_joint_jump_degrees and (min_j5 is None or min_j5 >= min_abs_j5_degrees)
    accepted = bool(sampled) and ik_failures == 0 and collision_count == 0 and len(configurations) <= 1 and posture_ok
    problems: list[str] = []
    if ik_failures:
        problems.append(f"IK失败 {ik_failures}/{len(sampled)}")
    if collision_count:
        problems.append(f"机械臂碰撞 {collision_count}/{len(sampled)} ({','.join(sorted(collision_links))})")
    if len(configurations) > 1:
        problems.append(f"构型数 {len(configurations)}")
    if max_jump > max_joint_jump_degrees:
        problems.append(f"最大关节跳变 {max_jump:.1f}°")
    if min_j5 is not None and min_j5 < min_abs_j5_degrees:
        problems.append(f"|J5| 最小 {min_j5:.1f}°")
    if first_failure:
        problems.append(first_failure)
    message = "通过代表点筛查" if accepted else "; ".join(problems) or "没有可评估路径点"
    return PoseTrial(
        name=pose_name(roll_degrees),
        roll_degrees=roll_degrees,
        accepted=accepted,
        sampled_waypoints=len(sampled),
        ik_failures=ik_failures,
        collision_count=collision_count,
        collision_links=tuple(sorted(collision_links)),
        configuration_count=len(configurations),
        max_joint_jump_degrees=max_jump,
        min_abs_j5_degrees=min_j5,
        message=message,
    )


def pose_name(roll_degrees: float) -> str:
    if abs(roll_degrees) <= 1e-9:
        return "base_y"
    sign = "p" if roll_degrees > 0.0 else "m"
    return f"tcp_z_{sign}{abs(roll_degrees):g}"


def sample_waypoints(waypoints: list[Waypoint], limit: int) -> list[Waypoint]:
    if limit <= 0 or len(waypoints) <= limit:
        return list(waypoints)
    if limit == 1:
        return [waypoints[0]]
    return [waypoints[round(index * (len(waypoints) - 1) / (limit - 1))] for index in range(limit)]


def target_for_waypoint(waypoint: Waypoint) -> CartesianTarget:
    q = waypoint.quaternion
    point = waypoint.position_world
    return CartesianTarget(
        name=f"avoidance_{waypoint.index}",
        pose=Pose(point[0], point[1], point[2], q1=q.w, q2=q.x, q3=q.y, q4=q.z),
    )
