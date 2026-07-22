"""Small experimental TCP-roll library for robot-arm collision trials.

The contact point and TCP +Z direction stay unchanged. Only a constant local
TCP-Z roll is applied to a complete region/patch path. Candidate paths are
screened with the reusable ``src`` IK, robot FK segments, and workpiece
collision mesh. Tool geometry is intentionally excluded during this phase.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

from robot_studio_qt.kinematics.model import JointState, RobotConfiguration, WorkpiecePlacement
from robot_studio_qt.kinematics.orientation import Quaternion
from robot_studio_qt.kinematics.robot_models import SerialRobotModel
from robot_studio_qt.kinematics.solvers import SerialKinematicsSolver, serial_configuration_from_joint_values
from robot_studio_qt.kinematics.targets import CartesianTarget, Pose
from robot_studio_qt.path_planning.models import PathResult, Waypoint
from robot_studio_qt.polishing_tool import PolishingToolData
from robot_studio_qt.tools.reachability.collision import (
    CollisionMesh,
    CollisionSettings,
    segment_triangle_distance_sq,
)


POSE_ROLL_DEGREES = (0.0, 15.0, -15.0, 30.0, -30.0)
EXPERIMENT_LINK_RADIUS_MM = 5.0
EXPERIMENT_CLEARANCE_MM = 0.0
EXPERIMENT_USE_SEGMENT_RADIUS = False
DEFAULT_MIN_CLEARANCE_MM = 5.0
DEFAULT_SAMPLE_LIMIT = 7
DEFAULT_MAX_JOINT_JUMP_DEGREES = 40.0
DEFAULT_MIN_ABS_J5_DEGREES = 6.0


@dataclass(frozen=True)
class PoseTrial:
    name: str
    roll_degrees: float
    validation_status: str
    accepted: bool
    sampled_waypoints: int
    ik_failures: int
    collision_count: int
    collision_links: tuple[str, ...]
    minimum_clearance_mm: float | None
    required_clearance_mm: float
    configuration_count: int
    max_joint_jump_degrees: float
    min_abs_j5_degrees: float | None
    message: str

    @property
    def interference(self) -> str:
        if self.collision_count:
            return "detected"
        if self.ik_failures or self.validation_status in {
            "joint-discontinuous",
            "near-singularity",
            "clearance-unresolved",
        }:
            return "not-confirmed"
        return "not-detected"

    def as_dict(self) -> dict:
        return {
            "pose_name": self.name,
            "roll_degrees": self.roll_degrees,
            "validation_status": self.validation_status,
            "accepted": self.accepted,
            "sampled_waypoints": self.sampled_waypoints,
            "ik_failures": self.ik_failures,
            "collision_count": self.collision_count,
            "collision_links": ",".join(self.collision_links),
            "minimum_clearance_mm": self.minimum_clearance_mm,
            "required_clearance_mm": self.required_clearance_mm,
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

    @property
    def selected_trial(self) -> PoseTrial:
        for trial in self.trials:
            if trial.name == self.selected_name:
                return trial
        return self.trials[0]

    @property
    def selected_minimum_clearance_mm(self) -> float | None:
        return self.selected_trial.minimum_clearance_mm


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
    minimum_clearance_mm: float = DEFAULT_MIN_CLEARANCE_MM,
    collision_settings: CollisionSettings | None = None,
    collision_mesh: CollisionMesh | None = None,
) -> PoseSelection:
    """Select the smallest validated TCP roll, or retain baseline for diagnosis.

    A fallback path remains geometrically valid and can be exported for
    RobotStudio follow-up, but ``validated`` stays false so it cannot become an
    internally validated optimal avoidance result.
    """

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
                required_clearance_mm=minimum_clearance_mm,
            )
        )

    accepted_indices = [index for index, trial in enumerate(trials) if trial.accepted]
    if not accepted_indices:
        return PoseSelection(path, fallback_selection_status(trials), trials[0].name, 0.0, tuple(trials))
    accepted_index = min(accepted_indices, key=lambda index: (abs(trials[index].roll_degrees), index))
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
    required_clearance_mm: float = DEFAULT_MIN_CLEARANCE_MM,
    max_joint_jump_degrees: float = DEFAULT_MAX_JOINT_JUMP_DEGREES,
    min_abs_j5_degrees: float = DEFAULT_MIN_ABS_J5_DEGREES,
) -> PoseTrial:
    sampled = sample_waypoints(path.waypoints, sample_limit)
    # Continuity is carried by the previous successful solution as the next
    # seed. Do not lock the ABB confdata tuple: a tiny J1/J4/J6 motion across a
    # 0/90-degree boundary is continuous even though the discrete tuple changes.
    solver = SerialKinematicsSolver(max_iterations=90, lock_configuration_to_seed=False)
    seed = initial_state.as_list()
    previous: list[float] | None = None
    ik_failures = 0
    collision_count = 0
    collision_links: set[str] = set()
    minimum_clearance = math.inf
    max_jump = 0.0
    min_j5_abs = math.inf
    configurations: set[tuple[int, int, int, int]] = set()
    first_failure = ""

    for waypoint in sampled:
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
        segments = robot.forward().pose.segments
        waypoint_collision_links = collision_mesh.colliding_segment_names(segments, collision_settings)
        waypoint_clearance = minimum_robot_clearance_mm(segments, collision_mesh, collision_settings)
        if waypoint_clearance is not None:
            minimum_clearance = min(minimum_clearance, waypoint_clearance)
        if waypoint_collision_links:
            collision_count += 1
            collision_links.update(waypoint_collision_links)
        seed = values
        previous = values

    min_j5 = None if min_j5_abs == math.inf else min_j5_abs
    min_clearance = None if minimum_clearance == math.inf else minimum_clearance
    validation_status = trial_validation_status(
        sampled=bool(sampled),
        ik_failures=ik_failures,
        collision_count=collision_count,
        minimum_clearance_mm=min_clearance,
        required_clearance_mm=required_clearance_mm,
        max_joint_jump_degrees=max_jump,
        allowed_joint_jump_degrees=max_joint_jump_degrees,
        min_abs_j5_degrees=min_j5,
        required_abs_j5_degrees=min_abs_j5_degrees,
    )
    accepted = validation_status == "validated-clear"
    problems: list[str] = []
    if ik_failures:
        problems.append(f"IK unresolved {ik_failures}/{len(sampled)}")
    if collision_count:
        problems.append(f"robot interference {collision_count}/{len(sampled)} ({','.join(sorted(collision_links))})")
    if min_clearance is None:
        problems.append("minimum robot clearance unavailable")
    elif min_clearance < required_clearance_mm:
        problems.append(f"minimum clearance {min_clearance:.2f} mm < required {required_clearance_mm:.2f} mm")
    if max_jump > max_joint_jump_degrees:
        problems.append(f"maximum joint jump {max_jump:.1f} deg > allowed {max_joint_jump_degrees:.1f} deg")
    if min_j5 is not None and min_j5 < min_abs_j5_degrees:
        problems.append(f"minimum |J5| {min_j5:.1f} deg < required {min_abs_j5_degrees:.1f} deg")
    if first_failure:
        problems.append(first_failure)
    message = "sampled waypoints validated" if accepted else "; ".join(problems) or "no evaluable waypoint"
    return PoseTrial(
        name=pose_name(roll_degrees),
        roll_degrees=roll_degrees,
        validation_status=validation_status,
        accepted=accepted,
        sampled_waypoints=len(sampled),
        ik_failures=ik_failures,
        collision_count=collision_count,
        collision_links=tuple(sorted(collision_links)),
        minimum_clearance_mm=min_clearance,
        required_clearance_mm=required_clearance_mm,
        configuration_count=len(configurations),
        max_joint_jump_degrees=max_jump,
        min_abs_j5_degrees=min_j5,
        message=message,
    )


def trial_validation_status(
    *,
    sampled: bool,
    ik_failures: int,
    collision_count: int,
    minimum_clearance_mm: float | None,
    required_clearance_mm: float,
    max_joint_jump_degrees: float,
    allowed_joint_jump_degrees: float,
    min_abs_j5_degrees: float | None,
    required_abs_j5_degrees: float,
) -> str:
    if not sampled or ik_failures:
        return "ik-unresolved"
    if max_joint_jump_degrees > allowed_joint_jump_degrees:
        return "joint-discontinuous"
    if min_abs_j5_degrees is not None and min_abs_j5_degrees < required_abs_j5_degrees:
        return "near-singularity"
    if collision_count:
        return "validated-interference"
    if minimum_clearance_mm is None:
        return "clearance-unresolved"
    if minimum_clearance_mm < required_clearance_mm:
        return "clearance-insufficient"
    return "validated-clear"


def fallback_selection_status(trials: list[PoseTrial]) -> str:
    """Summarize why no roll candidate can be selected as validated-clear."""

    priority = (
        "ik-unresolved",
        "joint-discontinuous",
        "near-singularity",
        "clearance-unresolved",
        "clearance-insufficient",
        "validated-interference",
    )
    statuses = {trial.validation_status for trial in trials}
    return next((status for status in priority if status in statuses), "fallback-unverified")


def minimum_robot_clearance_mm(segments, collision_mesh: CollisionMesh, settings: CollisionSettings) -> float | None:
    """Return sampled robot-envelope clearance to the workpiece mesh."""

    if not collision_mesh.triangles:
        return None
    minimum = math.inf
    for index, segment in enumerate(segments):
        if settings.ignore_tcp_segment and index == len(segments) - 1:
            continue
        configured_radius = segment.radius if settings.use_segment_radius else 0.0
        radius = max(settings.link_radius, configured_radius)
        for triangle in collision_mesh.triangles:
            centerline_distance = math.sqrt(
                segment_triangle_distance_sq(segment.start, segment.end, triangle)
            )
            minimum = min(minimum, centerline_distance - radius)
    return None if minimum == math.inf else minimum


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
