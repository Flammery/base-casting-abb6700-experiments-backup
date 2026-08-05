from __future__ import annotations

from pathlib import Path
import sys
from types import SimpleNamespace


EXPERIMENT_DIR = Path(__file__).resolve().parents[1]
ROOT = EXPERIMENT_DIR.parents[1]
for folder in (ROOT / "src", EXPERIMENT_DIR / "scripts", EXPERIMENT_DIR / "experimental_algorithms"):
    if str(folder) not in sys.path:
        sys.path.insert(0, str(folder))

import robot_pose_avoidance  # noqa: E402
from robot_pose_avoidance import (  # noqa: E402
    POSE_ROLL_DEGREES,
    PoseTrial,
    apply_local_tcp_z_roll,
    evaluate_robot_pose,
    minimum_robot_clearance_mm,
    select_robot_pose,
)
from robot_studio_qt.kinematics.model import JointState, RobotConfiguration, WorkpiecePlacement  # noqa: E402
from robot_studio_qt.kinematics.kinematics import Segment  # noqa: E402
from robot_studio_qt.kinematics.orientation import Quaternion  # noqa: E402
from robot_studio_qt.path_planning.models import PathResult, PathSource, RasterPlannerSettings, Waypoint  # noqa: E402
from robot_studio_qt.tools.reachability.collision import CollisionMesh, CollisionSettings  # noqa: E402


def _path() -> PathResult:
    waypoint = Waypoint(
        index=0,
        source=PathSource.MESH,
        region_id=0,
        face_id=1,
        line_id=0,
        point_id=0,
        position_model=(0.0, 0.0, 0.0),
        position_world=(0.0, 0.0, 0.0),
        position_wobj=(0.0, 0.0, 0.0),
        normal_world=(0.0, 0.0, 1.0),
        normal_wobj=(0.0, 0.0, 1.0),
        quaternion=Quaternion(),
    )
    return PathResult(PathSource.MESH, "wobj0", RasterPlannerSettings(), [waypoint], "test")


def test_small_pose_library_is_deterministic_and_contains_baseline() -> None:
    assert POSE_ROLL_DEGREES == (
        -90.0,
        -75.0,
        -60.0,
        -45.0,
        -30.0,
        -15.0,
        0.0,
        15.0,
        30.0,
        45.0,
        60.0,
        75.0,
        90.0,
    )


def test_unconfigured_avoidance_uses_uniform_100_mm_link_radius() -> None:
    assert robot_pose_avoidance.EXPERIMENT_LINK_RADIUS_MM == 100.0
    assert robot_pose_avoidance.EXPERIMENT_USE_SEGMENT_RADIUS is False


def test_local_tcp_z_roll_preserves_tcp_z_axis() -> None:
    original = _path()
    rolled = apply_local_tcp_z_roll(original, 30.0)

    original_z = original.waypoints[0].quaternion.rotate_vector((0.0, 0.0, 1.0))
    rolled_z = rolled.waypoints[0].quaternion.rotate_vector((0.0, 0.0, 1.0))
    assert rolled_z == original_z
    assert rolled.waypoints[0].quaternion != original.waypoints[0].quaternion


def test_pose_selection_maximizes_clearance_without_preferring_small_roll(monkeypatch) -> None:
    clearances = {
        -90.0: 10.0,
        -75.0: 11.0,
        -60.0: 12.0,
        -45.0: 13.0,
        -30.0: 20.0,
        -15.0: 21.0,
        0.0: 22.0,
        15.0: 23.0,
        30.0: 40.0,
        45.0: 50.0,
        60.0: 60.0,
        75.0: 70.0,
        90.0: 80.0,
    }

    def fake_evaluate(*_args, **kwargs):
        roll_degrees = float(_args[6])
        return PoseTrial(
            name=robot_pose_avoidance.pose_name(roll_degrees),
            roll_degrees=roll_degrees,
            validation_status="validated-clear",
            accepted=True,
            sampled_waypoints=1,
            ik_failures=0,
            collision_count=0,
            collision_links=(),
            minimum_clearance_mm=clearances[roll_degrees],
            required_clearance_mm=float(kwargs["required_clearance_mm"]),
            configuration_count=1,
            max_joint_jump_degrees=0.0,
            min_abs_j5_degrees=90.0,
            message="sampled waypoints validated",
        )

    monkeypatch.setattr(robot_pose_avoidance, "evaluate_robot_pose", fake_evaluate)
    selection = select_robot_pose(
        _path(),
        None,
        WorkpiecePlacement(),
        RobotConfiguration(),
        JointState([0.0] * 6),
        None,
        collision_mesh=CollisionMesh([]),
    )

    assert selection.selected_roll_degrees == 90.0
    assert selection.selected_minimum_clearance_mm == 80.0
    assert selection.status == "alternative-validated"


def test_minimum_robot_clearance_subtracts_experimental_link_radius() -> None:
    mesh = CollisionMesh([((0.0, 10.0, 0.0), (10.0, 10.0, 0.0), (0.0, 10.0, 10.0))])
    segment = Segment("test", (0.0, 0.0, 0.0), (10.0, 0.0, 0.0), 50.0, "#000000")
    settings = CollisionSettings(
        link_radius=2.0,
        clearance=0.0,
        ignore_tcp_segment=False,
        use_segment_radius=False,
    )

    assert minimum_robot_clearance_mm([segment], mesh, settings) == 8.0


def test_avoidance_continuity_accepts_small_j6_crossing_between_confdata_sectors(monkeypatch) -> None:
    path = _path()
    path.waypoints.append(path.waypoints[0])
    solutions = iter(
        (
            [0.0, 0.0, 0.0, 0.0, 65.0, 0.2],
            [0.0, 0.0, 0.0, 0.0, 65.0, -0.2],
        )
    )

    class FakeSolver:
        def __init__(self, *, max_iterations, lock_configuration_to_seed):
            assert max_iterations == 90
            assert lock_configuration_to_seed is False

        def inverse(self, _robot, _target, _seed):
            return SimpleNamespace(success=True, joint_target=SimpleNamespace(values=next(solutions)))

    class ClearMesh:
        triangles = [((10000.0, 0.0, 0.0), (10000.0, 1.0, 0.0), (10000.0, 0.0, 1.0))]

        @staticmethod
        def colliding_segment_names(_segments, _settings):
            return []

    monkeypatch.setattr(robot_pose_avoidance, "SerialKinematicsSolver", FakeSolver)
    monkeypatch.setattr(robot_pose_avoidance, "minimum_robot_clearance_mm", lambda *_args: 100.0)
    trial = evaluate_robot_pose(
        path,
        RobotConfiguration(),
        JointState([0.0] * 6),
        None,
        ClearMesh(),
        CollisionSettings(ignore_tcp_segment=False),
        0.0,
        sample_limit=2,
    )

    assert trial.configuration_count == 2
    assert trial.max_joint_jump_degrees == 0.4
    assert trial.validation_status == "validated-clear"
    assert trial.accepted is True
