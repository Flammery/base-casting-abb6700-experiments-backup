from __future__ import annotations

from pathlib import Path
import sys


EXPERIMENT_DIR = Path(__file__).resolve().parents[1]
ROOT = EXPERIMENT_DIR.parents[1]
for folder in (ROOT / "src", EXPERIMENT_DIR / "scripts", EXPERIMENT_DIR / "experimental_algorithms"):
    if str(folder) not in sys.path:
        sys.path.insert(0, str(folder))

from robot_pose_avoidance import POSE_ROLL_DEGREES, apply_local_tcp_z_roll
from robot_studio_qt.kinematics.orientation import Quaternion
from robot_studio_qt.path_planning.models import PathResult, PathSource, RasterPlannerSettings, Waypoint


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
    assert POSE_ROLL_DEGREES == (0.0, 15.0, -15.0, 30.0, -30.0)


def test_local_tcp_z_roll_preserves_tcp_z_axis() -> None:
    original = _path()
    rolled = apply_local_tcp_z_roll(original, 30.0)

    original_z = original.waypoints[0].quaternion.rotate_vector((0.0, 0.0, 1.0))
    rolled_z = rolled.waypoints[0].quaternion.rotate_vector((0.0, 0.0, 1.0))
    assert rolled_z == original_z
    assert rolled.waypoints[0].quaternion != original.waypoints[0].quaternion
