from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[3]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from robot_studio_qt.kinematics.model import WorkpiecePlacement
from robot_studio_qt.kinematics.orientation import Quaternion
from robot_studio_qt.path_planning.mesh_raster import (
    MeshTriangle,
    ProjectedTriangle,
    encoded_raster_line_id,
    raster_segment_id,
    sample_projected_mesh,
)
from robot_studio_qt.path_planning.models import PathSource, RasterPlannerSettings, Waypoint


def _load_window_export_module():
    script = Path(__file__).resolve().parents[1] / "scripts" / "window_conf_export.py"
    spec = importlib.util.spec_from_file_location("window_conf_export_segments_test", script)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _projected_rect(face_id: int, x0: float, x1: float, y0: float, y1: float) -> list[ProjectedTriangle]:
    a = (x0, y0, 0.0)
    b = (x1, y0, 0.0)
    c = (x1, y1, 0.0)
    d = (x0, y1, 0.0)
    return [
        ProjectedTriangle(face_id, (a, b, c), ((x0, y0), (x1, y0), (x1, y1)), (0.0, 0.0, 1.0)),
        ProjectedTriangle(face_id, (a, c, d), ((x0, y0), (x1, y1), (x0, y1)), (0.0, 0.0, 1.0)),
    ]


def test_disconnected_scanline_intervals_become_separate_raster_segments() -> None:
    triangles = [
        *_projected_rect(1, 0.0, 10.0, 0.0, 10.0),
        *_projected_rect(2, 30.0, 40.0, 0.0, 10.0),
    ]
    settings = RasterPlannerSettings(spacing=5.0, point_step=5.0)

    samples = sample_projected_mesh(triangles, settings)
    segment_ids = {raster_segment_id(line_id) for line_id, _point_id, _face_id, _point, _normal in samples}

    assert segment_ids == {0, 1}
    assert all(point[0] <= 10.0 or point[0] >= 30.0 for _line, _pid, _fid, point, _normal in samples)


def _waypoint(index: int, line_id: int, position) -> Waypoint:
    return Waypoint(
        index=index,
        source=PathSource.MESH,
        region_id=0,
        face_id=1,
        line_id=line_id,
        point_id=index,
        position_model=position,
        position_world=position,
        position_wobj=position,
        normal_world=(0.0, 0.0, 1.0),
        normal_wobj=(0.0, 0.0, 1.0),
        quaternion=Quaternion(),
    )


def test_build_motion_adds_safe_approach_for_each_raster_segment() -> None:
    module = _load_window_export_module()
    path = SimpleNamespace(
        waypoints=[
            _waypoint(0, encoded_raster_line_id(0, 0), (0.0, 0.0, 0.0)),
            _waypoint(1, encoded_raster_line_id(0, 0), (10.0, 0.0, 0.0)),
            _waypoint(2, encoded_raster_line_id(1, 0), (30.0, 0.0, 0.0)),
            _waypoint(3, encoded_raster_line_id(1, 0), (40.0, 0.0, 0.0)),
        ]
    )

    motion = module.build_motion(WorkpiecePlacement(), path)

    assert [waypoint.index for waypoint in motion].count(-1) == 2
    assert len(motion) == 8


def test_hole_aware_motion_retracts_and_approaches_each_cell() -> None:
    module = _load_window_export_module()
    path = SimpleNamespace(
        waypoints=[
            _waypoint(0, encoded_raster_line_id(0, 0), (1000.0, 50.0, 200.0)),
            _waypoint(1, encoded_raster_line_id(1, 1), (1100.0, 60.0, 220.0)),
        ]
    )

    motion = module.build_hole_aware_motion(WorkpiecePlacement(), path)

    assert len(motion) == 6
    assert motion[0].position_world == (1000.0, 50.0, 350.0)
    assert motion[2].position_world == (1000.0, 50.0, 350.0)
    assert motion[3].position_world == (1100.0, 60.0, 370.0)
    assert motion[-1].position_world == (1100.0, 60.0, 370.0)
    assert [waypoint.index for waypoint in motion].count(-1) == 2


def test_split_scanline_detection_distinguishes_holes_from_normal_lines() -> None:
    module = _load_window_export_module()
    normal = SimpleNamespace(
        waypoints=[
            _waypoint(0, encoded_raster_line_id(0, 0), (0.0, 0.0, 0.0)),
            _waypoint(1, encoded_raster_line_id(1, 1), (0.0, 10.0, 0.0)),
        ]
    )
    split = SimpleNamespace(
        waypoints=[
            _waypoint(0, encoded_raster_line_id(0, 0), (0.0, 0.0, 0.0)),
            _waypoint(1, encoded_raster_line_id(1, 0), (20.0, 0.0, 0.0)),
        ]
    )

    assert module.path_has_split_scanlines(normal) is False
    assert module.path_has_split_scanlines(split) is True


def test_hole_aware_plans_projected_face_id_region_without_manifest(monkeypatch) -> None:
    module = _load_window_export_module()
    normal = (0.0, 0.0, 1.0)
    triangles = [
        MeshTriangle(1, ((-120.0, 0.0, 0.0), (120.0, 0.0, 0.0), (120.0, 30.0, 0.0)), normal, 3600.0),
        MeshTriangle(2, ((-120.0, 0.0, 0.0), (120.0, 30.0, 0.0), (-120.0, 30.0, 0.0)), normal, 3600.0),
    ]
    raw_samples = []
    for line_index, u_values in (
        (0, [80.0, 100.0]),
        (1, [100.0, 80.0, 60.0, 40.0, -40.0, -60.0, -80.0, -100.0]),
        (2, list(range(-100, 101, 20))),
    ):
        raw_samples.extend(
            (encoded_raster_line_id(0, line_index), point_id, 1, (u, line_index * 10.0, 0.0), normal)
            for point_id, u in enumerate(u_values)
        )
    monkeypatch.setattr(module, "read_triangles", lambda _polydata, _region: triangles)
    monkeypatch.setattr(module, "sample_projected_mesh", lambda _projected, _settings: raw_samples)
    settings = RasterPlannerSettings(spacing=10.0, point_step=20.0, boundary_margin=0.0)

    path = module.plan_region_uv_hole_aware(
        object(),
        WorkpiecePlacement(),
        settings,
        {1, 2},
        module.RasterFeedDirection.LONG_SIDE,
    )

    assert path.waypoints
    assert "3 cells" in path.message
    assert "projected-face-id" in path.message
    assert {raster_segment_id(waypoint.line_id) for waypoint in path.waypoints} == {0, 1, 2}
