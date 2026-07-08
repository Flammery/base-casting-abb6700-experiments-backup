from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[3]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from robot_studio_qt.core.geometry import cross, dot, normalize
from robot_studio_qt.path_planning.mesh_raster import MeshTriangle


def _load_partitioning_module():
    script = Path(__file__).resolve().parents[1] / "scripts" / "region_partitioning.py"
    spec = importlib.util.spec_from_file_location("region_partitioning_test", script)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _triangle(face_id: int, a, b, c) -> MeshTriangle:
    normal_raw = cross((b[0] - a[0], b[1] - a[1], b[2] - a[2]), (c[0] - a[0], c[1] - a[1], c[2] - a[2]))
    area = 0.5 * (dot(normal_raw, normal_raw) ** 0.5)
    return MeshTriangle(face_id, (a, b, c), normalize(normal_raw), area)


def _add_rect(triangles: list[MeshTriangle], face_id: int, x0: float, x1: float, y0: float, y1: float, z: float = 0.0) -> int:
    a = (x0, y0, z)
    b = (x1, y0, z)
    c = (x1, y1, z)
    d = (x0, y1, z)
    triangles.append(_triangle(face_id, a, b, c))
    triangles.append(_triangle(face_id, a, c, d))
    return face_id + 1


def _grid_rects(triangles: list[MeshTriangle], face_id: int, x0: float, x1: float, y0: float, y1: float, step: float) -> int:
    x = x0
    while x < x1 - 1e-9:
        nx = min(x + step, x1)
        y = y0
        while y < y1 - 1e-9:
            ny = min(y + step, y1)
            face_id = _add_rect(triangles, face_id, x, nx, y, ny)
            y = ny
        x = nx
    return face_id


def test_neck_barrier_splits_two_large_planar_areas() -> None:
    module = _load_partitioning_module()
    triangles: list[MeshTriangle] = []
    face_id = 0
    face_id = _grid_rects(triangles, face_id, 0.0, 100.0, 0.0, 100.0, 10.0)
    face_id = _grid_rects(triangles, face_id, 100.0, 120.0, 40.0, 60.0, 10.0)
    face_id = _grid_rects(triangles, face_id, 120.0, 220.0, 0.0, 100.0, 10.0)
    faces = module.face_geometries_from_triangles(triangles)
    settings = module.PartitionSettings(
        left_cut_x=-1_000_000.0,
        right_cut_x=1_000_000.0,
        max_base_x_span_mm=1_000_000.0,
        max_base_y_span_mm=1_000_000.0,
        scan_spacing_mm=10.0,
        neck_width_ratio=0.35,
        min_neck_lines=1,
        min_patch_faces=5,
        min_patch_area_ratio=0.0,
    )

    partitioned = module.partition_selected_regions([list(faces)], {1}, faces, settings)

    assert len(partitioned.regions) == 2
    assert partitioned.records[0].unchanged is False
    assert {patch.label for patch in partitioned.records[0].patches} == {"1.1", "1.2"}


def test_wide_connected_l_shape_is_not_cut_by_global_axis() -> None:
    module = _load_partitioning_module()
    triangles: list[MeshTriangle] = []
    face_id = 0
    face_id = _grid_rects(triangles, face_id, 0.0, 100.0, 0.0, 100.0, 20.0)
    face_id = _grid_rects(triangles, face_id, 40.0, 100.0, 100.0, 200.0, 20.0)
    faces = module.face_geometries_from_triangles(triangles)
    settings = module.PartitionSettings(
        left_cut_x=-1_000_000.0,
        right_cut_x=1_000_000.0,
        max_base_x_span_mm=1_000_000.0,
        max_base_y_span_mm=1_000_000.0,
        scan_spacing_mm=10.0,
        neck_width_ratio=0.25,
        min_neck_lines=1,
        min_patch_faces=1,
        min_patch_area_ratio=0.0,
    )

    partitioned = module.partition_selected_regions([list(faces)], {1}, faces, settings)

    assert len(partitioned.regions) == 1
    assert partitioned.records[0].unchanged is True


def test_auto_turn_zone_cuts_find_two_column_valleys() -> None:
    module = _load_partitioning_module()
    triangles: list[MeshTriangle] = []
    face_id = 0
    face_id = _grid_rects(triangles, face_id, -300.0, -150.0, 0.0, 100.0, 25.0)
    face_id = _grid_rects(triangles, face_id, -50.0, 50.0, 0.0, 100.0, 25.0)
    _grid_rects(triangles, face_id, 150.0, 300.0, 0.0, 100.0, 25.0)
    faces = module.face_geometries_from_triangles(triangles)

    left_cut, right_cut = module.auto_turn_zone_cuts(set(faces), faces, module.PartitionSettings(turn_histogram_bin_mm=25.0))

    assert -150.0 < left_cut < -50.0
    assert 50.0 < right_cut < 150.0


def test_manual_turn_zone_cuts_override_auto_detection() -> None:
    module = _load_partitioning_module()
    triangles: list[MeshTriangle] = []
    face_id = 0
    face_id = _grid_rects(triangles, face_id, -200.0, -100.0, 0.0, 100.0, 50.0)
    face_id = _grid_rects(triangles, face_id, -25.0, 25.0, 0.0, 100.0, 50.0)
    _grid_rects(triangles, face_id, 100.0, 200.0, 0.0, 100.0, 50.0)
    faces = module.face_geometries_from_triangles(triangles)
    adjacency = module.build_face_adjacency(faces, module.PartitionSettings())
    settings = module.PartitionSettings(left_cut_x=-75.0, right_cut_x=75.0)

    zones = module.split_by_turn_zone(set(faces), faces, adjacency, settings)

    assert {zone for zone, _patch in zones} == {"left", "center", "right"}


def test_partition_records_turn_zone_before_surface_classification() -> None:
    module = _load_partitioning_module()
    triangles: list[MeshTriangle] = []
    face_id = 0
    face_id = _grid_rects(triangles, face_id, -200.0, -100.0, 0.0, 100.0, 50.0)
    face_id = _grid_rects(triangles, face_id, -25.0, 25.0, 0.0, 100.0, 50.0)
    _grid_rects(triangles, face_id, 100.0, 200.0, 0.0, 100.0, 50.0)
    faces = module.face_geometries_from_triangles(triangles)
    settings = module.PartitionSettings(
        left_cut_x=-75.0,
        right_cut_x=75.0,
        left_angles=(345, 0, 15),
        right_angles=(180,),
        center_angles=(270,),
        max_base_x_span_mm=1_000_000.0,
        max_base_y_span_mm=1_000_000.0,
        min_patch_faces=1,
        min_patch_area_ratio=0.0,
    )

    partitioned = module.partition_selected_regions([list(faces)], {1}, faces, settings)
    patches = partitioned.records[0].patches

    assert {patch.turn_zone for patch in patches} == {"left", "center", "right"}
    assert next(patch for patch in patches if patch.turn_zone == "left").allowed_angles == (345, 0, 15)
    assert {patch.surface_class for patch in patches} == {"main_plane"}


def test_slope_patch_is_manifest_compatible_curved_kind() -> None:
    module = _load_partitioning_module()
    triangles = [
        _triangle(1, (0.0, 0.0, 0.0), (100.0, 0.0, 0.0), (0.0, 100.0, 0.0)),
        _triangle(2, (100.0, 0.0, 0.0), (0.0, 100.0, 0.0), (100.0, 100.0, 80.0)),
    ]
    faces = module.face_geometries_from_triangles(triangles)
    settings = module.PartitionSettings(
        left_cut_x=-1_000_000.0,
        right_cut_x=1_000_000.0,
        max_base_x_span_mm=1_000_000.0,
        max_base_y_span_mm=1_000_000.0,
        planar_normal_deg=5.0,
        planar_seed_min_faces=1,
        min_patch_faces=1,
        min_patch_area_ratio=0.0,
    )

    partitioned = module.partition_selected_regions([[1, 2]], {1}, faces, settings)

    assert {patch.kind for patch in partitioned.records[0].patches} == {"curved"}
    assert {patch.surface_class for patch in partitioned.records[0].patches} == {"slope"}


def test_base_window_size_split_cuts_long_patch() -> None:
    module = _load_partitioning_module()
    triangles: list[MeshTriangle] = []
    _grid_rects(triangles, 0, 0.0, 800.0, 0.0, 2400.0, 200.0)
    faces = module.face_geometries_from_triangles(triangles)
    settings = module.PartitionSettings(
        left_cut_x=-1_000_000.0,
        right_cut_x=1_000_000.0,
        center_angles=(0,),
        max_base_x_span_mm=1000.0,
        max_base_y_span_mm=2000.0,
        min_patch_faces=1,
        min_patch_area_ratio=0.0,
    )

    partitioned = module.partition_selected_regions([list(faces)], {1}, faces, settings)

    assert len(partitioned.regions) == 2
    assert all(patch.split_reason.startswith("size_split_base_y") for patch in partitioned.records[0].patches)


def test_curvature_stage_can_return_planar_and_curved_patches() -> None:
    module = _load_partitioning_module()
    faces = {
        1: module.FaceGeometry(1, (), 1.0, (0.0, 0.0, 1.0), (0.0, 0.0, 0.0)),
        2: module.FaceGeometry(2, (), 1.0, (0.0, 0.0, 1.0), (1.0, 0.0, 0.0)),
        3: module.FaceGeometry(3, (), 1.0, normalize((0.0, 0.0, 1.0)), (3.0, 0.0, 0.0)),
        4: module.FaceGeometry(4, (), 1.0, normalize((0.17, 0.0, 0.98)), (4.0, 0.0, 0.0)),
        5: module.FaceGeometry(5, (), 1.0, normalize((0.34, 0.0, 0.94)), (5.0, 0.0, 0.0)),
    }
    adjacency = {1: {2}, 2: {1}, 3: {4}, 4: {3, 5}, 5: {4}}
    settings = module.PartitionSettings(hard_edge_angle_deg=20.0, planar_normal_deg=5.0, planar_rms_mm=100.0)

    patches = module.partition_by_curvature(set(faces), faces, adjacency, settings)

    assert sorted(kind for kind, _patch in patches) == ["curved", "planar"]


def test_unselected_regions_pass_through_unchanged() -> None:
    module = _load_partitioning_module()
    triangles: list[MeshTriangle] = []
    _add_rect(triangles, 10, 0.0, 10.0, 0.0, 10.0)
    _add_rect(triangles, 20, 20.0, 30.0, 0.0, 10.0)
    faces = module.face_geometries_from_triangles(triangles)

    partitioned = module.partition_selected_regions([[10], [20]], {2}, faces, module.PartitionSettings(min_patch_faces=1))

    assert partitioned.regions[0] == [10]
    assert partitioned.records[0].reason == "not selected"
    assert partitioned.regions[1] == [20]
