from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from robot_studio_qt.core.geometry import cross, dot, normalize
from robot_studio_qt.path_planning.mesh_raster import MeshTriangle


def _load_manual_module():
    script = Path(__file__).resolve().parents[1] / "scripts" / "manual_region_partitioning.py"
    spec = importlib.util.spec_from_file_location("manual_region_partitioning_test", script)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _load_partitioning_module():
    script = Path(__file__).resolve().parents[1] / "scripts" / "region_partitioning.py"
    spec = importlib.util.spec_from_file_location("region_partitioning_for_manual_test", script)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _triangle(face_id: int, a, b, c) -> MeshTriangle:
    normal_raw = cross((b[0] - a[0], b[1] - a[1], b[2] - a[2]), (c[0] - a[0], c[1] - a[1], c[2] - a[2]))
    area = 0.5 * (dot(normal_raw, normal_raw) ** 0.5)
    return MeshTriangle(face_id, (a, b, c), normalize(normal_raw), area)


def _rect(face_id: int, x0: float, x1: float, y0: float, y1: float) -> list[MeshTriangle]:
    a = (x0, y0, 0.0)
    b = (x1, y0, 0.0)
    c = (x1, y1, 0.0)
    d = (x0, y1, 0.0)
    return [_triangle(face_id, a, b, c), _triangle(face_id, a, c, d)]


def _frame_with_rect_hole() -> list[MeshTriangle]:
    return [
        *_rect(10, 0.0, 40.0, 0.0, 40.0),
        *_rect(11, 40.0, 60.0, 0.0, 40.0),
        *_rect(12, 60.0, 100.0, 0.0, 40.0),
        *_rect(13, 0.0, 40.0, 40.0, 60.0),
        *_rect(14, 60.0, 100.0, 40.0, 60.0),
        *_rect(15, 0.0, 40.0, 60.0, 100.0),
        *_rect(16, 40.0, 60.0, 60.0, 100.0),
        *_rect(17, 60.0, 100.0, 60.0, 100.0),
    ]


def test_manual_barrier_splits_adjacent_faces() -> None:
    manual = _load_manual_module()
    partitioning = _load_partitioning_module()
    triangles = [*_rect(1, 0.0, 100.0, 0.0, 100.0), *_rect(2, 100.0, 200.0, 0.0, 100.0)]
    faces = partitioning.face_geometries_from_triangles(triangles)

    result = manual.partition_face_ids_by_barriers({1, 2}, faces, [((100.0, -10.0), (100.0, 110.0))])

    assert result.cut_edge_count > 0
    assert result.regions == [[1], [2]]


def test_replace_regions_only_partitions_selected_region() -> None:
    manual = _load_manual_module()
    partitioning = _load_partitioning_module()
    triangles = [*_rect(1, 0.0, 100.0, 0.0, 100.0), *_rect(2, 100.0, 200.0, 0.0, 100.0), *_rect(3, 300.0, 400.0, 0.0, 100.0)]
    faces = partitioning.face_geometries_from_triangles(triangles)

    regions, records = manual.replace_regions_with_manual_partitions([[1, 2], [3]], {1}, faces, [((100.0, -10.0), (100.0, 110.0))])

    assert regions == [[1], [2], [3]]
    assert records[0]["output_patch_count"] == 2
    assert records[1]["unchanged"] is True


def test_clip_partitions_keep_source_region_labels() -> None:
    manual = _load_manual_module()
    partitioning = _load_partitioning_module()
    triangles = [*_rect(10, 0.0, 300.0, 0.0, 100.0)]
    faces = partitioning.face_geometries_from_triangles(triangles)

    partitions = manual.clip_partitions_from_barriers(
        6,
        {10},
        faces,
        [((100.0, -20.0), (100.0, 120.0)), ((200.0, -20.0), (200.0, 120.0))],
    )

    assert [partition.label for partition in partitions] == ["6_1", "6_2", "6_3"]
    assert all(len(partition.clip_polygon_model_xy) >= 3 for partition in partitions)
    assert any(partition.exclude_polygons_model_xy for partition in partitions)


def test_slab_clip_partitions_keep_old_through_cut_behavior() -> None:
    manual = _load_manual_module()
    partitioning = _load_partitioning_module()
    triangles = [*_rect(10, 0.0, 300.0, 0.0, 100.0)]
    faces = partitioning.face_geometries_from_triangles(triangles)

    partitions = manual.clip_partitions_from_barriers(
        6,
        {10},
        faces,
        [((100.0, -20.0), (100.0, 120.0)), ((200.0, -20.0), (200.0, 120.0))],
        mode=manual.PARTITION_MODE_SLAB,
    )

    assert [partition.label for partition in partitions] == ["6_1", "6_2", "6_3"]
    assert all(not partition.exclude_polygons_model_xy for partition in partitions)
    assert min(point[1] for partition in partitions for point in partition.clip_polygon_model_xy) < 0.0
    assert max(point[1] for partition in partitions for point in partition.clip_polygon_model_xy) > 100.0


def test_clip_partition_uses_projected_face_boundary_not_bbox() -> None:
    manual = _load_manual_module()
    partitioning = _load_partitioning_module()
    triangles = [
        *_rect(10, 0.0, 60.0, 0.0, 100.0),
        *_rect(12, 60.0, 300.0, 0.0, 100.0),
        *_rect(11, 0.0, 60.0, 100.0, 180.0),
    ]
    faces = partitioning.face_geometries_from_triangles(triangles)

    partitions = manual.clip_partitions_from_barriers(
        6,
        {10, 11},
        faces,
        [((70.0, 20.0), (70.0, 160.0))],
    )

    side = min(partitions, key=lambda partition: sum(point[0] for point in partition.clip_polygon_model_xy) / len(partition.clip_polygon_model_xy))
    assert (60.0, 100.0) in side.clip_polygon_model_xy
    assert (0.0, 180.0) in side.clip_polygon_model_xy
    assert all(point[0] <= 70.0 for point in side.clip_polygon_model_xy)


def test_boundary_clip_records_inner_holes_as_exclusions() -> None:
    manual = _load_manual_module()
    partitioning = _load_partitioning_module()
    triangles = _frame_with_rect_hole()
    faces = partitioning.face_geometries_from_triangles(triangles)
    face_ids = set(faces)

    partitions = manual.clip_partitions_from_barriers(
        6,
        face_ids,
        faces,
        [((30.0, -10.0), (30.0, 110.0))],
        mode=manual.PARTITION_MODE_BOUNDARY,
    )

    assert partitions
    assert all(partition.exclude_polygons_model_xy for partition in partitions)
    hole_points = {point for partition in partitions for polygon in (partition.exclude_polygons_model_xy or []) for point in polygon}
    assert (40.0, 40.0) in hole_points
    assert (60.0, 60.0) in hole_points


def test_pick_clip_only_outputs_selected_polygons() -> None:
    manual = _load_manual_module()
    partitioning = _load_partitioning_module()
    triangles = [*_rect(10, 0.0, 300.0, 0.0, 100.0)]
    faces = partitioning.face_geometries_from_triangles(triangles)

    partitions = manual.clip_partitions_from_picked_polygons(
        6,
        {10},
        faces,
        [
            [(0.0, 0.0), (80.0, 0.0), (80.0, 100.0), (0.0, 100.0)],
            [(200.0, 0.0), (300.0, 0.0), (300.0, 100.0), (200.0, 100.0)],
        ],
    )

    assert [partition.label for partition in partitions] == ["6_1", "6_2"]
    assert partitions[0].clip_polygon_model_xy[0] == (0.0, 0.0)
    assert partitions[1].clip_polygon_model_xy[0] == (200.0, 0.0)


def test_pick_clip_keeps_holes_as_exclusions() -> None:
    manual = _load_manual_module()
    partitioning = _load_partitioning_module()
    triangles = _frame_with_rect_hole()
    faces = partitioning.face_geometries_from_triangles(triangles)

    records = manual.manual_pick_manifest_records(
        [sorted(faces)],
        {1},
        faces,
        [[(0.0, 0.0), (100.0, 0.0), (100.0, 100.0), (0.0, 100.0)]],
    )

    assert records[0]["reason"] == "manual_uv_pick_clip"
    assert records[0]["partition_mode"] == manual.PARTITION_MODE_PICK
    assert [patch["label"] for patch in records[0]["patches"]] == ["1_1"]
    hole_points = {tuple(point) for polygon in records[0]["patches"][0]["exclude_polygons"] for point in polygon}
    assert (40.0, 40.0) in hole_points
    assert (60.0, 60.0) in hole_points


def test_manual_clip_manifest_records_do_not_renumber_following_regions() -> None:
    manual = _load_manual_module()
    partitioning = _load_partitioning_module()
    triangles = [*_rect(10, 0.0, 300.0, 0.0, 100.0), *_rect(20, 500.0, 600.0, 0.0, 100.0)]
    faces = partitioning.face_geometries_from_triangles(triangles)

    records = manual.manual_clip_manifest_records(
        [[10], [20]],
        {1},
        faces,
        [((100.0, -20.0), (100.0, 120.0)), ((200.0, -20.0), (200.0, 120.0))],
    )

    assert len(records) == 1
    assert records[0]["original_region"] == 1
    assert [patch["label"] for patch in records[0]["patches"]] == ["1_1", "1_2", "1_3"]


def test_manual_clip_manifest_records_include_partition_mode() -> None:
    manual = _load_manual_module()
    partitioning = _load_partitioning_module()
    triangles = [*_rect(10, 0.0, 300.0, 0.0, 100.0)]
    faces = partitioning.face_geometries_from_triangles(triangles)

    records = manual.manual_clip_manifest_records(
        [[10]],
        {1},
        faces,
        [((100.0, -20.0), (100.0, 120.0))],
        mode=manual.PARTITION_MODE_SLAB,
    )

    assert records[0]["reason"] == "manual_uv_slab_clip"
    assert records[0]["partition_mode"] == manual.PARTITION_MODE_SLAB
    assert {patch["partition_mode"] for patch in records[0]["patches"]} == {manual.PARTITION_MODE_SLAB}
