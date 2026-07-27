from __future__ import annotations

import json
from pathlib import Path
import sys

from vtkmodules.vtkCommonCore import vtkPoints
from vtkmodules.vtkCommonDataModel import vtkCellArray, vtkPolyData


EXPERIMENT_DIR = Path(__file__).resolve().parents[1]
ROOT = EXPERIMENT_DIR.parents[1]
for folder in (
    ROOT / "src",
    EXPERIMENT_DIR / "scripts",
    EXPERIMENT_DIR / "experimental_algorithms",
):
    if str(folder) not in sys.path:
        sys.path.insert(0, str(folder))

from support_surface_growth import (  # noqa: E402
    AvoidanceVolumeSettings,
    avoidance_settings_path_for,
    build_avoidance_volume,
    build_obstacle_mesh_template,
    grow_support_surface,
    load_avoidance_settings,
    write_avoidance_settings,
)
from window_conf_export import support_for_planning_region  # noqa: E402


def _floor_and_wall_mesh() -> vtkPolyData:
    coordinates = [
        (0.0, 0.0, 0.0),
        (10.0, 0.0, 0.0),
        (10.0, 10.0, 0.0),
        (0.0, 10.0, 0.0),
        (10.0, 0.0, 10.0),
        (10.0, 10.0, 10.0),
    ]
    points = vtkPoints()
    for point in coordinates:
        points.InsertNextPoint(*point)
    cells = vtkCellArray()
    for triangle in ((0, 1, 2), (0, 2, 3), (1, 4, 5), (1, 5, 2)):
        cells.InsertNextCell(3)
        for point_id in triangle:
            cells.InsertCellPoint(point_id)
    mesh = vtkPolyData()
    mesh.SetPoints(points)
    mesh.SetPolys(cells)
    return mesh


def _floor_near_and_far_wall_mesh() -> vtkPolyData:
    coordinates = [
        (0.0, 0.0, 0.0),
        (10.0, 0.0, 0.0),
        (10.0, 10.0, 0.0),
        (0.0, 10.0, 0.0),
        (10.0, 0.0, 10.0),
        (10.0, 10.0, 10.0),
        (30.0, 0.0, 0.0),
        (30.0, 10.0, 0.0),
        (30.0, 0.0, 10.0),
        (30.0, 10.0, 10.0),
    ]
    points = vtkPoints()
    for point in coordinates:
        points.InsertNextPoint(*point)
    cells = vtkCellArray()
    for triangle in (
        (0, 1, 2),
        (0, 2, 3),
        (1, 4, 5),
        (1, 5, 2),
        (6, 8, 9),
        (6, 9, 7),
    ):
        cells.InsertNextCell(3)
        for point_id in triangle:
            cells.InsertCellPoint(point_id)
    mesh = vtkPolyData()
    mesh.SetPoints(points)
    mesh.SetPolys(cells)
    return mesh


def _concave_support_and_walls_mesh() -> vtkPolyData:
    coordinates = [
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (2.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (1.0, 1.0, 0.0),
        (2.0, 1.0, 0.0),
        (0.0, 2.0, 0.0),
        (1.0, 2.0, 0.0),
        # Wall inside the missing corner of the L-shaped support bounds.
        (1.4, 1.4, 0.0),
        (1.8, 1.4, 0.0),
        (1.4, 1.4, 1.0),
        (1.8, 1.4, 1.0),
        # Wall inside the lower arm of the actual support footprint.
        (1.4, 0.5, 0.0),
        (1.8, 0.5, 0.0),
        (1.4, 0.5, 1.0),
        (1.8, 0.5, 1.0),
    ]
    points = vtkPoints()
    for point in coordinates:
        points.InsertNextPoint(*point)
    cells = vtkCellArray()
    for triangle in (
        (0, 1, 4),
        (0, 4, 3),
        (1, 2, 5),
        (1, 5, 4),
        (3, 4, 7),
        (3, 7, 6),
        (8, 9, 11),
        (8, 11, 10),
        (12, 13, 15),
        (12, 15, 14),
    ):
        cells.InsertNextCell(3)
        for point_id in triangle:
            cells.InsertCellPoint(point_id)
    mesh = vtkPolyData()
    mesh.SetPoints(points)
    mesh.SetPolys(cells)
    return mesh


def test_seeded_growth_recovers_floor_and_stops_at_wall() -> None:
    mesh = _floor_and_wall_mesh()

    result = grow_support_surface(mesh, {0})

    assert result.seed_cell_ids == frozenset({0})
    assert result.support_cell_ids == frozenset({0, 1})
    assert result.max_normal_angle_deg == 0.0


def test_obstacle_template_excludes_recovered_support_surface() -> None:
    mesh = _floor_and_wall_mesh()
    support = grow_support_surface(mesh, {0})

    template = build_obstacle_mesh_template(mesh, support)

    assert template.support_cell_count == 2
    assert len(template.triangles_model) == 2
    assert all(all(point[0] == 10.0 for point in triangle) for triangle in template.triangles_model)


def test_complete_region_uses_exact_selected_cells_as_support() -> None:
    mesh = _floor_and_wall_mesh()
    planning_region = {
        "source_region": 1,
        "label": "1",
        "face_ids": {0, 2},
        "clip_polygon": None,
    }

    support = support_for_planning_region(mesh, planning_region, {0})

    assert support.seed_cell_ids == frozenset({0, 2})
    assert support.support_cell_ids == frozenset({0, 2})


def test_partition_patch_grows_support_and_always_keeps_source_cells() -> None:
    mesh = _floor_and_wall_mesh()
    planning_region = {
        "source_region": 1,
        "label": "1_1",
        "face_ids": {0, 2},
        "clip_polygon": [[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]],
    }

    support = support_for_planning_region(mesh, planning_region, {0})

    assert support.seed_cell_ids == frozenset({0})
    assert support.support_cell_ids == frozenset({0, 1, 2})


def test_uvn_volume_keeps_intersecting_near_wall_and_excludes_far_wall() -> None:
    mesh = _floor_near_and_far_wall_mesh()
    support = grow_support_surface(mesh, {0})

    volume = build_avoidance_volume(
        mesh,
        support,
        AvoidanceVolumeSettings(
            u_expand_percent=0.0,
            v_expand_percent=0.0,
            n_plus_mm=10.0,
            n_minus_mm=0.0,
        ),
        raster_chart={
            "origin": [0.0, 0.0, 0.0],
            "u_axis": [1.0, 0.0, 0.0],
            "v_axis": [0.0, 1.0, 0.0],
            "normal": [0.0, 0.0, 1.0],
        },
    )

    assert volume.obstacle_cell_ids == frozenset({2, 3})
    assert volume.outside_cell_count == 2
    assert len(volume.vertices_model) == 8


def test_uvn_volume_percentage_is_total_width_expansion() -> None:
    mesh = _floor_near_and_far_wall_mesh()
    support = grow_support_surface(mesh, {0})

    volume = build_avoidance_volume(
        mesh,
        support,
        AvoidanceVolumeSettings(
            u_expand_percent=400.0,
            v_expand_percent=0.0,
            n_plus_mm=10.0,
            n_minus_mm=0.0,
        ),
        raster_chart={
            "origin": [0.0, 0.0, 0.0],
            "u_axis": [1.0, 0.0, 0.0],
            "v_axis": [0.0, 1.0, 0.0],
            "normal": [0.0, 0.0, 1.0],
        },
    )

    assert volume.obstacle_cell_ids == frozenset({2, 3, 4, 5})


def test_uvn_volume_convex_hull_encloses_concave_support_bay() -> None:
    mesh = _concave_support_and_walls_mesh()
    support = grow_support_surface(mesh, {0})

    volume = build_avoidance_volume(
        mesh,
        support,
        AvoidanceVolumeSettings(
            u_expand_percent=0.0,
            v_expand_percent=0.0,
            n_plus_mm=1.0,
            n_minus_mm=0.0,
        ),
        raster_chart={
            "origin": [0.0, 0.0, 0.0],
            "u_axis": [1.0, 0.0, 0.0],
            "v_axis": [0.0, 1.0, 0.0],
            "normal": [0.0, 0.0, 1.0],
        },
    )

    assert support.support_cell_ids == frozenset(range(6))
    assert volume.obstacle_cell_ids == frozenset({6, 7, 8, 9})
    assert volume.outside_cell_count == 0
    assert len(volume.footprint_loops_uv) == 1
    assert volume.hull_vertex_count == 5
    assert len(volume.vertices_model) == 10
    assert len(volume.volume_faces) == 7
    assert all(len(face) >= 4 for face in volume.volume_faces)
    assert volume.as_dict()["volume_shape"] == "support-convex-hull-prism"


def test_avoidance_settings_sidecar_round_trip(tmp_path: Path) -> None:
    project_path = tmp_path / "sample.rsp.json"
    settings_path = avoidance_settings_path_for(project_path)
    records = [
        {
            "region_label": "1_2",
            "source_region": 1,
            "settings": {
                "u_expand_percent": 30.0,
                "v_expand_percent": 40.0,
                "n_plus_mm": 500.0,
                "n_minus_mm": 100.0,
            },
        }
    ]

    write_avoidance_settings(
        settings_path,
        input_project=project_path,
        selectors=["1_2"],
        records=records,
    )
    payload = load_avoidance_settings(settings_path)

    assert settings_path.name == "sample_avoidance.json"
    assert payload["version"] == 2
    assert payload["selectors"] == ["1_2"]
    assert payload["regions"] == records

    legacy_payload = json.loads(settings_path.read_text(encoding="utf-8"))
    legacy_payload["version"] = 1
    settings_path.write_text(json.dumps(legacy_payload), encoding="utf-8")
    assert load_avoidance_settings(settings_path)["version"] == 1
