from __future__ import annotations

from pathlib import Path
import sys

from vtkmodules.vtkCommonCore import vtkPoints
from vtkmodules.vtkCommonDataModel import vtkCellArray, vtkPolyData


EXPERIMENT_DIR = Path(__file__).resolve().parents[1]
ROOT = EXPERIMENT_DIR.parents[1]
for folder in (ROOT / "src", EXPERIMENT_DIR / "experimental_algorithms"):
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
    assert payload["selectors"] == ["1_2"]
    assert payload["regions"] == records
