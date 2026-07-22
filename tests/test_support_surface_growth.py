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

from support_surface_growth import build_obstacle_mesh_template, grow_support_surface  # noqa: E402


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
