from __future__ import annotations

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[3]
SRC = ROOT / "src"
SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
EXPERIMENTAL = Path(__file__).resolve().parents[1] / "experimental_algorithms"
for path in (SRC, SCRIPTS, EXPERIMENTAL):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from hole_aware_raster import hole_aware_raster_samples, polygon_has_relevant_holes
from robot_studio_qt.path_planning.mesh_raster import MeshTriangle


def _plane_triangles():
    normal = (0.0, 0.0, 1.0)
    return [
        MeshTriangle(1, ((0.0, 0.0, 5.0), (100.0, 0.0, 5.0), (100.0, 100.0, 5.0)), normal, 5000.0),
        MeshTriangle(2, ((0.0, 0.0, 5.0), (100.0, 100.0, 5.0), (0.0, 100.0, 5.0)), normal, 5000.0),
    ]


def _chart():
    return {
        "origin": [0.0, 0.0, 0.0],
        "u_axis": [1.0, 0.0, 0.0],
        "v_axis": [0.0, 1.0, 0.0],
        "normal": [0.0, 0.0, 1.0],
    }


def test_central_hole_is_split_into_cells_without_cross_hole_motion() -> None:
    hole = [(40.0, 30.0), (60.0, 30.0), (60.0, 70.0), (40.0, 70.0)]
    samples, diagnostics = hole_aware_raster_samples(
        [(0.0, 0.0), (100.0, 0.0), (100.0, 100.0), (0.0, 100.0)],
        [hole],
        _plane_triangles(),
        _chart(),
        spacing=10.0,
        point_step=10.0,
        margin=2.0,
    )

    assert diagnostics["valid"] is True
    assert diagnostics["cell_count"] == 4
    assert samples
    assert all(not (40.0 < sample[4][0] < 60.0 and 30.0 < sample[4][1] < 70.0) for sample in samples)
    for first, second in zip(samples, samples[1:]):
        first_point, second_point = first[4], second[4]
        crosses_hole_on_same_line = (
            abs(first_point[1] - second_point[1]) < 1e-6
            and min(first_point[0], second_point[0]) < 40.0
            and max(first_point[0], second_point[0]) > 60.0
            and 30.0 < first_point[1] < 70.0
        )
        assert not crosses_hole_on_same_line


def test_fast_hole_check_ignores_holes_outside_current_patch() -> None:
    patch = [(0.0, 0.0), (100.0, 0.0), (100.0, 100.0), (0.0, 100.0)]

    assert polygon_has_relevant_holes(patch, [[(40.0, 40.0), (60.0, 40.0), (60.0, 60.0), (40.0, 60.0)]])
    assert not polygon_has_relevant_holes(patch, [[(140.0, 40.0), (160.0, 40.0), (160.0, 60.0), (140.0, 60.0)]])
