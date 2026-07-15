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

from hole_aware_raster import hole_aware_raster_samples, polygon_has_relevant_holes, projected_raster_cell_samples
from robot_studio_qt.path_planning.mesh_raster import MeshTriangle, encoded_raster_line_id


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
    assert diagnostics["transfer_count"] == 3
    assert diagnostics["connector_count"] == 0
    assert diagnostics["cell_order"] == [0, 1, 2, 3]
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


def test_hole_check_accepts_partial_overlap_but_not_boundary_only_touch() -> None:
    patch = [(0.0, 0.0), (100.0, 0.0), (100.0, 100.0), (0.0, 100.0)]

    assert polygon_has_relevant_holes(patch, [[(90.0, 40.0), (110.0, 40.0), (110.0, 60.0), (90.0, 60.0)]])
    assert not polygon_has_relevant_holes(patch, [[(100.0, 40.0), (120.0, 40.0), (120.0, 60.0), (100.0, 60.0)]])


def test_projected_split_runs_form_three_cells_without_manifest() -> None:
    normal = (0.0, 0.0, 1.0)
    samples = []
    for line_index, segment_id, u_values in (
        (0, 0, [80.0, 100.0]),
        (1, 0, [40.0, 60.0, 80.0, 100.0]),
        (1, 1, [-100.0, -80.0, -60.0, -40.0]),
        (2, 1, list(range(-100, 101, 20))),
    ):
        samples.extend(
            (encoded_raster_line_id(segment_id, line_index), point_id, 1, (u, line_index * 10.0, 0.0), normal)
            for point_id, u in enumerate(u_values)
        )

    ordered, diagnostics = projected_raster_cell_samples(samples, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), spacing=10.0)

    assert diagnostics["valid"] is True
    assert diagnostics["cell_count"] == 3
    assert diagnostics["transfer_count"] == 2
    assert {sample[0] for sample in ordered} == {0, 1, 2}
    assert [sample[1] for sample in ordered if sample[0] == 0] == [0, 0, 1, 1, 1, 1]


def test_missing_scanline_starts_a_new_projected_cell() -> None:
    normal = (0.0, 0.0, 1.0)
    samples = [
        (encoded_raster_line_id(0, line_index), point_id, 1, (u, line_index * 10.0, 0.0), normal)
        for line_index in (0, 2)
        for point_id, u in enumerate((0.0, 20.0))
    ]

    _ordered, diagnostics = projected_raster_cell_samples(samples, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), spacing=10.0)

    assert diagnostics["cell_count"] == 2
    assert diagnostics["transfer_count"] == 1
