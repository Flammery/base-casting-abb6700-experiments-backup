from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
SRC = ROOT / "src"
SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
for path in (SRC, SCRIPTS):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from robot_studio_qt.path_planning.mesh_raster import MeshTriangle
from raster_domain import patch_axes, raster_samples


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


def test_each_patch_uses_its_own_long_axis() -> None:
    horizontal, _ = patch_axes([(0, 0), (100, 0), (100, 20), (0, 20)])
    vertical, _ = patch_axes([(0, 0), (20, 0), (20, 100), (0, 100)])

    assert abs(horizontal[0] * vertical[0] + horizontal[1] * vertical[1]) < 0.2


def test_raster_domain_lifts_xyz_and_keeps_hole_empty() -> None:
    outer = [(0, 0), (100, 0), (100, 100), (0, 100)]
    hole = [(40, 40), (60, 40), (60, 60), (40, 60)]

    samples = raster_samples(outer, [hole], _plane_triangles(), _chart(), spacing=10.0, point_step=10.0, margin=0.0)

    assert samples
    assert {sample[4][2] for sample in samples} == {5.0}
    assert {sample[5] for sample in samples} == {(0.0, 0.0, 1.0)}
    assert not any(40.0 < sample[4][0] < 60.0 and 40.0 < sample[4][1] < 60.0 for sample in samples)
