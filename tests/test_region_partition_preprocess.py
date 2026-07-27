from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from robot_studio_qt.core.geometry import cross, dot, normalize
from robot_studio_qt.kinematics.model import WorkpiecePlacement
from robot_studio_qt.path_planning.mesh_raster import MeshTriangle
from robot_studio_qt.project import RobotStudioProject, load_project_file, save_project_file


def _load_preprocess_module():
    script = Path(__file__).resolve().parents[1] / "scripts" / "region_partition_preprocess.py"
    spec = importlib.util.spec_from_file_location("region_partition_preprocess_test", script)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
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


def test_preprocess_writes_loadable_partitioned_project(tmp_path, monkeypatch) -> None:
    module = _load_preprocess_module()
    input_path = tmp_path / "latest_script_test.rsp.json"
    output_path = tmp_path / "latest_partitioned.rsp.json"
    project = RobotStudioProject(
        workpiece=WorkpiecePlacement(file_path="dummy.step"),
        selected_path_face_regions=[[1], [2]],
    )
    save_project_file(input_path, project)
    triangles = [*_rect(1, 0.0, 10.0, 0.0, 10.0), *_rect(2, 20.0, 30.0, 0.0, 10.0)]

    monkeypatch.setattr(module, "load_polydata", lambda _project: object())
    monkeypatch.setattr(module, "read_triangles", lambda _polydata, _selected_face_ids: triangles)

    manifest = module.preprocess_project(
        input_path,
        output_path,
        {1},
        module.PartitionSettings(
            left_cut_x=-1_000_000.0,
            right_cut_x=1_000_000.0,
            max_base_x_span_mm=1_000_000.0,
            max_base_y_span_mm=1_000_000.0,
            min_patch_faces=1,
        ),
    )

    loaded = load_project_file(output_path)
    assert loaded.selected_path_face_regions == [[1], [2]]
    assert manifest["version"] == 2
    assert manifest["output_region_count"] == 2
    assert manifest["records"][0]["patches"][0]["turn_zone"] == "center"
    assert manifest["records"][0]["patches"][0]["surface_class"] == "main_plane"
    assert manifest["records"][0]["patches"][0]["collision_status"] == "not_evaluated"
    assert module.manifest_path_for(output_path).exists()
