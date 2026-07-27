from __future__ import annotations

import importlib.util
from pathlib import Path
import sys


def _load_step_module():
    script = Path(__file__).resolve().parents[1] / "scripts" / "step_cad_core.py"
    spec = importlib.util.spec_from_file_location("step_cad_core_test", script)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _write_step(path: Path, shape) -> None:
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.STEPControl import STEPControl_AsIs, STEPControl_Writer

    writer = STEPControl_Writer()
    writer.Transfer(shape, STEPControl_AsIs)
    status = writer.Write(str(path))
    assert status == IFSelect_RetDone


def _box_step(path: Path) -> None:
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox

    _write_step(path, BRepPrimAPI_MakeBox(100.0, 80.0, 20.0).Shape())


def _hole_face_step(path: Path) -> None:
    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakePolygon
    from OCP.gp import gp_Pnt

    outer = BRepBuilderAPI_MakePolygon()
    for x, y in [(0.0, 0.0), (100.0, 0.0), (100.0, 100.0), (0.0, 100.0), (0.0, 0.0)]:
        outer.Add(gp_Pnt(x, y, 0.0))
    outer.Close()

    inner = BRepBuilderAPI_MakePolygon()
    for x, y in [(40.0, 40.0), (40.0, 60.0), (60.0, 60.0), (60.0, 40.0), (40.0, 40.0)]:
        inner.Add(gp_Pnt(x, y, 0.0))
    inner.Close()

    face_builder = BRepBuilderAPI_MakeFace(outer.Wire(), True)
    face_builder.Add(inner.Wire())
    _write_step(path, face_builder.Face())


def test_step_face_index_has_stable_signature_and_display_mapping(tmp_path: Path) -> None:
    module = _load_step_module()
    step_path = tmp_path / "box.step"
    _box_step(step_path)

    manifest = module.build_face_index(step_path)
    polydata = module.tessellated_polydata_with_face_ids(step_path)
    face_ids = polydata.GetCellData().GetArray("cad_face_id")

    assert manifest["schema"] == "base_casting_abb6700.step_cad_manifest"
    assert len(manifest["faces"]) == 6
    assert manifest["faces"][0]["signature"]["hash"]
    assert polydata.GetNumberOfCells() > 0
    assert face_ids is not None
    assert {face_ids.GetValue(index) for index in range(polydata.GetNumberOfCells())} <= set(range(1, 7))


def test_pick_manifest_outputs_only_selected_patches(tmp_path: Path) -> None:
    module = _load_step_module()
    step_path = tmp_path / "box.step"
    _box_step(step_path)
    manifest = module.build_face_index(step_path)

    updated = module.attach_pick_partitions(
        manifest,
        [[1]],
        {
            1: [
                [(0.0, 0.0), (50.0, 0.0), (50.0, 50.0), (0.0, 50.0)],
                [(50.0, 0.0), (100.0, 0.0), (100.0, 50.0), (50.0, 50.0)],
            ]
        },
    )

    assert updated["selected_cad_face_regions"] == [[1]]
    assert [patch["label"] for patch in updated["patches"]] == ["1_1", "1_2"]
    assert updated["manual_partitions"][0]["output_patch_count"] == 2


def test_cad_surface_sampling_uses_trimmed_face_and_skips_hole(tmp_path: Path) -> None:
    module = _load_step_module()
    step_path = tmp_path / "hole.step"
    _hole_face_step(step_path)
    manifest = module.build_face_index(step_path)
    manifest = module.attach_pick_partitions(
        manifest,
        [[1]],
        {1: [[(0.0, 0.0), (100.0, 0.0), (100.0, 100.0), (0.0, 100.0)]]},
    )

    samples = module.sample_manifest_patches(step_path, manifest, spacing=10.0, point_step=10.0)

    assert samples
    assert all(not (40.0 < sample.position_model[0] < 60.0 and 40.0 < sample.position_model[1] < 60.0) for sample in samples)
    assert all(abs(sample.normal_model[2]) > 0.99 for sample in samples)
    assert {sample.label for sample in samples} == {"1_1"}
