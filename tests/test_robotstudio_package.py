from __future__ import annotations

import csv
import importlib.util
import json
from pathlib import Path
import sys
import zipfile


def _load_module():
    script = Path(__file__).resolve().parents[1] / "scripts" / "robotstudio_package.py"
    spec = importlib.util.spec_from_file_location("robotstudio_package_test", script)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


RAPID = """MODULE MODULE_R03
    PERS tooldata tool_disk:=[TRUE,[[1,2,3],[1,0,0,0]],[1,[0,0,0],[1,0,0,0],0,0,0]];
    PERS wobjdata wobj1:=[FALSE,TRUE,"",[[100,200,300],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];
    CONST robtarget p0001:=[[0,0,0],[1,0,0,0],[0,0,0,1],[9E9,9E9,9E9,9E9,9E9,9E9]];
    PROC main()
        MoveJ p0001,v100,z10,tool_disk\\WObj:=wobj1;
    ENDPROC
ENDMODULE
"""


def test_station_filename_supports_plain_and_partition_labels() -> None:
    module = _load_module()
    partitioned = module.OptimalRecord("1_3", 3600, -800, 440, 0, Path("x"))
    plain = module.OptimalRecord("2", 3600, -600, 440, 180, Path("x"))

    assert module.station_filename(partitioned) == "3600_m800_440_rz0_1-3.rsstn"
    assert module.station_filename(plain) == "3600_m600_440_rz180_2.rsstn"


def test_split_moves_exported_tool_and_wobj_to_calibdata() -> None:
    module = _load_module()
    path_text, calib_text = module.split_rapid_modules(RAPID, "1_3", "CalibData", "VALIDATE")

    assert "MODULE VALIDATE_R1_3" in path_text
    assert "PERS tooldata" not in path_text
    assert "PERS wobjdata" not in path_text
    assert "tool_disk\\WObj:=wobj1" in path_text
    assert "MODULE CalibData" in calib_text
    assert "PERS tooldata tool_disk" in calib_text
    assert "PERS wobjdata wobj1" in calib_text


def test_split_escapes_experiment_metadata_for_robotware_6() -> None:
    module = _load_module()
    rapid = RAPID.replace(
        "MODULE MODULE_R03\n",
        'MODULE MODULE_R03\n    ! RSP_EXPERIMENT_META_V1 {"workpiece_file_path":"C:\\\\cad\\\\底座毛坯.stp"}\n',
    )

    path_text, _ = module.split_rapid_modules(rapid, "3", "CalibData", "VALIDATE")
    metadata_line = next(line for line in path_text.splitlines() if "RSP_EXPERIMENT_META_V1" in line)
    payload = metadata_line.split("RSP_EXPERIMENT_META_V1", 1)[1].strip()

    assert path_text.encode("ascii").decode("ascii") == path_text
    assert "底座" not in metadata_line
    assert json.loads(payload)["workpiece_file_path"] == r"C:\cad\底座毛坯.stp"


def test_build_package_writes_station_beside_each_optimal_path(tmp_path: Path) -> None:
    module = _load_module()
    result_dir = tmp_path / "result"
    for label in ("1_1", "2"):
        folder = result_dir / "optimal_paths" / label
        folder.mkdir(parents=True)
        (folder / f"{label}.txt").write_text(RAPID, encoding="utf-8")

    with (result_dir / "optimal_selection.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=["row_kind", "model_x", "model_y", "model_z", "angle_deg", "covered_region"])
        writer.writeheader()
        writer.writerow({"row_kind": "BEST", "model_x": 3600, "model_y": -800, "model_z": 440, "angle_deg": 0, "covered_region": "1_1"})
        writer.writerow({"row_kind": "BEST", "model_x": 3600, "model_y": -600, "model_z": 440, "angle_deg": 180, "covered_region": "2"})

    template = tmp_path / "template.rsstn"
    pim = b'''<?xml version="1.0" encoding="utf-8"?><PIMDocument xmlns="urn:abb-robotics-pim"><Objects><ComponentInstance><Name Value="scene-model-not-wobj1"/><Transform><RowX><X Value="1"/><Y Value="0"/><Z Value="0"/></RowX><RowY><X Value="0"/><Y Value="1"/><Z Value="0"/></RowY><RowZ><X Value="0"/><Y Value="0"/><Z Value="1"/></RowZ><RowT><X Value="0"/><Y Value="0"/><Z Value="0"/></RowT></Transform></ComponentInstance></Objects></PIMDocument>'''
    with zipfile.ZipFile(template, "w") as archive:
        archive.writestr("PIM.xml", pim)
    config = tmp_path / "config.json"
    config.write_text(
        json.dumps(
            {
                "robotstudio_exe": "RobotStudio.exe",
                "sdk_bin": "sdk",
                "template_station": str(template),
                "controller_task": "T_ROB1",
                "calib_module_name": "CalibData",
                "workpiece_component_name": "scene-model-not-wobj1",
                "path_module_prefix": "VALIDATE",
            }
        ),
        encoding="utf-8",
    )

    manifest_path = module.build_package(result_dir, config)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))

    assert manifest["workpiece_component_name"] == "scene-model-not-wobj1"
    assert manifest["jobs"][0]["model_installation"]["y"] == -800
    assert manifest["jobs"][0]["rapid_coordinates_are_independent"] is True
    assert Path(manifest["jobs"][0]["output_station"]).parent.name == "1_1"
    assert Path(manifest["jobs"][0]["station_job"]).parent.name == "1_1"
    assert Path(manifest["jobs"][1]["output_station"]).name == "3600_m600_440_rz180_2.rsstn"
    with zipfile.ZipFile(manifest["jobs"][1]["output_station"]) as archive:
        root = module.ET.fromstring(archive.read("PIM.xml"))
    ns = "urn:abb-robotics-pim"
    component = next(root.iter(f"{{{ns}}}ComponentInstance"))
    transform = component.find(f"{{{ns}}}Transform")
    assert transform is not None
    assert float(transform.find(f"{{{ns}}}RowT/{{{ns}}}Y").get("Value")) == -0.6
    assert abs(float(transform.find(f"{{{ns}}}RowX/{{{ns}}}X").get("Value")) + 1.0) < 1e-12
    assert abs(float(transform.find(f"{{{ns}}}RowY/{{{ns}}}Y").get("Value")) + 1.0) < 1e-12


def test_queue_manifest_writes_bridge_request_without_launch(tmp_path: Path, monkeypatch) -> None:
    module = _load_module()
    bridge_root = tmp_path / "local"
    monkeypatch.setenv("LOCALAPPDATA", str(bridge_root))
    manifest_path = tmp_path / "robotstudio_jobs.json"
    manifest_path.write_text(json.dumps({"robotstudio_exe": "unused.exe"}), encoding="utf-8")

    pending = module.queue_manifest(manifest_path, launch=False)
    payload = json.loads(pending.read_text(encoding="utf-8"))

    assert pending == bridge_root / "ABB6700RobotStudioBridge" / "pending.json"
    assert Path(payload["manifest_path"]) == manifest_path.resolve()
