from pathlib import Path

import importlib.util
import json
from types import SimpleNamespace


def _load_script_module():
    script = Path(__file__).resolve().parents[1] / "scripts" / "window_conf_export.py"
    spec = importlib.util.spec_from_file_location("window_conf_export_test", script)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_resolve_default_project_path_prefers_latest_script_test(tmp_path) -> None:
    module = _load_script_module()
    experiment_dir = tmp_path / "experiment"
    root = tmp_path / "root"
    latest = experiment_dir / "inputs" / "latest_script_test.rsp.json"
    latest.parent.mkdir(parents=True)
    latest.write_text("{}", encoding="utf-8")

    assert module.resolve_default_project_path(experiment_dir, root) == latest


def test_resolve_default_project_path_prefers_partitioned_input(tmp_path) -> None:
    module = _load_script_module()
    experiment_dir = tmp_path / "experiment"
    root = tmp_path / "root"
    latest = experiment_dir / "inputs" / "latest_script_test.rsp.json"
    partitioned = experiment_dir / "inputs" / "latest_partitioned.rsp.json"
    latest.parent.mkdir(parents=True)
    latest.write_text("{}", encoding="utf-8")
    partitioned.write_text("{}", encoding="utf-8")

    assert module.resolve_default_project_path(experiment_dir, root) == partitioned


def test_resolve_default_project_path_falls_back_to_project_file(tmp_path) -> None:
    module = _load_script_module()
    experiment_dir = tmp_path / "experiment"
    root = tmp_path / "root"

    assert module.resolve_default_project_path(experiment_dir, root) == root / "project" / "test-0704-selected.rsp.json"


def test_rapid_module_name_is_short_region_only() -> None:
    module = _load_script_module()

    assert module.rapid_module_name(7) == "MODULE_R07"


def test_pose_file_label_uses_pose_and_region_only() -> None:
    module = _load_script_module()

    assert module.pose_file_label(3500, -1000, 440, 0, 7) == "x3500_yM1000_z440_rz000_R07"


def test_safe_region_label_keeps_manual_partition_name() -> None:
    module = _load_script_module()

    assert module.safe_region_label("1_1") == "1_1"
    assert module.safe_region_label("13 2") == "13_2"


def test_rapid_experiment_metadata_records_full_model_pose() -> None:
    module = _load_script_module()
    placement = SimpleNamespace(
        model_x=3700.0,
        model_y=-1200.0,
        model_z=440.0,
        model_rx=0.0,
        model_ry=0.0,
        model_rz=270.0,
        name="wobj1",
        file_path=r"C:\cad\part.stp",
        picked_origin=(-2011.833, -2309.704, 549.128),
        wobj_rx=0.0,
        wobj_ry=0.0,
    )

    metadata = module.rapid_experiment_metadata(placement, "1_1")

    assert metadata["schema"] == "robot_studio_qt.experiment_installation"
    assert metadata["model_x"] == 3700.0
    assert metadata["model_y"] == -1200.0
    assert metadata["model_z"] == 440.0
    assert metadata["model_rz"] == 270.0
    assert metadata["picked_origin"] == [-2011.833, -2309.704, 549.128]
    assert metadata["region_label"] == "1_1"


def test_rapid_experiment_metadata_comment_is_robotware_6_safe_ascii() -> None:
    module = _load_script_module()
    placement = SimpleNamespace(
        model_x=3500.0,
        model_y=1600.0,
        model_z=440.0,
        model_rx=0.0,
        model_ry=0.0,
        model_rz=0.0,
        name="wobj1",
        file_path=r"C:\cad\底座毛坯三维.stp",
        picked_origin=(-2011.833, -2309.704, 549.128),
        wobj_rx=0.0,
        wobj_ry=0.0,
    )

    comment = module.rapid_experiment_metadata_comment(placement, "2")
    prefix = "! RSP_EXPERIMENT_META_V1 "

    assert comment.encode("ascii").decode("ascii") == comment
    assert "底座" not in comment
    assert json.loads(comment[len(prefix) :])["workpiece_file_path"] == placement.file_path


def test_version_1_manifest_uses_materialized_project_regions(tmp_path) -> None:
    module = _load_script_module()
    project_path = tmp_path / "partitioned.rsp.json"
    project_path.write_text("{}", encoding="utf-8")
    manifest_path = module.partition_manifest_path_for(project_path)
    manifest_path.write_text(
        json.dumps(
            {
                "schema": "base_casting_abb6700.manual_region_partition_manifest",
                "version": 1,
                "records": [{"original_region": 1, "patches": [{"label": "1.1"}, {"label": "1.2"}]}],
            }
        ),
        encoding="utf-8",
    )

    regions = module.manual_clip_regions(project_path, [{1, 2}, {8, 9}])

    assert [item["face_ids"] for item in regions] == [{1, 2}, {8, 9}]
    assert [item["label"] for item in regions] == ["1", "2"]


def test_automatic_partition_manifest_restores_patch_labels_and_source_regions(tmp_path) -> None:
    module = _load_script_module()
    project_path = tmp_path / "partitioned.rsp.json"
    project_path.write_text("{}", encoding="utf-8")
    module.partition_manifest_path_for(project_path).write_text(
        json.dumps(
            {
                "schema": "base_casting_abb6700.region_partition_manifest",
                "version": 2,
                "records": [
                    {"original_region": 1, "patches": [{"label": "1.1"}, {"label": "1.2"}]},
                    {"original_region": 2, "patches": [{"label": "2"}]},
                ],
            }
        ),
        encoding="utf-8",
    )

    planning = module.manual_clip_regions(project_path, [{1}, {2}, {3}])

    assert [item["label"] for item in planning] == ["1.1", "1.2", "2"]
    assert [item["source_region"] for item in planning] == [1, 1, 2]
    module.validate_selectors(module.parse_region_selectors("1-2,2"), planning)


def test_hole_gap_starts_a_new_motion_segment() -> None:
    module = _load_script_module()
    normal = (0.0, 0.0, 1.0)
    samples = [
        (module.encoded_raster_line_id(0, 3), 0, 1, (0.0, 0.0, 0.0), normal),
        (module.encoded_raster_line_id(0, 3), 1, 1, (10.0, 0.0, 0.0), normal),
        (module.encoded_raster_line_id(0, 3), 2, 1, (100.0, 0.0, 0.0), normal),
    ]

    split = module.split_discontinuous_raster_segments(samples, point_step=10.0)

    assert module.raster_segment_id(split[0][0]) == 0
    assert module.raster_segment_id(split[1][0]) == 0
    assert module.raster_segment_id(split[2][0]) == 1
