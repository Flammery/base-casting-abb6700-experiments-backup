from pathlib import Path

import importlib.util


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
