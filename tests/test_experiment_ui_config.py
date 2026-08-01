from __future__ import annotations

import argparse
import importlib.util
from pathlib import Path
import sys

import pytest


def _load_ui_config_module():
    script = Path(__file__).resolve().parents[1] / "ui" / "experiment_config.py"
    spec = importlib.util.spec_from_file_location("experiment_config_test", script)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _load_runner_module():
    script = Path(__file__).resolve().parents[1] / "scripts" / "configurable_experiment_runner.py"
    spec = importlib.util.spec_from_file_location("configurable_experiment_runner_test", script)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_historical_runner_entrypoint_reexports_generic_runner() -> None:
    scripts_dir = Path(__file__).resolve().parents[1] / "scripts"
    if str(scripts_dir) not in sys.path:
        sys.path.insert(0, str(scripts_dir))

    import configurable_experiment_runner as generic_runner
    import optimal_y_score_configurable as historical_runner

    assert historical_runner.main is generic_runner.main
    assert historical_runner.run_from_args is generic_runner.run_from_args


def test_parse_region_text_keeps_order_and_deduplicates() -> None:
    module = _load_ui_config_module()

    assert module.parse_region_text("2,3,6,10,6") == [2, 3, 6, 10]


def test_parse_region_text_allows_empty() -> None:
    module = _load_ui_config_module()

    assert module.parse_region_text("") == []


def test_parse_avoidance_regions_supports_regions_and_patch_spellings() -> None:
    module = _load_ui_config_module()

    assert module.parse_region_selectors("1-1, 1_2, 2.3, 4, 1-1") == ["1_1", "1_2", "2_3", "4"]
    assert module.parse_region_selectors("1-1，2、3-2；4") == ["1_1", "2", "3_2", "4"]
    assert module.parse_region_selectors("") == []


def test_parse_avoidance_regions_rejects_invalid_or_zero_labels() -> None:
    module = _load_ui_config_module()

    with pytest.raises(ValueError, match="无效避障区域"):
        module.parse_region_selectors("1-a")
    with pytest.raises(ValueError, match="正整数"):
        module.parse_region_selectors("0,1-0")


def test_validate_regions_rejects_out_of_range() -> None:
    module = _load_ui_config_module()

    with pytest.raises(ValueError, match="超出当前输入范围"):
        module.validate_regions([2, 20], 19)


def test_turntable_angle_arguments_accept_single_and_multiple_angles() -> None:
    module = _load_ui_config_module()

    assert module.parse_turntable_angle_text("270") == [270]
    assert module.parse_turntable_angle_text("271") == [271]
    assert module.parse_turntable_angle_text("0,180") == [0, 180]
    assert module.parse_turntable_angle_text("360,-30,330") == [0, 330]
    expected_range = list(range(0, 360, 30))
    assert module.parse_turntable_angle_text("0-30-330") == expected_range
    assert module.parse_turntable_angle_text("0:30:330") == expected_range
    assert module.parse_turntable_angle_text("0-30-360") == expected_range
    assert module.turntable_angle_args("0,30,180") == [
        "--experiment-mode", "turntable", "--angles", "0,30,180"
    ]
    with pytest.raises(ValueError, match="转台角度"):
        module.parse_turntable_angle_text("0,abc")
    with pytest.raises(ValueError, match="stop"):
        module.parse_turntable_angle_text("330-30-0")


def test_saved_avoidance_settings_must_match_current_partition(tmp_path) -> None:
    module = _load_ui_config_module()
    project_path = tmp_path / "input.rsp.json"
    planning_regions = [
        {"label": "6_1", "source_region": 6},
        {"label": "6_2", "source_region": 6},
    ]
    payload = {
        "input_project": str(project_path),
        "selectors": ["1_1"],
        "regions": [{"region_label": "1_1"}],
    }

    with pytest.raises(ValueError, match="避障区域不存在"):
        module.validated_avoidance_settings(payload, project_path, planning_regions)

    payload["selectors"] = ["6_1"]
    payload["regions"] = [{"region_label": "6_1"}]
    assert module.validated_avoidance_settings(payload, project_path, planning_regions) == (
        ["6_1"],
        ["6-1"],
    )


def test_saved_avoidance_settings_must_belong_to_current_project(tmp_path) -> None:
    module = _load_ui_config_module()
    payload = {
        "input_project": str(tmp_path / "old.rsp.json"),
        "selectors": [],
        "regions": [],
    }

    with pytest.raises(ValueError, match="当前项目不一致"):
        module.validated_avoidance_settings(payload, tmp_path / "current.rsp.json", [])


def test_parse_coordinate_text_fixed_and_range() -> None:
    module = _load_ui_config_module()

    assert module.parse_coordinate_text("3700") == ([3700.0], False)
    assert module.parse_coordinate_text("3500,100,3700") == ([3500.0, 3600.0, 3700.0], True)


def test_scan_axis_allows_only_single_axis_range() -> None:
    module = _load_ui_config_module()

    assert module.scan_axis_for_coordinates("3500,100,3700", "0", "440") == "x"
    assert module.scan_axis_for_coordinates("3700", "-1900,100,1900", "440") == "y"
    assert module.scan_axis_for_coordinates("3700", "0", "400,20,440") == "z"
    with pytest.raises(ValueError, match="只能有一个范围"):
        module.scan_axis_for_coordinates("3500,100,3700", "-1900,100,1900", "440")


def test_runner_command_uses_equals_for_negative_coordinate_range(tmp_path) -> None:
    module = _load_ui_config_module()

    command = module.runner_command(
        "python",
        tmp_path / "input.rsp.json",
        "3500",
        "-1900,100,1900",
        "440",
        "0,180",
        "1400,2600;-1050,1050",
        "6",
    )

    assert "--model-y=-1900,100,1900" in command
    assert "--model-y" not in command
    assert "--boundary-margin=6" in command


def test_runner_command_adds_hole_aware_planner_only_when_requested(tmp_path) -> None:
    module = _load_ui_config_module()
    common = (
        "python",
        tmp_path / "input.rsp.json",
        "3500",
        "0",
        "440",
        "0,180",
        "1500,2500;-1050,1050",
        "6",
    )

    legacy = module.runner_command(*common)
    hole_aware = module.runner_command(*common, planner="hole-aware")

    assert "--planner" not in legacy
    assert "--planner" in hole_aware
    assert hole_aware[hole_aware.index("--planner") + 1] == "hole-aware"

    automatic = module.runner_command(*common, planner="auto")
    assert automatic[automatic.index("--planner") + 1] == "auto"


def test_runner_command_adds_normalized_avoidance_regions_only_when_requested(tmp_path) -> None:
    module = _load_ui_config_module()
    common = (
        "python",
        tmp_path / "input.rsp.json",
        "3500",
        "0",
        "440",
        "0,180",
        "1500,2500;-1050,1050",
        "6",
    )

    ordinary = module.runner_command(*common, planner="auto")
    avoidance = module.runner_command(*common, planner="auto", avoidance_regions="1-1,2")

    assert "--avoidance-regions" not in ordinary
    assert avoidance[avoidance.index("--avoidance-regions") + 1] == "1_1,2"


def test_runner_command_adds_robot_configuration_when_selected(tmp_path) -> None:
    module = _load_ui_config_module()
    config_path = tmp_path / "ABB 6700 Style.rsc.json"
    config_path.write_text("{}", encoding="utf-8")

    command = module.runner_command(
        "python",
        tmp_path / "input.rsp.json",
        "3700",
        "0",
        "440",
        "270",
        "1500,2500;-1050,1050",
        "6",
        "auto",
        "1",
        config_path,
    )

    assert command[command.index("--robot-config") + 1] == str(config_path)


def test_runner_command_adds_avoidance_volume_settings_when_selected(tmp_path) -> None:
    module = _load_ui_config_module()
    settings_path = tmp_path / "input_avoidance.json"
    settings_path.write_text("{}", encoding="utf-8")

    command = module.runner_command(
        "python",
        tmp_path / "input.rsp.json",
        "3700",
        "0",
        "440",
        "270",
        "1500,2500;-1050,1050",
        "6",
        "auto",
        "1-2",
        avoidance_settings_path=settings_path,
    )

    assert command[command.index("--avoidance-settings") + 1] == str(settings_path)


def test_runner_command_does_not_auto_enable_existing_avoidance_sidecar(tmp_path) -> None:
    project_path = tmp_path / "input.rsp.json"
    sidecar_path = tmp_path / "input_avoidance.json"
    sidecar_path.write_text("{}", encoding="utf-8")
    module = _load_ui_config_module()

    command = module.runner_command(
        "python",
        project_path,
        "3700",
        "0",
        "440",
        "270",
        "1500,2500;-1050,1050",
        "6",
        "auto",
    )

    assert sidecar_path.exists()
    assert "--avoidance-settings" not in command
    assert "--avoidance-regions" not in command


def test_configurable_runner_defaults_to_auto_planner() -> None:
    module = _load_runner_module()

    assert module.build_parser().parse_args([]).planner == "auto"


def test_parse_boundary_margin_text_defaults_and_rejects_negative() -> None:
    module = _load_ui_config_module()

    assert module.parse_boundary_margin_text("") == 6.0
    assert module.parse_boundary_margin_text("8.5") == 8.5
    with pytest.raises(ValueError, match="边缘余量"):
        module.parse_boundary_margin_text("-1")


def test_parse_custom_window_text_two_or_three_axes() -> None:
    module = _load_ui_config_module()

    assert module.parse_custom_window_text("1000,2000;-1050,1050") == {
        "x": (1000.0, 2000.0),
        "y": (-1050.0, 1050.0),
        "z": None,
    }
    assert module.parse_custom_window_text("1000,2000;-1050,1050;300,600")["z"] == (300.0, 600.0)
    assert module.parse_custom_window_text("") == {"x": None, "y": None, "z": None}


def test_configurable_runner_angle_range_excludes_duplicate_360() -> None:
    module = _load_runner_module()

    assert module.parse_angles_range("0,360,10")[-1] == 350
    assert len(module.parse_angles_range("0,360,10")) == 36


def test_current_avoidance_report_keeps_only_review_fields() -> None:
    from types import SimpleNamespace

    module = _load_runner_module()
    trial = SimpleNamespace(
        roll_degrees=-15.0,
        validation_status="validated-clear",
        interference="not-detected",
        minimum_clearance_mm=12.5,
        max_joint_jump_degrees=8.0,
        message="sampled waypoints validated",
    )

    row = module.compact_avoidance_report_row(
        3400.0,
        -1800.0,
        440.0,
        271,
        "1_1",
        "long_side",
        "alternative-validated",
        True,
        trial=trial,
    )

    assert list(row) == [
        "model_x",
        "model_y",
        "model_z",
        "angle_deg",
        "region_label",
        "feed_variant",
        "tool_roll_deg",
        "selected",
        "status",
        "interference",
        "minimum_clearance_mm",
        "max_joint_jump_deg",
        "reason",
    ]
    assert row["tool_roll_deg"] == -15.0
    assert row["interference"] == "not-detected"


def test_configurable_runner_coordinate_specs_and_scan_axis() -> None:
    module = _load_runner_module()

    args = argparse.Namespace(
        model_x="3500,100,3700",
        model_y="0",
        model_z="440",
        y_start=-1900,
        y_stop=1900,
        y_step=100,
    )
    x_spec, y_spec, z_spec = module.coordinate_specs_from_args(args)

    assert x_spec.values == [3500.0, 3600.0, 3700.0]
    assert y_spec.values == [0.0]
    assert z_spec.values == [440.0]
    assert module.determine_scan_axis(x_spec, y_spec, z_spec) == "x"


def test_configurable_runner_rejects_negative_boundary_margin() -> None:
    module = _load_runner_module()

    x_spec = module.parse_coordinate_spec("3700", 3700.0)
    y_spec = module.parse_coordinate_spec("0", 0.0)
    z_spec = module.parse_coordinate_spec("440", 440.0)
    args = argparse.Namespace(
        boundary_margin=-1.0,
        window_mode="unlimited",
    )

    with pytest.raises(ValueError, match="boundary-margin"):
        module.validate_args(args, x_spec, y_spec, z_spec)


def test_configurable_runner_rejects_xyz_grid() -> None:
    module = _load_runner_module()

    x_spec = module.parse_coordinate_spec("3500,100,3700", 3700.0)
    y_spec = module.parse_coordinate_spec("-100,100,100", 0.0)
    z_spec = module.parse_coordinate_spec("440", 440.0)

    with pytest.raises(ValueError, match="Only one"):
        module.determine_scan_axis(x_spec, y_spec, z_spec)


def test_configurable_runner_default_output_dir_mentions_scan() -> None:
    module = _load_runner_module()

    x_spec = module.parse_coordinate_spec("3700", 3700.0)
    y_spec = module.parse_coordinate_spec("-1900,100,1900", 0.0)
    z_spec = module.parse_coordinate_spec("440", 440.0)
    outdir = module.default_output_dir(
        x_spec,
        y_spec,
        z_spec,
        [0, 180],
        "custom",
        "rail",
        date_suffix="20260713",
        run_number=3,
    )

    assert outdir.name == "x3700-ym1900,100,1900-z440-angle0,180-20260713-03"


def test_configurable_runner_compacts_regular_angle_range() -> None:
    module = _load_runner_module()

    assert module.compact_angles_label(list(range(0, 360, 30))) == "angle0,30,330"


def test_configurable_runner_daily_number_avoids_existing_results(tmp_path) -> None:
    module = _load_runner_module()
    (tmp_path / "first-20260713-01").mkdir()
    (tmp_path / "different-parameters-20260713-03").mkdir()
    (tmp_path / "old-style-0713").mkdir()
    x_spec = module.parse_coordinate_spec("3500", 3700.0)
    y_spec = module.parse_coordinate_spec("-1900,100,1900", 0.0)
    z_spec = module.parse_coordinate_spec("440", 440.0)

    outdir = module.default_output_dir(
        x_spec,
        y_spec,
        z_spec,
        [270],
        "custom",
        "turntable",
        date_suffix="20260713",
        results_root=tmp_path,
    )

    assert outdir.name == "x3500-ym1900,100,1900-z440-angle270-20260713-04"


def test_configurable_runner_default_output_path_stays_short_enough() -> None:
    module = _load_runner_module()

    x_spec = module.parse_coordinate_spec("3700", 3700.0)
    y_spec = module.parse_coordinate_spec("-1900,100,1900", 0.0)
    z_spec = module.parse_coordinate_spec("440", 440.0)
    outdir = module.default_output_dir(
        x_spec,
        y_spec,
        z_spec,
        [0, 180],
        "custom",
        "rail",
        date_suffix="20260713",
        run_number=1,
    )
    sample = (
        outdir
        / "candidates"
        / "x3700_yM1900_z440"
        / "rz000"
        / "region08"
        / "long_side"
        / "R08_LONG_SIDE_X3700_YM1900_Z440_RZ000_points.csv"
    )

    assert len(str(sample)) < 240
