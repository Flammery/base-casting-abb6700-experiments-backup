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
    script = Path(__file__).resolve().parents[1] / "scripts" / "runs" / "optimal_y_score_configurable.py"
    spec = importlib.util.spec_from_file_location("optimal_y_score_configurable_test", script)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_parse_region_text_keeps_order_and_deduplicates() -> None:
    module = _load_ui_config_module()

    assert module.parse_region_text("2,3,6,10,6") == [2, 3, 6, 10]


def test_parse_region_text_allows_empty() -> None:
    module = _load_ui_config_module()

    assert module.parse_region_text("") == []


def test_validate_regions_rejects_out_of_range() -> None:
    module = _load_ui_config_module()

    with pytest.raises(ValueError, match="超出当前输入范围"):
        module.validate_regions([2, 20], 19)


def test_angle_preset_arguments() -> None:
    module = _load_ui_config_module()

    assert module.angle_args("地轨 0,180") == ["--experiment-mode", "rail", "--angles", "0,180"]
    assert module.angle_args("转台 0..360 step10") == [
        "--experiment-mode",
        "turntable",
        "--angles-range",
        "0,360,10",
    ]


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
        "地轨 0,180",
        "1400,2600;-1050,1050",
        "6",
    )

    assert "--model-y=-1900,100,1900" in command
    assert "--model-y" not in command
    assert "--boundary-margin=6" in command


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
    outdir = module.default_output_dir(x_spec, y_spec, z_spec, [0, 180], "custom", "rail")

    assert outdir.name.startswith("optY_")
    assert "x3700" in outdir.name
    assert "yM1900_1900_step100" in outdir.name
    assert "rz000_180" not in outdir.name
    assert "cwin" not in outdir.name


def test_configurable_runner_default_output_path_stays_short_enough() -> None:
    module = _load_runner_module()

    x_spec = module.parse_coordinate_spec("3700", 3700.0)
    y_spec = module.parse_coordinate_spec("-1900,100,1900", 0.0)
    z_spec = module.parse_coordinate_spec("440", 440.0)
    outdir = module.default_output_dir(x_spec, y_spec, z_spec, [0, 180], "custom", "rail")
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
