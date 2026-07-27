from __future__ import annotations

from pathlib import Path
import sys


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from optimal_y_selection import choose_best_by_region  # noqa: E402


def test_ordinary_region_keeps_world_y_policy() -> None:
    rows = [
        {"region": 1, "score_max_abs_world_y": 100.0, "model_y": -1000.0, "angle_deg": 0},
        {"region": 1, "score_max_abs_world_y": 20.0, "model_y": 900.0, "angle_deg": 180},
    ]

    assert choose_best_by_region(rows)[0] is rows[1]


def test_avoidance_region_ignores_base_y_and_minimizes_roll_after_clearance_filter() -> None:
    rows = [
        {
            "region": 1,
            "avoidance_selected": True,
            "avoidance_status": "alternative-validated",
            "avoidance_roll_degrees": 15.0,
            "avoidance_min_clearance_mm": 8.0,
            "score_max_abs_world_y": 1000.0,
            "model_y": -1900.0,
            "angle_deg": 270,
        },
        {
            "region": 1,
            "avoidance_selected": True,
            "avoidance_status": "alternative-validated",
            "avoidance_roll_degrees": 30.0,
            "avoidance_min_clearance_mm": 40.0,
            "score_max_abs_world_y": 1.0,
            "model_y": 0.0,
            "angle_deg": 270,
        },
    ]

    assert choose_best_by_region(rows)[0] is rows[0]


def test_avoidance_equal_roll_uses_larger_clearance_without_world_y() -> None:
    rows = [
        {
            "region": 1,
            "avoidance_selected": True,
            "avoidance_status": "alternative-validated",
            "avoidance_roll_degrees": -15.0,
            "avoidance_min_clearance_mm": 7.0,
            "score_max_abs_world_y": 1.0,
            "model_y": 0.0,
            "angle_deg": 270,
        },
        {
            "region": 1,
            "avoidance_selected": True,
            "avoidance_status": "alternative-validated",
            "avoidance_roll_degrees": 15.0,
            "avoidance_min_clearance_mm": 9.0,
            "score_max_abs_world_y": 1000.0,
            "model_y": -1900.0,
            "angle_deg": 270,
        },
    ]

    assert choose_best_by_region(rows)[0] is rows[1]


def test_not_evaluated_avoidance_region_uses_world_y_policy() -> None:
    rows = [
        {
            "region": 1,
            "avoidance_selected": True,
            "avoidance_status": "not-evaluated",
            "avoidance_roll_degrees": 0.0,
            "avoidance_min_clearance_mm": None,
            "score_max_abs_world_y": 800.0,
            "model_y": -1800.0,
            "angle_deg": 271,
        },
        {
            "region": 1,
            "avoidance_selected": True,
            "avoidance_status": "not-evaluated",
            "avoidance_roll_degrees": 0.0,
            "avoidance_min_clearance_mm": None,
            "score_max_abs_world_y": 120.0,
            "model_y": -600.0,
            "angle_deg": 271,
        },
    ]

    assert choose_best_by_region(rows)[0] is rows[1]
