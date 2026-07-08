from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
SCRIPTS_DIR = SCRIPT_DIR.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import optimal_y_score_x3500_z440 as runner


X_VALUES = [3600.0, 3700.0]
OUTDIR = (
    runner.base.EXPERIMENT_DIR
    / "results"
    / "dual_robot_rail_optimal_y_score_x3600_3700_z440_yM1900_1900_step100_rz000_180_long"
)


def x_label(value: float) -> str:
    return f"x{runner.base.module_coord_label(value)}"


def read_csv_rows(path: Path) -> list[dict]:
    if not path.exists() or path.stat().st_size == 0:
        return []
    with path.open(encoding="utf-8-sig", newline="") as handle:
        return list(csv.DictReader(handle))


def configure_runner(model_x: float) -> Path:
    x_outdir = OUTDIR / x_label(model_x)
    runner.MODEL_X = model_x
    runner.OUTDIR = x_outdir
    runner.CANDIDATES_DIR = x_outdir / "candidates"
    runner.OPTIMAL_DIR = x_outdir / "optimal_paths"
    return x_outdir


def main() -> None:
    OUTDIR.mkdir(parents=True, exist_ok=True)

    all_candidates: list[dict] = []
    all_optimal: list[dict] = []
    all_coverage: list[dict] = []
    summaries: list[dict] = []

    for model_x in X_VALUES:
        x_outdir = configure_runner(model_x)
        runner.main()

        summary_path = x_outdir / "summary.json"
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        summaries.append(summary)

        all_candidates.extend(read_csv_rows(Path(summary["candidate_table"])))
        all_optimal.extend(read_csv_rows(Path(summary["optimal_table"])))
        all_coverage.extend(read_csv_rows(Path(summary["coverage_table"])))

    candidate_table = runner.write_csv_safely(OUTDIR / "all_candidates.csv", all_candidates)
    optimal_table = runner.write_csv_safely(OUTDIR / "optimal_selection.csv", all_optimal)
    coverage_table = runner.write_csv_safely(OUTDIR / "coverage_by_pose.csv", all_coverage)

    summary = {
        "output_dir": str(OUTDIR),
        "model_x_values": X_VALUES,
        "model_z": runner.MODEL_Z,
        "model_y_values": runner.Y_VALUES,
        "model_y_start": runner.Y_START,
        "model_y_stop": runner.Y_STOP,
        "y_step": runner.Y_STEP,
        "angles_deg": runner.ANGLES,
        "feed_variants": [variant for variant, _feed in runner.FEED_VARIANTS],
        "selection_metric": "score_max_abs_world_y",
        "selection_metric_definition": "max(abs(world_y)) over processing waypoints only",
        "candidate_table": str(candidate_table),
        "optimal_table": str(optimal_table),
        "coverage_table": str(coverage_table),
        "candidate_count": sum(int(item["candidate_count"]) for item in summaries),
        "optimal_region_count": sum(int(item["optimal_region_count"]) for item in summaries),
        "pose_count": len(X_VALUES) * len(runner.Y_VALUES) * len(runner.ANGLES),
        "runs": summaries,
    }
    (OUTDIR / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
