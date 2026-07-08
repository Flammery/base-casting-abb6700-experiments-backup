from __future__ import annotations

import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
SCRIPTS_DIR = SCRIPT_DIR.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import optimal_y_score_x3500_z440 as runner


MODEL_X = 3900.0
OUTDIR = (
    runner.base.EXPERIMENT_DIR
    / "results"
    / "dual_robot_rail_optimal_y_score_x3900_z440_yM1900_1900_step100_rz000_180_long"
)


def main() -> None:
    runner.MODEL_X = MODEL_X
    runner.OUTDIR = OUTDIR
    runner.CANDIDATES_DIR = OUTDIR / "candidates"
    runner.OPTIMAL_DIR = OUTDIR / "optimal_paths"
    runner.main()


if __name__ == "__main__":
    main()
