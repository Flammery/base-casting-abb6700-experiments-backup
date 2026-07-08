from __future__ import annotations

from collections.abc import Iterable
from pathlib import Path
import shutil


def score_max_abs_world_y(path) -> float:
    """Score a processing path by its farthest base-frame Y distance from zero."""
    if not path.waypoints:
        raise ValueError("Cannot score a path without processing waypoints.")
    return max(abs(waypoint.position_world[1]) for waypoint in path.waypoints)


def choose_best_by_region(candidate_rows: Iterable[dict]) -> list[dict]:
    """Choose one best candidate per region using max(abs(world_y)) only.

    Tie breakers are deterministic output policy, not extra optimization metrics.
    """
    by_region: dict[int, list[dict]] = {}
    for row in candidate_rows:
        by_region.setdefault(int(row["region"]), []).append(row)

    best_rows: list[dict] = []
    for _region, rows in sorted(by_region.items()):
        best_rows.append(
            min(
                rows,
                key=lambda row: (
                    float(row["score_max_abs_world_y"]),
                    abs(float(row["model_y"])),
                    int(row["angle_deg"]),
                    int(row["region"]),
                ),
            )
        )
    return best_rows


def copy_optimal_files(best_row: dict, optimal_dir: Path) -> dict[str, str]:
    region = int(best_row["region"])
    destination = optimal_dir / f"region{region:02d}"
    destination.mkdir(parents=True, exist_ok=True)

    copied: dict[str, str] = {}
    for key in ("module", "txt", "points_csv"):
        source = Path(best_row[key])
        target = destination / source.name
        shutil.copy2(source, target)
        copied[key] = str(target)
    return copied


def covered_regions_text(region_ids: Iterable[int]) -> str:
    return " ".join(f"{value:02d}" for value in sorted(region_ids))


def candidate_table_row(row: dict) -> dict:
    return {
        "row_kind": "CANDIDATE",
        "model_x": row["model_x"],
        "model_y": row["model_y"],
        "model_z": row["model_z"],
        "angle_deg": row["angle_deg"],
        "covered_region": str(row.get("region_label", f"{int(row['region']):02d}")),
    }


def best_table_row(row: dict) -> dict:
    return {
        "row_kind": "BEST",
        "model_x": row["model_x"],
        "model_y": row["model_y"],
        "model_z": row["model_z"],
        "angle_deg": row["angle_deg"],
        "covered_region": str(row.get("region_label", f"{int(row['region']):02d}")),
    }


def coverage_table_row(model_x: float, model_y: float, model_z: float, angle: int, region_ids: Iterable[int]) -> dict:
    return {
        "model_x": model_x,
        "model_y": model_y,
        "model_z": model_z,
        "angle_deg": angle,
        "covered_regions": covered_regions_text(region_ids),
    }
