from __future__ import annotations

from collections.abc import Iterable
from pathlib import Path
import shutil


def score_max_abs_world_y(path) -> float:
    """Score a processing path by its farthest base-frame Y distance from zero."""
    if not path.waypoints:
        raise ValueError("Cannot score a path without processing waypoints.")
    return max(abs(waypoint.position_world[1]) for waypoint in path.waypoints)


def score_max_abs_world_x(path) -> float:
    """Score a processing path by its farthest base-frame X distance from zero."""
    if not path.waypoints:
        raise ValueError("Cannot score a path without processing waypoints.")
    return max(abs(waypoint.position_world[0]) for waypoint in path.waypoints)


def choose_best_by_region(candidate_rows: Iterable[dict]) -> list[dict]:
    """Choose one best candidate per region with a scope-specific policy.

    Ordinary and hole-aware candidates minimize the historical world-Y score,
    then use the corresponding world-X score only when world-Y is tied.
    Internally validated avoidance candidates maximize their sampled minimum
    robot clearance; TCP roll is not a ranking metric. Unresolved avoidance
    rows may remain in ``all_candidates.csv`` for RobotStudio diagnosis, but
    the configurable runner filters them before calling this selector for
    optimal output.
    """
    by_region: dict[int, list[dict]] = {}
    for row in candidate_rows:
        by_region.setdefault(int(row["region"]), []).append(row)

    best_rows: list[dict] = []
    for _region, rows in sorted(by_region.items()):
        avoidance_rows = [
            row
            for row in rows
            if bool(row.get("avoidance_selected"))
            and str(row.get("avoidance_status", "")) in {"baseline-validated", "alternative-validated"}
            and row.get("avoidance_min_clearance_mm") is not None
        ]
        if avoidance_rows:
            best_rows.append(
                max(
                    avoidance_rows,
                    key=lambda row: float(row["avoidance_min_clearance_mm"]),
                )
            )
            continue
        best_rows.append(
            min(
                rows,
                key=lambda row: (
                    float(row["score_max_abs_world_y"]),
                    float(row["score_max_abs_world_x"]),
                ),
            )
        )
    return best_rows


def copy_optimal_files(best_row: dict, optimal_dir: Path) -> dict[str, str]:
    region = int(best_row["region"])
    label = str(best_row.get("region_label", f"{region}")).strip() or f"{region}"
    destination = optimal_dir / label
    destination.mkdir(parents=True, exist_ok=True)

    copied: dict[str, str] = {}
    for key in ("txt",):
        source_value = best_row.get(key)
        if not source_value:
            continue
        source = Path(source_value)
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
        "avoidance_status": row.get("avoidance_status", "not-requested"),
        "tool_roll_deg": row.get("avoidance_roll_degrees", ""),
        "interference": row.get("avoidance_interference", "not-requested"),
    }


def best_table_row(row: dict) -> dict:
    return {
        "row_kind": "BEST",
        "model_x": row["model_x"],
        "model_y": row["model_y"],
        "model_z": row["model_z"],
        "angle_deg": row["angle_deg"],
        "covered_region": str(row.get("region_label", f"{int(row['region']):02d}")),
        "avoidance_status": row.get("avoidance_status", "not-requested"),
        "tool_roll_deg": row.get("avoidance_roll_degrees", ""),
        "interference": row.get("avoidance_interference", "not-requested"),
    }


def coverage_table_row(model_x: float, model_y: float, model_z: float, angle: int, region_ids: Iterable[int]) -> dict:
    return {
        "model_x": model_x,
        "model_y": model_y,
        "model_z": model_z,
        "angle_deg": angle,
        "covered_regions": covered_regions_text(region_ids),
    }
