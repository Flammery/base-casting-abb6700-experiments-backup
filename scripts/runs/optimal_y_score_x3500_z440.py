from __future__ import annotations

import json
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
SCRIPTS_DIR = SCRIPT_DIR.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import window_conf_export as base
import optimal_y_selection as optimal


MODEL_X = 3500.0
MODEL_Z = 440.0
Y_START = -1900
Y_STOP = 1900
Y_STEP = 100
Y_VALUES = [float(value) for value in range(Y_START, Y_STOP + 1, Y_STEP)]
ANGLES = [0, 180]
FEED_VARIANTS = [("long_side", base.RasterFeedDirection.LONG_SIDE)]

OUTDIR = (
    base.EXPERIMENT_DIR
    / "results"
    / "dual_robot_rail_optimal_y_score_x3500_z440_yM1900_1900_step100_rz000_180_long"
)
CANDIDATES_DIR = OUTDIR / "candidates"
OPTIMAL_DIR = OUTDIR / "optimal_paths"


def y_label(value: float) -> str:
    return f"y{base.module_coord_label(value)}"


def pose_label(model_y: float, angle: int) -> str:
    return f"{y_label(model_y)}_rz{angle:03d}"


def configure_base_export(model_y: float) -> Path:
    y_outdir = CANDIDATES_DIR / y_label(model_y)
    base.MODEL_X = MODEL_X
    base.MODEL_Y = float(model_y)
    base.MODEL_Z = MODEL_Z
    base.ANGLES = list(ANGLES)
    base.FEED_VARIANTS = list(FEED_VARIANTS)
    base.OUTDIR = y_outdir
    return y_outdir


def planner_settings(project) -> base.RasterPlannerSettings:
    return base.RasterPlannerSettings(
        spacing=base.SPACING,
        point_step=base.POINT_STEP,
        angle_degrees=0.0,
        boundary_margin=base.BOUNDARY_MARGIN,
        bidirectional=True,
        feed_direction=base.RasterFeedDirection.LONG_SIDE,
        start_corner=base.StartCorner.LOWER_LEFT,
        tool_axis="-z",
        speed=100.0,
        zone="z1",
        tool_name=project.polishing_tool.name,
    )


def enrich_candidate_row(model_y: float, angle: int, row: dict, path: base.PathResult) -> dict:
    return {
        "model_x": MODEL_X,
        "model_y": model_y,
        "model_z": MODEL_Z,
        "pose_label": pose_label(model_y, angle),
        "score_max_abs_world_y": optimal.score_max_abs_world_y(path),
        **row,
    }


def best_record_row(best_row: dict) -> dict:
    copied = optimal.copy_optimal_files(best_row, OPTIMAL_DIR)
    row = dict(best_row)
    row["source_module"] = best_row["module"]
    row["source_txt"] = best_row["txt"]
    row["source_points_csv"] = best_row["points_csv"]
    row["module"] = copied["module"]
    row["txt"] = copied["txt"]
    row["points_csv"] = copied["points_csv"]
    return row


def write_csv_safely(path: Path, rows: list[dict]) -> Path:
    try:
        base.write_csv(path, rows)
        return path
    except PermissionError:
        fallback = path.with_name(f"{path.stem}_updated{path.suffix}")
        base.write_csv(fallback, rows)
        return fallback


def main() -> None:
    OUTDIR.mkdir(parents=True, exist_ok=True)
    CANDIDATES_DIR.mkdir(parents=True, exist_ok=True)
    OPTIMAL_DIR.mkdir(parents=True, exist_ok=True)

    project = base.load_project_file(base.PROJECT_PATH)
    regions = [set(region) for region in project.selected_path_face_regions]
    if not regions:
        raise RuntimeError(f"No selected_path_face_regions in {base.PROJECT_PATH}")
    if project.polishing_tool.mass_kg <= 0.0:
        project.polishing_tool.mass_kg = base.TOOL_LOAD_MASS_KG

    original_workpiece = project.workpiece.clone()
    importer = base.CadImportService().import_model(project.workpiece.file_path)
    reader = base.create_mesh_reader(importer.display_path, importer.display_format)
    reader.SetFileName(str(importer.display_path))
    reader.Update()
    polydata = reader.GetOutput()

    settings = planner_settings(project)
    vertices_by_region = base.region_vertices_by_id(polydata, regions)

    candidate_rows: list[dict] = []
    deferred_rows: list[dict] = []
    coverage_rows: list[dict] = []

    for model_y in Y_VALUES:
        y_outdir = configure_base_export(model_y)
        y_outdir.mkdir(parents=True, exist_ok=True)
        for angle in ANGLES:
            (y_outdir / f"rz{angle:03d}").mkdir(parents=True, exist_ok=True)

        adjusted_project = base.load_project_file(base.PROJECT_PATH)
        adjusted_project.workpiece = base.placement_for(
            original_workpiece,
            original_workpiece.picked_origin,
            MODEL_X,
            model_y,
            MODEL_Z,
            0.0,
        )
        base.save_project_file(
            y_outdir / f"{base.pose_file_label(MODEL_X, model_y, MODEL_Z, 0)}.rsp.json",
            adjusted_project,
        )

        for angle in ANGLES:
            placement = base.placement_for(
                original_workpiece,
                original_workpiece.picked_origin,
                MODEL_X,
                model_y,
                MODEL_Z,
                float(angle),
            )
            transform = base.WorkpieceTransform(placement)
            inside_regions: list[int] = []

            for region_index, region in enumerate(regions, 1):
                if not base.region_inside_window(vertices_by_region[region_index], transform):
                    continue

                inside_regions.append(region_index)
                for variant_name, feed_variant in FEED_VARIANTS:
                    path = base.plan_region_uv(polydata, placement, settings, region, feed_variant)
                    if path.waypoints:
                        row = base.export_path_variant(project, placement, path, angle, region_index, variant_name)
                        candidate = enrich_candidate_row(model_y, angle, row, path)
                        candidate_rows.append(candidate)
                    else:
                        deferred_rows.append(
                            {
                                "model_x": MODEL_X,
                                "model_y": model_y,
                                "model_z": MODEL_Z,
                                "pose_label": pose_label(model_y, angle),
                                "angle_deg": angle,
                                "region": region_index,
                                "feed_variant": variant_name,
                                "reason": path.message,
                            }
                        )

            coverage_rows.append(
                optimal.coverage_table_row(MODEL_X, model_y, MODEL_Z, angle, inside_regions)
            )

    best_records = [best_record_row(row) for row in optimal.choose_best_by_region(candidate_rows)]
    candidate_table_rows = [optimal.candidate_table_row(row) for row in candidate_rows]
    best_table_rows = [optimal.best_table_row(row) for row in best_records]

    candidate_table = write_csv_safely(OUTDIR / "all_candidates.csv", [*candidate_table_rows, *best_table_rows])
    optimal_table = write_csv_safely(OUTDIR / "optimal_selection.csv", best_table_rows)
    deferred_table = write_csv_safely(OUTDIR / "deferred_paths.csv", deferred_rows)
    coverage_table = write_csv_safely(OUTDIR / "coverage_by_pose.csv", coverage_rows)

    summary = {
        "input_project": str(base.PROJECT_PATH),
        "selected_region_count": len(regions),
        "output_dir": str(OUTDIR),
        "candidate_dir": str(CANDIDATES_DIR),
        "optimal_dir": str(OPTIMAL_DIR),
        "model_x": MODEL_X,
        "model_y_values": Y_VALUES,
        "model_y_start": Y_START,
        "model_y_stop": Y_STOP,
        "y_step": Y_STEP,
        "model_z": MODEL_Z,
        "angles_deg": ANGLES,
        "window_base_xy": {"x": base.WINDOW_X, "y": base.WINDOW_Y},
        "window_shape": base.WINDOW_SHAPE,
        "spacing_mm": base.SPACING,
        "point_step_mm": base.POINT_STEP,
        "orientation_mode": base.ORIENTATION_MODE,
        "axis_mode": base.AXIS_MODE,
        "feed_variants": [variant for variant, _feed in FEED_VARIANTS],
        "selection_metric": "score_max_abs_world_y",
        "selection_metric_definition": "max(abs(world_y)) over processing waypoints only",
        "tie_breakers": ["abs(model_y)", "angle_deg", "region"],
        "conf_y_negative": base.CONF_Y_NEGATIVE,
        "conf_y_nonnegative": base.CONF_Y_NONNEGATIVE,
        "tool_load_placeholder": base.RAPID_LOAD_PLACEHOLDER,
        "confl_off": True,
        "y_position_count": len(Y_VALUES),
        "pose_count": len(Y_VALUES) * len(ANGLES),
        "coverage_table": str(coverage_table),
        "candidate_table": str(candidate_table),
        "optimal_table": str(optimal_table),
        "deferred_table": str(deferred_table),
        "candidate_count": len(candidate_rows),
        "optimal_region_count": len(best_records),
        "deferred_count": len(deferred_rows),
    }
    (OUTDIR / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
