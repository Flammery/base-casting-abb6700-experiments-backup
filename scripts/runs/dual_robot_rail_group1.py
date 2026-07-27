from __future__ import annotations

import json
import sys
from copy import deepcopy
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
SCRIPTS_DIR = SCRIPT_DIR.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import window_conf_export as base


MODEL_X = 4000.0
MODEL_Z = 500.0
Y_START = -1500
Y_STOP = 1500
Y_STEP = 200
Y_VALUES = [float(value) for value in range(Y_START, Y_STOP + 1, Y_STEP)]
ANGLES = [0, 180]
FEED_VARIANTS = [("long_side", base.RasterFeedDirection.LONG_SIDE)]

OUTDIR = (
    base.EXPERIMENT_DIR
    / "results"
    / "dual_robot_rail_group1_x4000_z500_yM1500_1500_step200_rz000_180_long"
)


def y_label(value: float) -> str:
    return f"y{base.module_coord_label(value)}"


def pose_label(model_y: float, angle: int) -> str:
    return f"{y_label(model_y)}_rz{angle:03d}"


def configure_base_export(model_y: float, outdir: Path) -> None:
    base.MODEL_X = MODEL_X
    base.MODEL_Y = float(model_y)
    base.MODEL_Z = MODEL_Z
    base.ANGLES = list(ANGLES)
    base.FEED_VARIANTS = list(FEED_VARIANTS)
    base.OUTDIR = outdir


def planner_settings(project) -> base.RasterPlannerSettings:
    return base.RasterPlannerSettings(
        spacing=base.SPACING,
        point_step=base.POINT_STEP,
        angle_degrees=0.0,
        bidirectional=True,
        feed_direction=base.RasterFeedDirection.LONG_SIDE,
        start_corner=base.StartCorner.LOWER_LEFT,
        tool_axis="-z",
        speed=100.0,
        zone="z1",
        tool_name=project.polishing_tool.name,
    )


def prefixed_row(model_y: float, angle: int, row: dict) -> dict:
    return {
        "model_x": MODEL_X,
        "model_y": model_y,
        "model_z": MODEL_Z,
        "pose_label": pose_label(model_y, angle),
        **row,
    }


def export_for_y(
    project,
    original_workpiece,
    polydata,
    regions: list[set[int]],
    vertices_by_region: dict[int, list[tuple[float, float, float]]],
    settings: base.RasterPlannerSettings,
    model_y: float,
) -> tuple[list[dict], list[dict], list[dict]]:
    y_outdir = OUTDIR / y_label(model_y)
    configure_base_export(model_y, y_outdir)
    y_outdir.mkdir(parents=True, exist_ok=True)
    for angle in ANGLES:
        (y_outdir / f"rz{angle:03d}").mkdir(parents=True, exist_ok=True)

    adjusted_project = deepcopy(project)
    adjusted_project.workpiece = base.placement_for(
        original_workpiece,
        original_workpiece.picked_origin,
        MODEL_X,
        model_y,
        MODEL_Z,
        0.0,
    )
    base.save_project_file(
        y_outdir
        / (
            f"input_project_x{int(MODEL_X)}_{y_label(model_y)}_"
            f"z{int(MODEL_Z)}_rz000_dual_robot_rail_group1.rsp.json"
        ),
        adjusted_project,
    )

    coverage_rows: list[dict] = []
    matrix_rows: list[dict] = []
    export_rows: list[dict] = []
    deferred_rows: list[dict] = []

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
        matrix_row = {"model_y": model_y, "angle_deg": angle}

        for region_index, region in enumerate(regions, 1):
            inside = base.region_inside_window(vertices_by_region[region_index], transform)
            matrix_row[f"region_{region_index:02d}"] = region_index if inside else ""
            if not inside:
                continue

            inside_regions.append(region_index)
            for variant_name, feed_variant in FEED_VARIANTS:
                path = base.plan_region_uv(polydata, placement, settings, region, feed_variant)
                if path.waypoints:
                    row = base.export_path_variant(project, placement, path, angle, region_index, variant_name)
                    export_rows.append(prefixed_row(model_y, angle, row))
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

        matrix_rows.append(matrix_row)
        coverage_rows.append(
            {
                "model_x": MODEL_X,
                "model_y": model_y,
                "model_z": MODEL_Z,
                "pose_label": pose_label(model_y, angle),
                "angle_deg": angle,
                "region_count": len(inside_regions),
                "region_ids": " ".join(f"{value:02d}" for value in inside_regions),
            }
        )

    base.write_csv(y_outdir / "angle_region_table.csv", matrix_rows)
    base.write_csv(y_outdir / "coverage_by_angle.csv", coverage_rows)
    base.write_csv(y_outdir / "exported_paths.csv", export_rows)
    base.write_csv(y_outdir / "deferred_paths.csv", deferred_rows)
    return export_rows, deferred_rows, coverage_rows


def main() -> None:
    OUTDIR.mkdir(parents=True, exist_ok=True)
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

    all_export_rows: list[dict] = []
    all_deferred_rows: list[dict] = []
    all_coverage_rows: list[dict] = []

    for model_y in Y_VALUES:
        export_rows, deferred_rows, coverage_rows = export_for_y(
            project,
            original_workpiece,
            polydata,
            regions,
            vertices_by_region,
            settings,
            model_y,
        )
        all_export_rows.extend(export_rows)
        all_deferred_rows.extend(deferred_rows)
        all_coverage_rows.extend(coverage_rows)

    base.write_csv(OUTDIR / "all_exported_paths.csv", all_export_rows)
    base.write_csv(OUTDIR / "all_deferred_paths.csv", all_deferred_rows)
    base.write_csv(OUTDIR / "all_coverage_by_pose.csv", all_coverage_rows)

    summary = {
        "input_project": str(base.PROJECT_PATH),
        "selected_region_count": len(regions),
        "output_dir": str(OUTDIR),
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
        "conf_y_negative": base.CONF_Y_NEGATIVE,
        "conf_y_nonnegative": base.CONF_Y_NONNEGATIVE,
        "tool_load_placeholder": base.RAPID_LOAD_PLACEHOLDER,
        "confl_off": True,
        "y_position_count": len(Y_VALUES),
        "pose_count": len(Y_VALUES) * len(ANGLES),
        "coverage": all_coverage_rows,
        "exported_count": len(all_export_rows),
        "deferred_count": len(all_deferred_rows),
    }
    (OUTDIR / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
