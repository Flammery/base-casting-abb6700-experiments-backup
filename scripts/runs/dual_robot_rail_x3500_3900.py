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

import dual_robot_rail_group1 as group


X_START = 3500
X_STOP = 3900
X_STEP = 100
X_VALUES = [float(value) for value in range(X_START, X_STOP + 1, X_STEP)]

OUTDIR = (
    group.base.EXPERIMENT_DIR
    / "results"
    / "dual_robot_rail_x3500_3900_step100_z500_yM1500_1500_step200_rz000_180_long"
)


def x_label(value: float) -> str:
    return f"x{group.base.module_coord_label(value)}"


def configure_group_for_x(model_x: float) -> Path:
    x_outdir = OUTDIR / x_label(model_x)
    group.MODEL_X = float(model_x)
    group.OUTDIR = x_outdir
    return x_outdir


def main() -> None:
    OUTDIR.mkdir(parents=True, exist_ok=True)
    project = group.base.load_project_file(group.base.PROJECT_PATH)
    regions = [set(region) for region in project.selected_path_face_regions]
    if not regions:
        raise RuntimeError(f"No selected_path_face_regions in {group.base.PROJECT_PATH}")
    if project.polishing_tool.mass_kg <= 0.0:
        project.polishing_tool.mass_kg = group.base.TOOL_LOAD_MASS_KG

    original_workpiece = project.workpiece.clone()
    importer = group.base.CadImportService().import_model(project.workpiece.file_path)
    reader = group.base.create_mesh_reader(importer.display_path, importer.display_format)
    reader.SetFileName(str(importer.display_path))
    reader.Update()
    polydata = reader.GetOutput()

    settings = group.planner_settings(project)
    vertices_by_region = group.base.region_vertices_by_id(polydata, regions)

    all_export_rows: list[dict] = []
    all_deferred_rows: list[dict] = []
    all_coverage_rows: list[dict] = []

    for model_x in X_VALUES:
        x_outdir = configure_group_for_x(model_x)
        x_outdir.mkdir(parents=True, exist_ok=True)

        x_export_rows: list[dict] = []
        x_deferred_rows: list[dict] = []
        x_coverage_rows: list[dict] = []

        for model_y in group.Y_VALUES:
            export_rows, deferred_rows, coverage_rows = group.export_for_y(
                project,
                original_workpiece,
                polydata,
                regions,
                vertices_by_region,
                settings,
                model_y,
            )
            x_export_rows.extend(export_rows)
            x_deferred_rows.extend(deferred_rows)
            x_coverage_rows.extend(coverage_rows)

        group.base.write_csv(x_outdir / "all_exported_paths.csv", x_export_rows)
        group.base.write_csv(x_outdir / "all_deferred_paths.csv", x_deferred_rows)
        group.base.write_csv(x_outdir / "all_coverage_by_pose.csv", x_coverage_rows)

        all_export_rows.extend(x_export_rows)
        all_deferred_rows.extend(x_deferred_rows)
        all_coverage_rows.extend(x_coverage_rows)

    group.base.write_csv(OUTDIR / "all_exported_paths.csv", all_export_rows)
    group.base.write_csv(OUTDIR / "all_deferred_paths.csv", all_deferred_rows)
    group.base.write_csv(OUTDIR / "all_coverage_by_pose.csv", all_coverage_rows)

    summary = {
        "input_project": str(group.base.PROJECT_PATH),
        "selected_region_count": len(regions),
        "output_dir": str(OUTDIR),
        "model_x_values": X_VALUES,
        "model_x_start": X_START,
        "model_x_stop": X_STOP,
        "x_step": X_STEP,
        "model_z": group.MODEL_Z,
        "model_y_values": group.Y_VALUES,
        "model_y_start": group.Y_START,
        "model_y_stop": group.Y_STOP,
        "y_step": group.Y_STEP,
        "angles_deg": group.ANGLES,
        "window_base_xy": {"x": group.base.WINDOW_X, "y": group.base.WINDOW_Y},
        "window_shape": group.base.WINDOW_SHAPE,
        "spacing_mm": group.base.SPACING,
        "point_step_mm": group.base.POINT_STEP,
        "orientation_mode": group.base.ORIENTATION_MODE,
        "axis_mode": group.base.AXIS_MODE,
        "feed_variants": [variant for variant, _feed in group.FEED_VARIANTS],
        "conf_y_negative": group.base.CONF_Y_NEGATIVE,
        "conf_y_nonnegative": group.base.CONF_Y_NONNEGATIVE,
        "tool_load_placeholder": group.base.RAPID_LOAD_PLACEHOLDER,
        "confl_off": True,
        "x_position_count": len(X_VALUES),
        "y_position_count": len(group.Y_VALUES),
        "pose_count": len(X_VALUES) * len(group.Y_VALUES) * len(group.ANGLES),
        "coverage": all_coverage_rows,
        "exported_count": len(all_export_rows),
        "deferred_count": len(all_deferred_rows),
    }
    (OUTDIR / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
