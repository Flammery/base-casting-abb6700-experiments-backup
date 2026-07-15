from __future__ import annotations

import argparse
from datetime import datetime
import json
import sys
from dataclasses import dataclass
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
SCRIPTS_DIR = SCRIPT_DIR.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import optimal_y_score_x3500_z440 as runner


@dataclass(frozen=True)
class CoordinateSpec:
    raw: str
    values: list[float]

    @property
    def is_range(self) -> bool:
        return len(self.values) > 1


def parse_coordinate_spec(raw: str | None, default: float) -> CoordinateSpec:
    text = str(default) if raw is None or not raw.strip() else raw.strip()
    parts = [part.strip() for part in text.split(",") if part.strip()]
    if len(parts) == 1:
        return CoordinateSpec(text, [float(parts[0])])
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("coordinate expects value or start,step,stop")

    start = float(parts[0])
    step = float(parts[1])
    stop = float(parts[2])
    if step <= 0:
        raise argparse.ArgumentTypeError("coordinate range step must be positive")
    if stop < start:
        raise argparse.ArgumentTypeError("coordinate range stop must be >= start")

    values: list[float] = []
    current = start
    epsilon = abs(step) * 1e-9
    while current <= stop + epsilon:
        values.append(round(current, 10))
        current += step
    return CoordinateSpec(text, values)


def parse_angles(raw: str) -> list[int]:
    values: list[int] = []
    for item in raw.split(","):
        item = item.strip()
        if not item:
            continue
        values.append(int(item))
    if not values:
        raise argparse.ArgumentTypeError("at least one angle is required")
    return values


def parse_angles_range(raw: str) -> list[int]:
    parts = [int(item.strip()) for item in raw.split(",") if item.strip()]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("--angles-range expects start,stop,step")
    start, stop, step = parts
    if step <= 0:
        raise argparse.ArgumentTypeError("angle range step must be positive")
    if stop <= start:
        raise argparse.ArgumentTypeError("angle range stop must be greater than start")
    return list(range(start, stop, step))


def module_label(value: float) -> str:
    return runner.base.module_coord_label(value)


def angles_label(angles: list[int]) -> str:
    if len(angles) > 6:
        return f"{angles[0]:03d}_{angles[-1]:03d}_n{len(angles)}"
    return "_".join(f"{angle:03d}" for angle in angles)


def coord_label(axis: str, spec: CoordinateSpec) -> str:
    if len(spec.values) == 1:
        return f"{axis}{module_label(spec.values[0])}"
    step = spec.values[1] - spec.values[0]
    return f"{axis}{module_label(spec.values[0])}_{module_label(spec.values[-1])}_step{module_label(step)}"


def pose_label(model_x: float, model_y: float, model_z: float) -> str:
    return f"x{module_label(model_x)}_y{module_label(model_y)}_z{module_label(model_z)}"


def determine_scan_axis(x_spec: CoordinateSpec, y_spec: CoordinateSpec, z_spec: CoordinateSpec) -> str:
    ranged = [
        axis
        for axis, spec in (("x", x_spec), ("y", y_spec), ("z", z_spec))
        if spec.is_range
    ]
    if len(ranged) > 1:
        raise ValueError("Only one of X/Y/Z may be a range expression.")
    return ranged[0] if ranged else "none"


def default_output_dir(
    x_spec: CoordinateSpec,
    y_spec: CoordinateSpec,
    z_spec: CoordinateSpec,
    angles: list[int],
    window_mode: str,
    experiment_mode: str,
    date_suffix: str | None = None,
    planner: str = "legacy",
) -> Path:
    mode_suffixes = {
        "rail": "rail",
        "turntable": "turn",
    }
    mode_suffix = mode_suffixes.get(experiment_mode, experiment_mode)
    date_suffix = date_suffix or datetime.now().strftime("%m%d")
    planner_suffix = "_hole_aware" if planner == "hole-aware" else ""
    return (
        runner.base.EXPERIMENT_DIR
        / "results"
        / (
            f"{coord_label('x', x_spec)}_"
            f"{coord_label('y', y_spec)}_"
            f"{coord_label('z', z_spec)}_"
            f"{mode_suffix}_{date_suffix}{planner_suffix}"
        )
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Configurable ABB6700 single-axis Optimal-Y runner.")
    parser.add_argument("--project", type=Path, default=runner.base.resolve_default_project_path())
    parser.add_argument("--model-x", default="3700")
    parser.add_argument("--model-y")
    parser.add_argument("--model-z", default="440")
    parser.add_argument("--boundary-margin", type=float, default=runner.base.BOUNDARY_MARGIN)
    parser.add_argument("--planner", choices=["legacy", "auto", "hole-aware"], default="auto")
    parser.add_argument("--y-start", type=int, default=-1900, help="Compatibility fallback when --model-y is omitted.")
    parser.add_argument("--y-stop", type=int, default=1900, help="Compatibility fallback when --model-y is omitted.")
    parser.add_argument("--y-step", type=int, default=100, help="Compatibility fallback when --model-y is omitted.")
    angle_group = parser.add_mutually_exclusive_group()
    angle_group.add_argument("--angles", type=parse_angles)
    angle_group.add_argument("--angles-range", type=parse_angles_range)
    parser.add_argument("--experiment-mode", choices=["rail", "turntable"], default="rail")
    parser.add_argument("--window-mode", choices=["default", "unlimited", "custom"], default="default")
    parser.add_argument("--x-min", type=float)
    parser.add_argument("--x-max", type=float)
    parser.add_argument("--y-min", type=float)
    parser.add_argument("--y-max", type=float)
    parser.add_argument("--z-min", type=float)
    parser.add_argument("--z-max", type=float)
    parser.add_argument("--output-dir", type=Path)
    return parser


def coordinate_specs_from_args(args: argparse.Namespace) -> tuple[CoordinateSpec, CoordinateSpec, CoordinateSpec]:
    model_y = args.model_y
    if model_y is None:
        if args.y_step <= 0:
            raise ValueError("--y-step must be positive")
        if args.y_stop < args.y_start:
            raise ValueError("--y-stop must be greater than or equal to --y-start")
        model_y = f"{args.y_start},{args.y_step},{args.y_stop}"
    return (
        parse_coordinate_spec(args.model_x, 3700.0),
        parse_coordinate_spec(model_y, 0.0),
        parse_coordinate_spec(args.model_z, 440.0),
    )


def angles_from_args(args: argparse.Namespace) -> list[int]:
    if args.angles_range is not None:
        return args.angles_range
    if args.angles is not None:
        return args.angles
    if args.experiment_mode == "turntable":
        return parse_angles_range("0,360,10")
    return [0, 180]


def validate_args(args: argparse.Namespace, x_spec: CoordinateSpec, y_spec: CoordinateSpec, z_spec: CoordinateSpec) -> None:
    determine_scan_axis(x_spec, y_spec, z_spec)
    if args.boundary_margin < 0.0:
        raise ValueError("--boundary-margin must be >= 0")
    if args.window_mode == "custom":
        pairs = (
            ("x", args.x_min, args.x_max),
            ("y", args.y_min, args.y_max),
            ("z", args.z_min, args.z_max),
        )
        for axis, minimum, maximum in pairs:
            if (minimum is None) != (maximum is None):
                raise ValueError(f"custom window {axis} needs both min and max, or neither")
            if minimum is not None and maximum is not None and maximum < minimum:
                raise ValueError(f"custom window {axis} max must be >= min")


def bounds_from_args(args: argparse.Namespace) -> dict[str, tuple[float, float] | None]:
    if args.window_mode == "default":
        return {
            "x": tuple(runner.base.WINDOW_X),
            "y": tuple(runner.base.WINDOW_Y),
            "z": None,
        }
    if args.window_mode == "unlimited":
        return {"x": None, "y": None, "z": None}
    return {
        "x": None if args.x_min is None else (float(args.x_min), float(args.x_max)),
        "y": None if args.y_min is None else (float(args.y_min), float(args.y_max)),
        "z": None if args.z_min is None else (float(args.z_min), float(args.z_max)),
    }


def configure_window(mode: str, bounds: dict[str, tuple[float, float] | None]) -> None:
    if bounds["x"] is not None:
        runner.base.WINDOW_X = bounds["x"]
    if bounds["y"] is not None:
        runner.base.WINDOW_Y = bounds["y"]

    if mode == "default":
        return

    def region_inside_window(vertices_model, transform) -> bool:
        if not vertices_model:
            return False
        if mode == "unlimited":
            return True
        for point_model in vertices_model:
            point_world = transform.model_point_to_world(point_model)
            for axis_index, axis in enumerate(("x", "y", "z")):
                limit = bounds[axis]
                if limit is None:
                    continue
                if not (limit[0] <= point_world[axis_index] <= limit[1]):
                    return False
        return True

    runner.base.region_inside_window = region_inside_window


def iter_poses(
    x_spec: CoordinateSpec,
    y_spec: CoordinateSpec,
    z_spec: CoordinateSpec,
) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x in x_spec.values for y in y_spec.values for z in z_spec.values]


def enrich_candidate_row(model_x: float, model_y: float, model_z: float, angle: int, row: dict, path) -> dict:
    return {
        "model_x": model_x,
        "model_y": model_y,
        "model_z": model_z,
        "pose_label": f"{pose_label(model_x, model_y, model_z)}_rz{angle:03d}",
        "score_max_abs_world_y": runner.optimal.score_max_abs_world_y(path),
        **row,
    }


def write_csv_safely(path: Path, rows: list[dict]) -> Path:
    try:
        runner.base.write_csv(path, rows)
        return path
    except PermissionError:
        fallback = path.with_name(f"{path.stem}_updated{path.suffix}")
        runner.base.write_csv(fallback, rows)
        return fallback


def run_optimal_scan(
    args: argparse.Namespace,
    x_spec: CoordinateSpec,
    y_spec: CoordinateSpec,
    z_spec: CoordinateSpec,
    angles: list[int],
    bounds: dict[str, tuple[float, float] | None],
    outdir: Path,
) -> None:
    outdir.mkdir(parents=True, exist_ok=True)
    candidates_dir = outdir / "candidates"
    optimal_dir = outdir / "optimal_paths"
    candidates_dir.mkdir(parents=True, exist_ok=True)
    optimal_dir.mkdir(parents=True, exist_ok=True)

    runner.base.PROJECT_PATH = args.project
    runner.base.ANGLES = list(angles)
    runner.base.FEED_VARIANTS = list(runner.FEED_VARIANTS)
    runner.base.BOUNDARY_MARGIN = float(args.boundary_margin)
    runner.OPTIMAL_DIR = optimal_dir
    configure_window(args.window_mode, bounds)

    project = runner.base.load_project_file(args.project)
    regions = [set(region) for region in project.selected_path_face_regions]
    if not regions:
        raise RuntimeError(f"No selected_path_face_regions in {args.project}")
    planning_regions = runner.base.manual_clip_regions(args.project, regions)
    if project.polishing_tool.mass_kg <= 0.0:
        project.polishing_tool.mass_kg = runner.base.TOOL_LOAD_MASS_KG

    original_workpiece = project.workpiece.clone()
    importer = runner.base.CadImportService().import_model(project.workpiece.file_path)
    reader = runner.base.create_mesh_reader(importer.display_path, importer.display_format)
    reader.SetFileName(str(importer.display_path))
    reader.Update()
    polydata = reader.GetOutput()

    settings = runner.planner_settings(project)
    vertices_by_region = {
        index: runner.base.clip_region_vertices(polydata, item["face_ids"], item.get("clip_polygon"), item.get("exclude_polygons"), item.get("raster_chart"))
        for index, item in enumerate(planning_regions, 1)
    }

    candidate_rows: list[dict] = []
    deferred_rows: list[dict] = []
    coverage_rows: list[dict] = []
    auto_hole_aware_count = 0
    auto_raster_count = 0
    auto_planner_reasons: dict[str, int] = {}

    for model_x, model_y, model_z in iter_poses(x_spec, y_spec, z_spec):
        pose_dir = candidates_dir / pose_label(model_x, model_y, model_z)
        pose_dir.mkdir(parents=True, exist_ok=True)
        for angle in angles:
            (pose_dir / f"rz{angle:03d}").mkdir(parents=True, exist_ok=True)

        runner.base.MODEL_X = float(model_x)
        runner.base.MODEL_Y = float(model_y)
        runner.base.MODEL_Z = float(model_z)
        runner.base.OUTDIR = pose_dir

        adjusted_project = runner.base.load_project_file(args.project)
        adjusted_project.workpiece = runner.base.placement_for(
            original_workpiece,
            original_workpiece.picked_origin,
            model_x,
            model_y,
            model_z,
            0.0,
        )
        runner.base.save_project_file(
            pose_dir
            / f"{runner.base.pose_file_label(model_x, model_y, model_z, 0)}.rsp.json",
            adjusted_project,
        )

        for angle in angles:
            placement = runner.base.placement_for(
                original_workpiece,
                original_workpiece.picked_origin,
                model_x,
                model_y,
                model_z,
                float(angle),
            )
            transform = runner.base.WorkpieceTransform(placement)
            inside_regions: list[int] = []

            for region_index, planning_region in enumerate(planning_regions, 1):
                if not runner.base.region_inside_window(vertices_by_region[region_index], transform):
                    continue

                inside_regions.append(region_index)
                for variant_name, feed_variant in runner.FEED_VARIANTS:
                    planner_arguments = (
                        polydata,
                        placement,
                        settings,
                        planning_region["face_ids"],
                        feed_variant,
                        planning_region.get("clip_polygon"),
                        planning_region.get("exclude_polygons"),
                        planning_region.get("raster_chart"),
                    )
                    if args.planner == "auto":
                        path, use_hole_aware, planner_reason = runner.base.plan_region_uv_auto(*planner_arguments)
                    elif args.planner == "hole-aware":
                        use_hole_aware = True
                        planner_reason = "forced-cell-transfer"
                        path = runner.base.plan_region_uv_hole_aware(
                            *planner_arguments
                        )
                    else:
                        use_hole_aware = False
                        planner_reason = "forced-regular-raster"
                        path = runner.base.plan_region_uv(*planner_arguments)
                    if args.planner == "auto":
                        auto_planner_reasons[planner_reason] = auto_planner_reasons.get(planner_reason, 0) + 1
                        if use_hole_aware:
                            auto_hole_aware_count += 1
                        else:
                            auto_raster_count += 1
                    if path.waypoints:
                        row = runner.base.export_path_variant(
                            project,
                            placement,
                            path,
                            angle,
                            region_index,
                            variant_name,
                            planning_region["label"],
                            hole_aware=use_hole_aware,
                            planner_label=("auto-cell-transfer" if use_hole_aware else "auto-raster") if args.planner == "auto" else args.planner,
                        )
                        row["planner_reason"] = planner_reason
                        row["source_region"] = planning_region["source_region"]
                        candidate_rows.append(enrich_candidate_row(model_x, model_y, model_z, angle, row, path))
                    else:
                        deferred_rows.append(
                            {
                                "model_x": model_x,
                                "model_y": model_y,
                                "model_z": model_z,
                                "pose_label": f"{pose_label(model_x, model_y, model_z)}_rz{angle:03d}",
                                "angle_deg": angle,
                                "region": region_index,
                                "region_label": planning_region["label"],
                                "feed_variant": variant_name,
                                "planner_reason": planner_reason,
                                "reason": path.message,
                            }
                        )

            coverage_rows.append(runner.optimal.coverage_table_row(model_x, model_y, model_z, angle, inside_regions))

    best_records = [runner.best_record_row(row) for row in runner.optimal.choose_best_by_region(candidate_rows)]
    candidate_table_rows = [runner.optimal.candidate_table_row(row) for row in candidate_rows]
    best_table_rows = [runner.optimal.best_table_row(row) for row in best_records]

    candidate_table = write_csv_safely(outdir / "all_candidates.csv", [*candidate_table_rows, *best_table_rows])
    optimal_table = write_csv_safely(outdir / "optimal_selection.csv", best_table_rows)
    deferred_table = write_csv_safely(outdir / "deferred_paths.csv", deferred_rows)
    coverage_table = write_csv_safely(outdir / "coverage_by_pose.csv", coverage_rows)
    optimal_records_path = outdir / "optimal_records.json"
    optimal_records_path.write_text(
        json.dumps(
            {
                "schema": "base_casting_abb6700.optimal_records",
                "version": 1,
                "records": best_records,
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )

    scan_axis = determine_scan_axis(x_spec, y_spec, z_spec)
    summary = {
        "input_project": str(args.project),
        "selected_region_count": len(regions),
        "planning_region_count": len(planning_regions),
        "output_dir": str(outdir),
        "candidate_dir": str(candidates_dir),
        "optimal_dir": str(optimal_dir),
        "configurable_runner": True,
        "planner": args.planner,
        "auto_hole_aware_path_count": auto_hole_aware_count,
        "auto_raster_path_count": auto_raster_count,
        "auto_planner_reason_counts": auto_planner_reasons,
        "experiment_mode": args.experiment_mode,
        "scan_axis": scan_axis,
        "model_x_values": x_spec.values,
        "model_y_values": y_spec.values,
        "model_z_values": z_spec.values,
        "model_x": x_spec.values[0] if len(x_spec.values) == 1 else None,
        "model_y": y_spec.values[0] if len(y_spec.values) == 1 else None,
        "model_z": z_spec.values[0] if len(z_spec.values) == 1 else None,
        "model_y_start": y_spec.values[0],
        "model_y_stop": y_spec.values[-1],
        "y_step": (y_spec.values[1] - y_spec.values[0]) if len(y_spec.values) > 1 else 0,
        "angles_deg": angles,
        "window_mode": args.window_mode,
        "window_limits": {
            axis: None if limit is None else {"min": limit[0], "max": limit[1]}
            for axis, limit in bounds.items()
        },
        "window_base_xy": {"x": runner.base.WINDOW_X, "y": runner.base.WINDOW_Y},
        "window_shape": runner.base.WINDOW_SHAPE,
        "spacing_mm": runner.base.SPACING,
        "point_step_mm": runner.base.POINT_STEP,
        "boundary_margin_mm": runner.base.BOUNDARY_MARGIN,
        "orientation_mode": runner.base.ORIENTATION_MODE,
        "axis_mode": runner.base.AXIS_MODE,
        "feed_variants": [variant for variant, _feed in runner.FEED_VARIANTS],
        "selection_metric": "score_max_abs_world_y",
        "selection_metric_definition": "max(abs(world_y)) over processing waypoints only",
        "tie_breakers": ["abs(model_y)", "angle_deg", "region"],
        "conf_y_negative": runner.base.CONF_Y_NEGATIVE,
        "conf_y_nonnegative": runner.base.CONF_Y_NONNEGATIVE,
        "tool_load_placeholder": runner.base.RAPID_LOAD_PLACEHOLDER,
        "confl_off": True,
        "pose_count": len(iter_poses(x_spec, y_spec, z_spec)) * len(angles),
        "coverage_table": str(coverage_table),
        "candidate_table": str(candidate_table),
        "optimal_table": str(optimal_table),
        "optimal_records": str(optimal_records_path),
        "deferred_table": str(deferred_table),
        "candidate_count": len(candidate_rows),
        "optimal_region_count": len(best_records),
        "deferred_count": len(deferred_rows),
    }
    (outdir / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")


def run_from_args(args: argparse.Namespace) -> Path:
    x_spec, y_spec, z_spec = coordinate_specs_from_args(args)
    validate_args(args, x_spec, y_spec, z_spec)
    angles = angles_from_args(args)
    bounds = bounds_from_args(args)
    outdir = args.output_dir or default_output_dir(
        x_spec,
        y_spec,
        z_spec,
        angles,
        args.window_mode,
        args.experiment_mode,
        planner=args.planner,
    )
    run_optimal_scan(args, x_spec, y_spec, z_spec, angles, bounds, outdir)
    return outdir


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    outdir = run_from_args(args)
    summary_path = outdir / "summary.json"
    print(f"OUTPUT_DIR={outdir}")
    print(f"SUMMARY_JSON={summary_path}")


if __name__ == "__main__":
    main()
