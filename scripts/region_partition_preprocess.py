from __future__ import annotations

import argparse
from copy import deepcopy
import json
import sys
from pathlib import Path

EXPERIMENT_DIR = Path(__file__).resolve().parents[1]
ROOT = EXPERIMENT_DIR.parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))
SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from robot_studio_qt.cad.import_service import CadImportService
from robot_studio_qt.cad.mesh_io import create_mesh_reader
from robot_studio_qt.path_planning.mesh_raster import read_triangles
from robot_studio_qt.project import load_project_file, save_project_file

from region_partitioning import (
    PartitionSettings,
    face_geometries_from_triangles,
    partition_selected_regions,
    record_to_manifest,
)

DEFAULT_INPUT = EXPERIMENT_DIR / "inputs" / "latest_script_test.rsp.json"
DEFAULT_OUTPUT = EXPERIMENT_DIR / "inputs" / "latest_partitioned.rsp.json"


def parse_region_numbers(raw: str) -> set[int]:
    values: set[int] = set()
    for item in raw.split(","):
        item = item.strip()
        if not item:
            continue
        value = int(item)
        if value <= 0:
            raise ValueError("--regions uses 1-based positive selected region numbers.")
        values.add(value)
    if not values:
        raise ValueError("At least one region number is required.")
    return values


def parse_angle_list(raw: str) -> tuple[int, ...]:
    values: list[int] = []
    for item in raw.split(","):
        item = item.strip()
        if not item:
            continue
        values.append(int(round(float(item))) % 360)
    if not values:
        raise ValueError("Angle list must contain at least one angle.")
    return tuple(values)


def manifest_path_for(output_path: Path) -> Path:
    name = output_path.name
    if name.endswith(".rsp.json"):
        stem = name[: -len(".rsp.json")]
    else:
        stem = output_path.stem
    return output_path.with_name(f"{stem}_manifest.json")


def load_polydata(project):
    importer = CadImportService().import_model(project.workpiece.file_path)
    reader = create_mesh_reader(importer.display_path, importer.display_format)
    reader.SetFileName(str(importer.display_path))
    reader.Update()
    return reader.GetOutput()


def preprocess_project(input_path: Path, output_path: Path, region_numbers: set[int], settings: PartitionSettings, dry_run: bool = False) -> dict:
    project = load_project_file(input_path)
    regions = [list(region) for region in project.selected_path_face_regions]
    if not regions:
        raise RuntimeError(f"No selected_path_face_regions in {input_path}")
    invalid = sorted(region for region in region_numbers if region > len(regions))
    if invalid:
        raise RuntimeError(f"Requested region(s) {invalid} but input only has {len(regions)} selected region(s).")

    selected_face_ids = {face_id for region in regions for face_id in region}
    polydata = load_polydata(project)
    triangles = read_triangles(polydata, selected_face_ids)
    faces = face_geometries_from_triangles(triangles)
    partitioned = partition_selected_regions(regions, region_numbers, faces, settings)

    manifest = {
        "schema": "base_casting_abb6700.region_partition_manifest",
        "version": 2,
        "input_project": str(input_path),
        "output_project": str(output_path),
        "selected_regions": sorted(region_numbers),
        "input_region_count": len(regions),
        "output_region_count": len(partitioned.regions),
        "settings": {
            "hard_edge_angle_deg": settings.hard_edge_angle_deg,
            "planar_normal_deg": settings.planar_normal_deg,
            "planar_rms_mm": settings.planar_rms_mm,
            "scan_spacing_mm": settings.scan_spacing_mm,
            "interval_overlap_ratio": settings.interval_overlap_ratio,
            "neck_width_ratio": settings.neck_width_ratio,
            "min_neck_lines": settings.min_neck_lines,
            "min_patch_faces": settings.min_patch_faces,
            "min_patch_area_ratio": settings.min_patch_area_ratio,
            "planar_seed_min_faces": settings.planar_seed_min_faces,
            "left_cut_x": settings.left_cut_x,
            "right_cut_x": settings.right_cut_x,
            "left_angles": list(settings.left_angles),
            "right_angles": list(settings.right_angles),
            "center_angles": list(settings.center_angles),
            "max_base_x_span_mm": settings.max_base_x_span_mm,
            "max_base_y_span_mm": settings.max_base_y_span_mm,
            "turn_histogram_bin_mm": settings.turn_histogram_bin_mm,
        },
        "records": [record_to_manifest(record) for record in partitioned.records],
    }

    if not dry_run:
        output_project = deepcopy(project)
        output_project.selected_path_face_regions = partitioned.regions
        save_project_file(output_path, output_project)
        manifest_path_for(output_path).write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")

    return manifest


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Partition selected polishing regions before ABB6700 window/conf export.")
    parser.add_argument("--regions", required=True, help="1-based selected region numbers to partition, e.g. 6 or 6,8.")
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT, help=f"Input .rsp.json project. Default: {DEFAULT_INPUT}")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT, help=f"Partitioned output .rsp.json. Default: {DEFAULT_OUTPUT}")
    parser.add_argument("--dry-run", action="store_true", help="Print the partition manifest without writing output files.")
    parser.add_argument("--dump-manifest", action="store_true", help="Print the full manifest, including face id lists.")
    parser.add_argument("--hard-edge-angle-deg", type=float, default=PartitionSettings.hard_edge_angle_deg)
    parser.add_argument("--planar-normal-deg", type=float, default=PartitionSettings.planar_normal_deg)
    parser.add_argument("--planar-rms-mm", type=float, default=PartitionSettings.planar_rms_mm)
    parser.add_argument("--scan-spacing-mm", type=float, default=PartitionSettings.scan_spacing_mm)
    parser.add_argument("--interval-overlap-ratio", type=float, default=PartitionSettings.interval_overlap_ratio)
    parser.add_argument("--neck-width-ratio", type=float, default=PartitionSettings.neck_width_ratio)
    parser.add_argument("--min-neck-lines", type=int, default=PartitionSettings.min_neck_lines)
    parser.add_argument("--min-patch-faces", type=int, default=PartitionSettings.min_patch_faces)
    parser.add_argument("--min-patch-area-ratio", type=float, default=PartitionSettings.min_patch_area_ratio)
    parser.add_argument("--planar-seed-min-faces", type=int, default=PartitionSettings.planar_seed_min_faces)
    parser.add_argument("--left-cut-x", type=float, default=PartitionSettings.left_cut_x, help="Optional model-X cut between left and center turn zones.")
    parser.add_argument("--right-cut-x", type=float, default=PartitionSettings.right_cut_x, help="Optional model-X cut between center and right turn zones.")
    parser.add_argument("--left-angles", default=",".join(str(value) for value in PartitionSettings.left_angles), help="Comma-separated allowed turntable angles for the left zone.")
    parser.add_argument("--right-angles", default=",".join(str(value) for value in PartitionSettings.right_angles), help="Comma-separated allowed turntable angles for the right zone.")
    parser.add_argument("--center-angles", default=",".join(str(value) for value in PartitionSettings.center_angles), help="Comma-separated allowed turntable angles for the center zone.")
    parser.add_argument("--max-base-x-span-mm", type=float, default=PartitionSettings.max_base_x_span_mm)
    parser.add_argument("--max-base-y-span-mm", type=float, default=PartitionSettings.max_base_y_span_mm)
    parser.add_argument("--turn-histogram-bin-mm", type=float, default=PartitionSettings.turn_histogram_bin_mm)
    return parser


def settings_from_args(args: argparse.Namespace) -> PartitionSettings:
    return PartitionSettings(
        hard_edge_angle_deg=args.hard_edge_angle_deg,
        planar_normal_deg=args.planar_normal_deg,
        planar_rms_mm=args.planar_rms_mm,
        scan_spacing_mm=args.scan_spacing_mm,
        interval_overlap_ratio=args.interval_overlap_ratio,
        neck_width_ratio=args.neck_width_ratio,
        min_neck_lines=args.min_neck_lines,
        min_patch_faces=args.min_patch_faces,
        min_patch_area_ratio=args.min_patch_area_ratio,
        planar_seed_min_faces=args.planar_seed_min_faces,
        left_cut_x=args.left_cut_x,
        right_cut_x=args.right_cut_x,
        left_angles=parse_angle_list(args.left_angles),
        right_angles=parse_angle_list(args.right_angles),
        center_angles=parse_angle_list(args.center_angles),
        max_base_x_span_mm=args.max_base_x_span_mm,
        max_base_y_span_mm=args.max_base_y_span_mm,
        turn_histogram_bin_mm=args.turn_histogram_bin_mm,
    )


def compact_summary(manifest: dict) -> dict:
    return {
        "input_project": manifest["input_project"],
        "output_project": manifest["output_project"],
        "selected_regions": manifest["selected_regions"],
        "input_region_count": manifest["input_region_count"],
        "output_region_count": manifest["output_region_count"],
        "records": [
            {
                "original_region": record["original_region"],
                "input_face_count": record["input_face_count"],
                "output_patch_count": record["output_patch_count"],
                "unchanged": record["unchanged"],
                "reason": record["reason"],
                "patches": [
                    {
                        "label": patch["label"],
                        "kind": patch["kind"],
                        "turn_zone": patch.get("turn_zone"),
                        "allowed_angles": patch.get("allowed_angles"),
                        "surface_class": patch.get("surface_class"),
                        "split_reason": patch.get("split_reason"),
                        "source": patch["source"],
                        "face_count": patch["face_count"],
                        "area": patch["area"],
                    }
                    for patch in record["patches"]
                ],
            }
            for record in manifest["records"]
        ],
    }


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    regions = parse_region_numbers(args.regions)
    manifest = preprocess_project(args.input, args.output, regions, settings_from_args(args), dry_run=args.dry_run)
    print(json.dumps(manifest if args.dump_manifest else compact_summary(manifest), ensure_ascii=False, indent=2))
    if not args.dry_run:
        print(f"Wrote partitioned project: {args.output}")
        print(f"Wrote manifest: {manifest_path_for(args.output)}")


if __name__ == "__main__":
    main()
