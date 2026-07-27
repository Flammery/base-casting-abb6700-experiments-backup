from __future__ import annotations

import argparse
import json
from pathlib import Path

from step_cad_core import (
    attach_pick_partitions,
    build_face_index,
    read_manifest,
    sample_manifest_patches,
    samples_to_jsonable,
    write_manifest,
)


def parse_regions(raw: str) -> list[list[int]]:
    if not raw.strip():
        return []
    regions: list[list[int]] = []
    for chunk in raw.split(";"):
        values = [int(item.strip()) for item in chunk.split(",") if item.strip()]
        if values:
            regions.append(values)
    return regions


def parse_polygons(path: Path) -> dict[int, list[list[tuple[float, float]]]]:
    data = json.loads(path.read_text(encoding="utf-8"))
    result: dict[int, list[list[tuple[float, float]]]] = {}
    for key, polygons in data.items():
        result[int(key)] = [
            [(float(point[0]), float(point[1])) for point in polygon]
            for polygon in polygons
        ]
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description="STEP/CAD precise-surface experiment pipeline.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    index_parser = subparsers.add_parser("index", help="Read STEP and write a CAD face manifest.")
    index_parser.add_argument("--step", required=True, type=Path)
    index_parser.add_argument("--manifest", required=True, type=Path)

    pick_parser = subparsers.add_parser("pick", help="Attach selected CAD face regions and picked clip polygons.")
    pick_parser.add_argument("--manifest", required=True, type=Path)
    pick_parser.add_argument("--regions", required=True, help="Region CAD faces, e.g. '1,2;5'.")
    pick_parser.add_argument("--polygons-json", required=True, type=Path, help="JSON object keyed by 1-based region number.")
    pick_parser.add_argument("--output", required=True, type=Path)

    sample_parser = subparsers.add_parser("sample", help="Sample CAD surfaces from manifest patches.")
    sample_parser.add_argument("--step", required=True, type=Path)
    sample_parser.add_argument("--manifest", required=True, type=Path)
    sample_parser.add_argument("--output", required=True, type=Path)
    sample_parser.add_argument("--spacing", type=float, default=20.0)
    sample_parser.add_argument("--point-step", type=float, default=20.0)
    sample_parser.add_argument("--boundary-margin", type=float, default=0.0)

    args = parser.parse_args()
    if args.command == "index":
        manifest = build_face_index(args.step)
        write_manifest(args.manifest, manifest)
        print(f"Wrote STEP CAD manifest: {args.manifest}")
        return 0

    if args.command == "pick":
        manifest = read_manifest(args.manifest)
        updated = attach_pick_partitions(manifest, parse_regions(args.regions), parse_polygons(args.polygons_json))
        write_manifest(args.output, updated)
        print(f"Wrote STEP pick manifest: {args.output}")
        return 0

    if args.command == "sample":
        manifest = read_manifest(args.manifest)
        samples = sample_manifest_patches(args.step, manifest, args.spacing, args.point_step, args.boundary_margin)
        args.output.write_text(json.dumps(samples_to_jsonable(samples), ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"Wrote STEP CAD samples: {args.output} ({len(samples)} points)")
        return 0

    raise AssertionError(args.command)


if __name__ == "__main__":
    raise SystemExit(main())
