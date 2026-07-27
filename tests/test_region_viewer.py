from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import vtk
from vtkmodules.vtkRenderingCore import vtkPolyDataMapper


def _load_region_viewer_module():
    script = Path(__file__).resolve().parents[1] / "ui" / "region_viewer.py"
    spec = importlib.util.spec_from_file_location("region_viewer_test", script)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _manifest(records: list[dict]) -> dict:
    return {
        "schema": "base_casting_abb6700.manual_region_partition_manifest",
        "version": 2,
        "records": records,
    }


def _record(source_region: int, labels: list[str]) -> dict:
    return {
        "original_region": source_region,
        "raster_chart": {"origin": [0.0, 0.0, 0.0]},
        "patches": [
            {
                "label": label,
                "clip_polygon": [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]],
            }
            for label in labels
        ],
    }


def test_raster_preview_replaces_partitioned_source_and_keeps_other_regions() -> None:
    viewer = _load_region_viewer_module()

    plan = viewer.build_raster_preview_plan(
        _manifest([_record(1, ["1_1"])]),
        [{10, 11}, {20}, {30}],
    )

    assert plan is not None
    assert plan["labels"] == ["1_1", "2", "3"]
    assert [group["source_region"] for group in plan["groups"]] == [1]
    assert [entry["source_region"] for entry in plan["passthrough"]] == [2, 3]
    assert [entry["face_ids"] for entry in plan["passthrough"]] == [{20}, {30}]
    assert plan["groups"][0]["patches"][0]["preview_color"] == viewer.PALETTE[0]
    assert plan["passthrough"][0]["preview_color"] == viewer.PALETTE[1]
    assert plan["passthrough"][1]["preview_color"] == viewer.PALETTE[2]


def test_raster_preview_preserves_source_order_with_multiple_patches() -> None:
    viewer = _load_region_viewer_module()

    plan = viewer.build_raster_preview_plan(
        _manifest([_record(2, ["2_1", "2_2"])]),
        [{10}, {20}, {30}],
    )

    assert plan is not None
    assert plan["labels"] == ["1", "2_1", "2_2", "3"]
    assert [patch["preview_color"] for patch in plan["groups"][0]["patches"]] == [
        viewer.PALETTE[1],
        viewer.PALETTE[2],
    ]
    assert [entry["preview_color"] for entry in plan["passthrough"]] == [
        viewer.PALETTE[0],
        viewer.PALETTE[3],
    ]


def test_raster_overlay_mapper_has_camera_facing_relative_depth_offset() -> None:
    viewer = _load_region_viewer_module()
    mapper = vtkPolyDataMapper()

    viewer.configure_raster_overlay_mapper(mapper)

    factor = vtk.reference(0.0)
    units = vtk.reference(0.0)
    mapper.GetRelativeCoincidentTopologyPolygonOffsetParameters(factor, units)
    assert float(factor) == viewer.RASTER_OVERLAY_OFFSET_FACTOR
    assert float(units) == viewer.RASTER_OVERLAY_OFFSET_UNITS
    assert mapper.GetResolveCoincidentTopology() != 0
