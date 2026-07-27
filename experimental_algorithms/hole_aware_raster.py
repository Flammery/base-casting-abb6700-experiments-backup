"""Hole-aware raster cells for polishing with lifted inter-cell transfers.

The production raster sampler already removes explicit exclusions and splits a
scanline whenever the selected mesh has no ray hit.  This module groups those
valid runs into stable boustrophedon cells.  A cell is finished completely
before the exporter retracts the tool and transfers to the next cell; no
on-surface connector is generated across a hole or unsupported surface gap.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from robot_studio_qt.path_planning.mesh_raster import raster_base_line_id, raster_segment_id
from raster_domain import patch_axes, point_to_uv, raster_samples


Point2 = tuple[float, float]


@dataclass
class RasterRun:
    run_id: int
    line_index: int
    samples: list[tuple]
    u_min: float
    u_max: float
    cell_id: int = -1


@dataclass
class RasterCell:
    cell_id: int
    runs: list[RasterRun] = field(default_factory=list)

    def samples(self) -> list[tuple]:
        return [sample for run in sorted(self.runs, key=lambda item: item.line_index) for sample in run.samples]


def _to_scan(point: Point2, u_axis: Point2, v_axis: Point2) -> Point2:
    return (
        point[0] * u_axis[0] + point[1] * u_axis[1],
        point[0] * v_axis[0] + point[1] * v_axis[1],
    )


def _point_location(point: Point2, polygon, tolerance: float = 1e-9) -> int:
    """Return 1 inside, 0 on the boundary, and -1 outside a polygon."""
    x, y = point
    inside = False
    previous = polygon[-1]
    for current in polygon:
        x1, y1 = previous
        x2, y2 = current
        cross = _orientation(previous, current, point)
        if (
            abs(cross) <= tolerance
            and min(x1, x2) - tolerance <= x <= max(x1, x2) + tolerance
            and min(y1, y2) - tolerance <= y <= max(y1, y2) + tolerance
        ):
            return 0
        if (y1 > y) != (y2 > y):
            crossing = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < crossing:
                inside = not inside
        previous = current
    return 1 if inside else -1


def _orientation(first: Point2, second: Point2, third: Point2) -> float:
    return (second[0] - first[0]) * (third[1] - first[1]) - (second[1] - first[1]) * (third[0] - first[0])


def _segments_properly_intersect(a: Point2, b: Point2, c: Point2, d: Point2) -> bool:
    """Return true only for an interior crossing, not a boundary-only touch."""
    if (
        max(a[0], b[0]) < min(c[0], d[0]) - 1e-9
        or max(c[0], d[0]) < min(a[0], b[0]) - 1e-9
        or max(a[1], b[1]) < min(c[1], d[1]) - 1e-9
        or max(c[1], d[1]) < min(a[1], b[1]) - 1e-9
    ):
        return False
    first = _orientation(a, b, c)
    second = _orientation(a, b, d)
    third = _orientation(c, d, a)
    fourth = _orientation(c, d, b)
    return first * second < -1e-9 and third * fourth < -1e-9


def polygon_has_relevant_holes(polygon, holes) -> bool:
    """Return whether a hole has positive-area overlap with the current patch.

    A hole fully inside the clip polygon is the primary hole case and must be
    selected.  A hole crossing the clip boundary is also relevant because it
    removes supported raster area near the edge.  Merely touching the boundary
    at one point or along one edge does not switch planners.
    """
    if not polygon or len(polygon) < 3 or not holes:
        return False
    polygon_edges = list(zip(polygon, [*polygon[1:], polygon[0]]))
    polygon_bounds = (
        min(point[0] for point in polygon),
        min(point[1] for point in polygon),
        max(point[0] for point in polygon),
        max(point[1] for point in polygon),
    )
    for hole in holes:
        if not hole or len(hole) < 3:
            continue
        hole_bounds = (
            min(point[0] for point in hole),
            min(point[1] for point in hole),
            max(point[0] for point in hole),
            max(point[1] for point in hole),
        )
        if (
            hole_bounds[2] < polygon_bounds[0]
            or hole_bounds[0] > polygon_bounds[2]
            or hole_bounds[3] < polygon_bounds[1]
            or hole_bounds[1] > polygon_bounds[3]
        ):
            continue
        if any(_point_location(point, polygon) == 1 for point in hole):
            return True
        if any(_point_location(point, hole) == 1 for point in polygon):
            return True
        hole_edges = list(zip(hole, [*hole[1:], hole[0]]))
        if any(_segments_properly_intersect(a, b, c, d) for a, b in polygon_edges for c, d in hole_edges):
            return True
    return False


def _interval_overlap(first: RasterRun, second: RasterRun, tolerance: float) -> bool:
    return min(first.u_max, second.u_max) >= max(first.u_min, second.u_min) - tolerance


def _make_runs(raw_samples, chart, polygon, long_side: bool) -> list[RasterRun]:
    u_axis, v_axis = patch_axes(polygon)
    if not long_side:
        u_axis, v_axis = v_axis, (-v_axis[1], v_axis[0])
    grouped: dict[int, list[tuple]] = {}
    for sample in raw_samples:
        grouped.setdefault(sample[0], []).append(sample)
    runs = []
    for run_id, samples in grouped.items():
        scan_u = [_to_scan(point_to_uv(sample[4], chart), u_axis, v_axis)[0] for sample in samples]
        runs.append(RasterRun(run_id, samples[0][1], samples, min(scan_u), max(scan_u)))
    return sorted(runs, key=lambda run: (run.line_index, run.u_min))


def _build_cells(runs: list[RasterRun], spacing: float) -> list[RasterCell]:
    by_line: dict[int, list[RasterRun]] = {}
    for run in runs:
        by_line.setdefault(run.line_index, []).append(run)
    cells: list[RasterCell] = []
    previous: list[RasterRun] = []
    previous_line_index = None
    for line_index in sorted(by_line):
        if previous_line_index is not None and line_index != previous_line_index + 1:
            previous = []
        current = by_line[line_index]
        predecessor_map = {
            run.run_id: [old for old in previous if _interval_overlap(old, run, max(spacing * 0.05, 1e-6))]
            for run in current
        }
        successor_count = {
            old.run_id: sum(old in predecessors for predecessors in predecessor_map.values())
            for old in previous
        }
        for run in current:
            predecessors = predecessor_map[run.run_id]
            if len(predecessors) == 1 and successor_count[predecessors[0].run_id] == 1:
                run.cell_id = predecessors[0].cell_id
            else:
                run.cell_id = len(cells)
                cells.append(RasterCell(run.cell_id))
            cells[run.cell_id].runs.append(run)
        previous = current
        previous_line_index = line_index
    return cells


def _ordered_cell_samples(runs: list[RasterRun], spacing: float):
    if not runs:
        return [], {
            "cell_count": 0,
            "transfer_count": 0,
            "connector_count": 0,
            "valid": False,
            "reason": "No valid raster samples.",
        }
    cells = _build_cells(runs, spacing)
    ordered_cells = sorted(
        cells,
        key=lambda cell: (
            min(run.line_index for run in cell.runs),
            min(run.u_min for run in cell.runs),
            cell.cell_id,
        ),
    )
    output = []
    for sequence_cell, cell in enumerate(ordered_cells):
        for _source_segment, line_id, point_id, face_id, point, normal in cell.samples():
            output.append((sequence_cell, line_id, point_id, face_id, point, normal))
    return output, {
        "cell_count": len(cells),
        "cell_order": [cell.cell_id for cell in ordered_cells],
        "transfer_count": max(0, len(cells) - 1),
        "connector_count": 0,
        "valid": True,
    }


def projected_raster_cell_samples(samples, origin, u_axis, spacing: float):
    """Build cells from the normal projected-mesh raster without a manifest.

    ``samples`` must already have discontinuous same-line jumps encoded as
    independent segment ids.  The selected region's projection U axis supplies
    the interval coordinate that the manual-v2 chart normally provides.
    """
    grouped: dict[tuple[int, int], list[tuple]] = {}
    for line_id, point_id, face_id, point, normal in samples:
        line_index = raster_base_line_id(line_id)
        source_segment = raster_segment_id(line_id)
        grouped.setdefault((line_index, source_segment), []).append(
            (source_segment, line_index, point_id, face_id, point, normal)
        )

    pending = []
    for (line_index, _source_segment), run_samples in grouped.items():
        scan_u = [
            sum((sample[4][axis] - origin[axis]) * u_axis[axis] for axis in range(3))
            for sample in run_samples
        ]
        pending.append((line_index, min(scan_u), max(scan_u), run_samples))
    pending.sort(key=lambda item: (item[0], item[1]))
    runs = [
        RasterRun(run_id, line_index, run_samples, u_min, u_max)
        for run_id, (line_index, u_min, u_max, run_samples) in enumerate(pending)
    ]
    return _ordered_cell_samples(runs, spacing)


def hole_aware_raster_samples(
    polygon,
    holes,
    triangles,
    chart,
    spacing: float,
    point_step: float,
    margin: float,
    bidirectional: bool = True,
    long_side: bool = True,
):
    """Return complete raster cells in their deterministic scan discovery order."""
    raw = raster_samples(polygon, holes, triangles, chart, spacing, point_step, margin, bidirectional, long_side)
    runs = _make_runs(raw, chart, polygon, long_side)
    return _ordered_cell_samples(runs, spacing)
