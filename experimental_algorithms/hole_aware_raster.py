"""Hole-aware raster ordering for continuous, on-surface machining motion.

The production raster sampler already computes valid scanline runs after holes
have been subtracted.  This module groups those runs into boustrophedon cells,
finishes one cell before visiting another, and creates free-domain connectors
instead of jumping across a hole.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import heapq
import math

from raster_domain import lift_uv, patch_axes, point_to_uv, raster_samples


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
    neighbors: set[int] = field(default_factory=set)

    def samples(self) -> list[tuple]:
        return [sample for run in sorted(self.runs, key=lambda item: item.line_index) for sample in run.samples]


def _to_scan(point: Point2, u_axis: Point2, v_axis: Point2) -> Point2:
    return (
        point[0] * u_axis[0] + point[1] * u_axis[1],
        point[0] * v_axis[0] + point[1] * v_axis[1],
    )


def _point_in_polygon(point: Point2, polygon) -> bool:
    x, y = point
    inside = False
    previous = polygon[-1]
    for current in polygon:
        x1, y1 = previous
        x2, y2 = current
        if (y1 > y) != (y2 > y):
            crossing = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < crossing:
                inside = not inside
        previous = current
    return inside


def _orientation(first: Point2, second: Point2, third: Point2) -> float:
    return (second[0] - first[0]) * (third[1] - first[1]) - (second[1] - first[1]) * (third[0] - first[0])


def _segments_intersect(a: Point2, b: Point2, c: Point2, d: Point2) -> bool:
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
    return first * second <= 1e-9 and third * fourth <= 1e-9


def polygon_has_relevant_holes(polygon, holes) -> bool:
    """Cheaply reject manifest holes that do not intersect the current patch."""
    if not polygon or not holes:
        return False
    polygon_edges = list(zip(polygon, [*polygon[1:], polygon[0]]))
    polygon_bounds = (
        min(point[0] for point in polygon),
        min(point[1] for point in polygon),
        max(point[0] for point in polygon),
        max(point[1] for point in polygon),
    )
    for hole in holes:
        if not hole:
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
        if any(_point_in_polygon(point, polygon) for point in hole) or any(_point_in_polygon(point, hole) for point in polygon):
            return True
        hole_edges = list(zip(hole, [*hole[1:], hole[0]]))
        if any(_segments_intersect(a, b, c, d) for a, b in polygon_edges for c, d in hole_edges):
            return True
    return False


def _free(point: Point2, polygon, holes) -> bool:
    return _point_in_polygon(point, polygon) and not any(_point_in_polygon(point, hole) for hole in holes)


def _interpolate(start: Point2, end: Point2, step: float) -> list[Point2]:
    distance = math.dist(start, end)
    count = max(1, int(math.ceil(distance / max(step, 1e-6))))
    return [
        (start[0] + (end[0] - start[0]) * index / count, start[1] + (end[1] - start[1]) * index / count)
        for index in range(count + 1)
    ]


def _segment_is_free(start: Point2, end: Point2, polygon, holes, step: float) -> bool:
    return all(_free(point, polygon, holes) for point in _interpolate(start, end, max(step * 0.4, 0.5)))


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
    for line_index in sorted(by_line):
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
            for predecessor in predecessors:
                if predecessor.cell_id != run.cell_id:
                    cells[predecessor.cell_id].neighbors.add(run.cell_id)
                    cells[run.cell_id].neighbors.add(predecessor.cell_id)
        previous = current
    return cells


def _uv_of_sample(sample, chart) -> Point2:
    return point_to_uv(sample[4], chart)


def _grid_route(start: Point2, end: Point2, polygon, holes, resolution: float) -> list[Point2] | None:
    """Find a deterministic free-domain route when a straight connector crosses a hole."""
    if _segment_is_free(start, end, polygon, holes, resolution):
        return [start, end]

    resolution = max(float(resolution), 1.0)
    min_x = min(point[0] for point in polygon)
    max_x = max(point[0] for point in polygon)
    min_y = min(point[1] for point in polygon)
    max_y = max(point[1] for point in polygon)
    columns = int(math.ceil((max_x - min_x) / resolution)) + 1
    rows = int(math.ceil((max_y - min_y) / resolution)) + 1
    if columns * rows > 250_000:
        scale = math.sqrt(columns * rows / 250_000)
        resolution *= scale
        columns = int(math.ceil((max_x - min_x) / resolution)) + 1
        rows = int(math.ceil((max_y - min_y) / resolution)) + 1

    def point_for(node: tuple[int, int]) -> Point2:
        return min_x + node[0] * resolution, min_y + node[1] * resolution

    free_nodes = {
        (column, row)
        for column in range(columns)
        for row in range(rows)
        if _free(point_for((column, row)), polygon, holes)
    }
    start_nodes = [node for node in free_nodes if math.dist(start, point_for(node)) <= resolution * 1.8 and _segment_is_free(start, point_for(node), polygon, holes, resolution)]
    end_nodes = {node for node in free_nodes if math.dist(end, point_for(node)) <= resolution * 1.8 and _segment_is_free(point_for(node), end, polygon, holes, resolution)}
    if not start_nodes or not end_nodes:
        return None

    queue: list[tuple[float, float, tuple[int, int]]] = []
    costs: dict[tuple[int, int], float] = {}
    previous: dict[tuple[int, int], tuple[int, int]] = {}
    for node in start_nodes:
        cost = math.dist(start, point_for(node))
        costs[node] = cost
        heapq.heappush(queue, (cost + math.dist(point_for(node), end), cost, node))
    target = None
    directions = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    while queue:
        _estimate, cost, node = heapq.heappop(queue)
        if cost > costs.get(node, math.inf) + 1e-9:
            continue
        if node in end_nodes:
            target = node
            break
        for dx, dy in directions:
            neighbor = (node[0] + dx, node[1] + dy)
            if neighbor not in free_nodes:
                continue
            if not _segment_is_free(point_for(node), point_for(neighbor), polygon, holes, resolution):
                continue
            next_cost = cost + math.dist(point_for(node), point_for(neighbor))
            if next_cost + 1e-9 >= costs.get(neighbor, math.inf):
                continue
            costs[neighbor] = next_cost
            previous[neighbor] = node
            heapq.heappush(queue, (next_cost + math.dist(point_for(neighbor), end), next_cost, neighbor))
    if target is None:
        return None
    nodes = [target]
    while nodes[-1] in previous:
        nodes.append(previous[nodes[-1]])
    nodes.reverse()
    route = [start, *[point_for(node) for node in nodes], end]
    simplified = [route[0]]
    anchor = 0
    while anchor < len(route) - 1:
        candidate = len(route) - 1
        while candidate > anchor + 1 and not _segment_is_free(route[anchor], route[candidate], polygon, holes, resolution):
            candidate -= 1
        simplified.append(route[candidate])
        anchor = candidate
    return simplified


def _lift_connector(route: list[Point2], triangles, chart, point_step: float) -> list[tuple] | None:
    uv_points: list[Point2] = []
    for start, end in zip(route, route[1:]):
        points = _interpolate(start, end, point_step)
        uv_points.extend(points if not uv_points else points[1:])
    lifted = []
    previous_distance = None
    for uv in uv_points[1:-1]:
        hit = lift_uv(uv, triangles, chart, previous_distance)
        if hit is None:
            return None
        previous_distance, point, normal, face_id = hit
        lifted.append((face_id, point, normal))
    return lifted


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
    """Return ordered samples, visiting complete cells without crossing holes."""
    raw = raster_samples(polygon, holes, triangles, chart, spacing, point_step, margin, bidirectional, long_side)
    if not raw:
        return [], {"cell_count": 0, "connector_count": 0, "valid": False}
    runs = _make_runs(raw, chart, polygon, long_side)
    cells = _build_cells(runs, spacing)
    cell_paths = {cell.cell_id: cell.samples() for cell in cells}
    first_cell = min(cells, key=lambda cell: min(run.line_index for run in cell.runs)).cell_id
    ordered_cells: list[tuple[int, list[tuple], list[tuple]]] = [(first_cell, cell_paths[first_cell], [])]
    unvisited = {cell.cell_id for cell in cells} - {first_cell}

    while unvisited:
        current_id, current_path, _connector = ordered_cells[-1]
        current_uv = _uv_of_sample(current_path[-1], chart)
        candidates = []
        for cell_id in sorted(unvisited):
            for reversed_path in (False, True):
                candidate_path = list(reversed(cell_paths[cell_id])) if reversed_path else cell_paths[cell_id]
                target_uv = _uv_of_sample(candidate_path[0], chart)
                route = _grid_route(current_uv, target_uv, polygon, holes, max(min(spacing, point_step) * 0.5, 1.0))
                if route is None:
                    continue
                lifted = _lift_connector(route, triangles, chart, point_step)
                if lifted is None:
                    continue
                adjacency_penalty = 0 if cell_id in cells[current_id].neighbors else 1
                length = sum(math.dist(a, b) for a, b in zip(route, route[1:]))
                candidates.append((adjacency_penalty, length, cell_id, reversed_path, candidate_path, lifted))
        if not candidates:
            return [], {"cell_count": len(cells), "connector_count": len(ordered_cells) - 1, "valid": False, "reason": "No free-domain connector between raster cells."}
        _penalty, _length, cell_id, _reversed, candidate_path, lifted = min(candidates, key=lambda item: item[:4])
        ordered_cells.append((cell_id, candidate_path, lifted))
        unvisited.remove(cell_id)

    output = []
    sequence_segment = 0
    for cell_id, path, connector in ordered_cells:
        if connector:
            for point_id, (face_id, point, normal) in enumerate(connector):
                output.append((sequence_segment, path[0][1], point_id, face_id, point, normal))
            sequence_segment += 1
        for _source_segment, line_id, point_id, face_id, point, normal in path:
            output.append((sequence_segment, line_id, point_id, face_id, point, normal))
        sequence_segment += 1
    return output, {"cell_count": len(cells), "connector_count": max(0, len(ordered_cells) - 1), "valid": True}
