"""2D raster-domain partitioning and ray projection for this experiment.

The domain owns partition boundaries and scanlines. STL triangles are queried
only to lift a 2D sample to XYZ and obtain the hit facet normal.
"""
from __future__ import annotations

import math

from robot_studio_qt.core.geometry import dot, normalize
from robot_studio_qt.path_planning.mesh_raster import MeshTriangle, average_normal, mesh_centroid, uv_axes_from_region


def chart_from_triangles(triangles: list[MeshTriangle]) -> dict:
    origin = mesh_centroid(triangles)
    normal = average_normal(triangles)
    u_axis, _discarded_v, _u_range, _v_range = uv_axes_from_region(triangles, normal)
    # Preserve one right-handed chart. Independently stabilizing U and V signs
    # can mirror the partition view even though the surface itself is unchanged.
    v_axis = normalize(_cross(normal, u_axis))
    return {"origin": list(origin), "u_axis": list(u_axis), "v_axis": list(v_axis), "normal": list(normal)}


def point_to_uv(point, chart: dict) -> tuple[float, float]:
    origin = chart["origin"]
    relative = tuple(point[index] - origin[index] for index in range(3))
    return dot(relative, chart["u_axis"]), dot(relative, chart["v_axis"])


def projected_triangles(triangles: list[MeshTriangle], chart: dict) -> list[MeshTriangle]:
    return [
        MeshTriangle(
            triangle.face_id,
            tuple((*point_to_uv(point, chart), 0.0) for point in triangle.points),
            (0.0, 0.0, 1.0),
            triangle.area,
        )
        for triangle in triangles
    ]


def polygon_scanline_intervals(polygon, y: float) -> list[tuple[float, float]]:
    crossings: list[float] = []
    previous = polygon[-1]
    for current in polygon:
        if (previous[1] <= y < current[1]) or (current[1] <= y < previous[1]):
            ratio = (y - previous[1]) / (current[1] - previous[1])
            crossings.append(previous[0] + ratio * (current[0] - previous[0]))
        previous = current
    crossings.sort()
    return [(crossings[index], crossings[index + 1]) for index in range(0, len(crossings) - 1, 2)]


def subtract_intervals(base, cuts):
    output = list(base)
    for cut_start, cut_end in cuts:
        next_output = []
        for start, end in output:
            if cut_end <= start or cut_start >= end:
                next_output.append((start, end))
            else:
                if cut_start > start:
                    next_output.append((start, min(cut_start, end)))
                if cut_end < end:
                    next_output.append((max(cut_end, start), end))
        output = next_output
    return output


def patch_axes(polygon) -> tuple[tuple[float, float], tuple[float, float]]:
    edges = []
    for start, end in zip(polygon, [*polygon[1:], polygon[0]]):
        dx, dy = end[0] - start[0], end[1] - start[1]
        length = math.hypot(dx, dy)
        if length > 1e-9:
            edges.append((length, (dx / length, dy / length)))
    if not edges:
        return (1.0, 0.0), (0.0, 1.0)

    def score(axis):
        return sum(length * abs(axis[0] * direction[0] + axis[1] * direction[1]) ** 4 for length, direction in edges)

    u_axis = max((direction for _length, direction in edges), key=score)
    v_axis = (-u_axis[1], u_axis[0])
    range_u = _projected_range(polygon, u_axis)
    range_v = _projected_range(polygon, v_axis)
    if range_v > range_u:
        u_axis, v_axis = v_axis, (-v_axis[1], v_axis[0])
    return u_axis, v_axis


def _projected_range(polygon, axis) -> float:
    values = [point[0] * axis[0] + point[1] * axis[1] for point in polygon]
    return max(values) - min(values)


def _to_scan(point, u_axis, v_axis):
    return point[0] * u_axis[0] + point[1] * u_axis[1], point[0] * v_axis[0] + point[1] * v_axis[1]


def _from_scan(u, v, u_axis, v_axis):
    return u * u_axis[0] + v * v_axis[0], u * u_axis[1] + v * v_axis[1]


def line_triangle_hit(uv, triangle: MeshTriangle, chart: dict):
    """Intersect the infinite chart-normal line with one source triangle."""
    origin, normal = chart["origin"], chart["normal"]
    plane_point = tuple(origin[index] + uv[0] * chart["u_axis"][index] + uv[1] * chart["v_axis"][index] for index in range(3))
    a, b, c = triangle.points
    edge1 = tuple(b[index] - a[index] for index in range(3))
    edge2 = tuple(c[index] - a[index] for index in range(3))
    pvec = _cross(normal, edge2)
    determinant = dot(edge1, pvec)
    if abs(determinant) <= 1e-10:
        return None
    inverse = 1.0 / determinant
    tvec = tuple(plane_point[index] - a[index] for index in range(3))
    bary_u = dot(tvec, pvec) * inverse
    if bary_u < -1e-7 or bary_u > 1.0 + 1e-7:
        return None
    qvec = _cross(tvec, edge1)
    bary_v = dot(normal, qvec) * inverse
    if bary_v < -1e-7 or bary_u + bary_v > 1.0 + 1e-7:
        return None
    distance = dot(edge2, qvec) * inverse
    point = tuple(plane_point[index] + distance * normal[index] for index in range(3))
    return distance, point, triangle.normal, triangle.face_id


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def lift_uv(uv, triangles, chart, previous_distance=None):
    hits = [hit for triangle in triangles if (hit := line_triangle_hit(uv, triangle, chart)) is not None]
    if not hits:
        return None
    chooser = (lambda hit: abs(hit[0] - previous_distance)) if previous_distance is not None else (lambda hit: abs(hit[0]))
    return min(hits, key=chooser)


def raster_samples(polygon, holes, triangles, chart, spacing: float, point_step: float, margin: float, bidirectional: bool = True, long_side: bool = True):
    """Generate regular 2D patch scanlines and lift them to the selected STL."""
    u_axis, v_axis = patch_axes(polygon)
    if not long_side:
        u_axis, v_axis = v_axis, (-v_axis[1], v_axis[0])
    outer = [_to_scan(point, u_axis, v_axis) for point in polygon]
    excluded = [[_to_scan(point, u_axis, v_axis) for point in hole] for hole in holes]
    v_min = min(point[1] for point in outer) + margin
    v_max = max(point[1] for point in outer) - margin
    samples = []
    line_index = 0
    segment_id = 0
    v = v_min
    while v <= v_max + 1e-9:
        intervals = polygon_scanline_intervals(outer, v)
        for hole in excluded:
            intervals = subtract_intervals(intervals, polygon_scanline_intervals(hole, v))
        reverse = bidirectional and line_index % 2 == 1
        if reverse:
            intervals = list(reversed(intervals))
        for start, end in intervals:
            start += margin
            end -= margin
            if end < start:
                continue
            if reverse:
                start, end = end, start
            distance = abs(end - start)
            count = max(1, int(math.floor(distance / point_step)) + 1)
            values = [start + (end - start) * min(index * point_step / max(distance, 1e-12), 1.0) for index in range(count)]
            if abs(values[-1] - end) > min(point_step * 0.45, 1e-3):
                values.append(end)
            previous_distance = None
            run = []
            for point_id, u in enumerate(values):
                uv = _from_scan(u, v, u_axis, v_axis)
                hit = lift_uv(uv, triangles, chart, previous_distance)
                if hit is None:
                    if run:
                        samples.extend((segment_id, line_index, *item) for item in run)
                        segment_id += 1
                        run = []
                    previous_distance = None
                    continue
                hit_distance, point, normal, face_id = hit
                run.append((point_id, face_id, point, normal))
                previous_distance = hit_distance
            if run:
                samples.extend((segment_id, line_index, *item) for item in run)
                segment_id += 1
        line_index += 1
        v += spacing
    return samples
