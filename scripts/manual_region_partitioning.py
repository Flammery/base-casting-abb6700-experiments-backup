from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass
import sys
from pathlib import Path

EXPERIMENT_DIR = Path(__file__).resolve().parents[1]
ROOT = EXPERIMENT_DIR.parents[1]
SRC = ROOT / "src"
SCRIPT_DIR = Path(__file__).resolve().parent
for path in (SRC, SCRIPT_DIR):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from robot_studio_qt.path_planning.mesh_raster import MeshTriangle

from region_partitioning import FaceGeometry, PartitionSettings, connected_components, point_key

Point2 = tuple[float, float]
BarrierLine = tuple[Point2, Point2]
PARTITION_MODE_BOUNDARY = "boundary"
PARTITION_MODE_SLAB = "slab"
PARTITION_MODE_PICK = "pick"
PARTITION_MODES = {PARTITION_MODE_BOUNDARY, PARTITION_MODE_SLAB, PARTITION_MODE_PICK}


@dataclass(frozen=True)
class ManualPartitionResult:
    regions: list[list[int]]
    cut_edge_count: int


@dataclass(frozen=True)
class ClipPartition:
    label: str
    source_region: int
    clip_polygon_model_xy: list[Point2]
    barrier_range: tuple[float | None, float | None]
    exclude_polygons_model_xy: list[list[Point2]] | None = None


def _orientation(a: Point2, b: Point2, c: Point2) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _on_segment(a: Point2, b: Point2, c: Point2, epsilon: float = 1e-9) -> bool:
    return (
        min(a[0], b[0]) - epsilon <= c[0] <= max(a[0], b[0]) + epsilon
        and min(a[1], b[1]) - epsilon <= c[1] <= max(a[1], b[1]) + epsilon
    )


def segments_intersect(a: Point2, b: Point2, c: Point2, d: Point2, epsilon: float = 1e-9) -> bool:
    ab_c = _orientation(a, b, c)
    ab_d = _orientation(a, b, d)
    cd_a = _orientation(c, d, a)
    cd_b = _orientation(c, d, b)

    if abs(ab_c) <= epsilon and _on_segment(a, b, c, epsilon):
        return True
    if abs(ab_d) <= epsilon and _on_segment(a, b, d, epsilon):
        return True
    if abs(cd_a) <= epsilon and _on_segment(c, d, a, epsilon):
        return True
    if abs(cd_b) <= epsilon and _on_segment(c, d, b, epsilon):
        return True
    return (ab_c > 0.0) != (ab_d > 0.0) and (cd_a > 0.0) != (cd_b > 0.0)


def _triangle_edges(triangle: MeshTriangle):
    points = triangle.points
    yield points[0], points[1]
    yield points[1], points[2]
    yield points[2], points[0]


def build_barrier_cut_adjacency(
    face_ids: set[int],
    faces: dict[int, FaceGeometry],
    barriers: list[BarrierLine],
    settings: PartitionSettings | None = None,
) -> tuple[dict[int, set[int]], int]:
    settings = settings or PartitionSettings()
    edge_faces: dict[tuple[tuple[int, int, int], tuple[int, int, int]], list[tuple[int, Point2, Point2]]] = defaultdict(list)
    for face_id in face_ids:
        face = faces.get(face_id)
        if face is None:
            continue
        for triangle in face.triangles:
            for start, end in _triangle_edges(triangle):
                key = tuple(sorted((point_key(start, settings.point_quantization), point_key(end, settings.point_quantization))))
                edge_faces[key].append((face_id, (start[0], start[1]), (end[0], end[1])))  # type: ignore[arg-type]

    adjacency = {face_id: set() for face_id in face_ids if face_id in faces}
    cut_edge_count = 0
    for entries in edge_faces.values():
        if len(entries) < 2:
            continue
        cuts_edge = any(segments_intersect(start, end, barrier_start, barrier_end) for _face_id, start, end in entries[:1] for barrier_start, barrier_end in barriers)
        ids = sorted({face_id for face_id, _start, _end in entries})
        for index, face_id in enumerate(ids):
            for other_id in ids[index + 1 :]:
                if cuts_edge:
                    cut_edge_count += 1
                    continue
                adjacency[face_id].add(other_id)
                adjacency[other_id].add(face_id)
    return adjacency, cut_edge_count


def partition_face_ids_by_barriers(
    face_ids: set[int],
    faces: dict[int, FaceGeometry],
    barriers: list[BarrierLine],
    settings: PartitionSettings | None = None,
) -> ManualPartitionResult:
    if not barriers:
        return ManualPartitionResult([sorted(face_ids)], 0)
    adjacency, cut_edge_count = build_barrier_cut_adjacency(face_ids, faces, barriers, settings)
    components = connected_components(set(adjacency), adjacency)
    regions = [sorted(component) for component in sorted(components, key=lambda item: (min(item), len(item))) if component]
    return ManualPartitionResult(regions or [sorted(face_ids)], cut_edge_count)


def replace_regions_with_manual_partitions(
    regions: list[list[int]],
    selected_region_numbers: set[int],
    faces: dict[int, FaceGeometry],
    barriers: list[BarrierLine],
    settings: PartitionSettings | None = None,
) -> tuple[list[list[int]], list[dict]]:
    output_regions: list[list[int]] = []
    records: list[dict] = []
    for region_index, raw_region in enumerate(regions, 1):
        face_ids = {int(face_id) for face_id in raw_region}
        if region_index not in selected_region_numbers:
            output_regions.append(sorted(face_ids))
            records.append(
                {
                    "original_region": region_index,
                    "unchanged": True,
                    "reason": "not selected",
                    "output_patch_count": 1 if face_ids else 0,
                    "patches": [{"label": str(region_index), "face_count": len(face_ids), "face_ids": sorted(face_ids)}] if face_ids else [],
                }
            )
            continue

        result = partition_face_ids_by_barriers(face_ids, faces, barriers, settings)
        patches = []
        for sub_index, partition in enumerate(result.regions, 1):
            label = f"{region_index}.{sub_index}" if len(result.regions) > 1 else str(region_index)
            patches.append({"label": label, "face_count": len(partition), "face_ids": partition})
            output_regions.append(partition)
        records.append(
            {
                "original_region": region_index,
                "unchanged": len(result.regions) == 1,
                "reason": "manual_barrier",
                "cut_edge_count": result.cut_edge_count,
                "output_patch_count": len(result.regions),
                "patches": patches,
            }
        )
    return output_regions, records


def _distance2(a: Point2, b: Point2) -> float:
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2


def _polygon_area(polygon: list[Point2]) -> float:
    if len(polygon) < 3:
        return 0.0
    area = 0.0
    previous = polygon[-1]
    for current in polygon:
        area += previous[0] * current[1] - current[0] * previous[1]
        previous = current
    return area * 0.5


def _polygon_centroid_x(polygon: list[Point2]) -> float:
    if not polygon:
        return 0.0
    return sum(point[0] for point in polygon) / len(polygon)


def _polygon_centroid(polygon: list[Point2]) -> Point2:
    if not polygon:
        return (0.0, 0.0)
    return (sum(point[0] for point in polygon) / len(polygon), sum(point[1] for point in polygon) / len(polygon))


def _point_in_polygon(point: Point2, polygon: list[Point2]) -> bool:
    if len(polygon) < 3:
        return False
    inside = False
    x, y = point
    previous = polygon[-1]
    for current in polygon:
        xi, yi = current
        xj, yj = previous
        if (yi > y) != (yj > y):
            cross_x = (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi
            if x <= cross_x:
                inside = not inside
        previous = current
    return inside


def _dedupe_polygon_points(points: list[Point2], epsilon: float = 1e-7) -> list[Point2]:
    output: list[Point2] = []
    for point in points:
        if output and _distance2(output[-1], point) <= epsilon * epsilon:
            continue
        output.append(point)
    if len(output) > 1 and _distance2(output[0], output[-1]) <= epsilon * epsilon:
        output.pop()
    return output


def _dot2(point: Point2, axis: Point2) -> float:
    return point[0] * axis[0] + point[1] * axis[1]


def _normalize2(vector: Point2) -> Point2:
    length = (vector[0] * vector[0] + vector[1] * vector[1]) ** 0.5
    if length <= 1e-12:
        return (1.0, 0.0)
    return (vector[0] / length, vector[1] / length)


def _clip_polygon_by_half_plane(polygon: list[Point2], axis: Point2, threshold: float, keep_less_equal: bool) -> list[Point2]:
    if not polygon:
        return []

    def inside(point: Point2) -> bool:
        value = _dot2(point, axis)
        return value <= threshold + 1e-9 if keep_less_equal else value >= threshold - 1e-9

    def intersection(a: Point2, b: Point2) -> Point2:
        da = _dot2(a, axis) - threshold
        db = _dot2(b, axis) - threshold
        denom = da - db
        if abs(denom) <= 1e-12:
            return a
        t = da / denom
        return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)

    output: list[Point2] = []
    previous = polygon[-1]
    previous_inside = inside(previous)
    for current in polygon:
        current_inside = inside(current)
        if current_inside:
            if not previous_inside:
                output.append(intersection(previous, current))
            output.append(current)
        elif previous_inside:
            output.append(intersection(previous, current))
        previous = current
        previous_inside = current_inside
    return output


def face_ids_bounds_xy(face_ids: set[int], faces: dict[int, FaceGeometry]) -> tuple[float, float, float, float]:
    points = [
        (point[0], point[1])
        for face_id in face_ids
        if face_id in faces
        for triangle in faces[face_id].triangles
        for point in triangle.points
    ]
    if not points:
        return (0.0, 0.0, 0.0, 0.0)
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    return min(xs), min(ys), max(xs), max(ys)


def boundary_loops_xy(face_ids: set[int], faces: dict[int, FaceGeometry], settings: PartitionSettings | None = None) -> list[list[Point2]]:
    settings = settings or PartitionSettings()
    edge_counts: dict[tuple[tuple[int, int, int], tuple[int, int, int]], int] = defaultdict(int)
    key_points: dict[tuple[int, int, int], Point2] = {}
    for face_id in face_ids:
        face = faces.get(face_id)
        if face is None:
            continue
        for triangle in face.triangles:
            for start, end in _triangle_edges(triangle):
                start_key = point_key(start, settings.point_quantization)
                end_key = point_key(end, settings.point_quantization)
                key_points[start_key] = (start[0], start[1])
                key_points[end_key] = (end[0], end[1])
                edge_counts[tuple(sorted((start_key, end_key)))] += 1

    adjacency: dict[tuple[int, int, int], set[tuple[int, int, int]]] = defaultdict(set)
    boundary_edges = [edge_key for edge_key, count in edge_counts.items() if count == 1]
    unused = {edge_key for edge_key in boundary_edges}
    for start_key, end_key in boundary_edges:
        adjacency[start_key].add(end_key)
        adjacency[end_key].add(start_key)

    loops: list[list[Point2]] = []
    while unused:
        start_key, next_key = next(iter(unused))
        unused.remove(tuple(sorted((start_key, next_key))))
        loop_keys = [start_key]
        previous_key = start_key
        current_key = next_key
        guard = 0
        while current_key != start_key and guard < len(boundary_edges) + 4:
            loop_keys.append(current_key)
            candidates = [
                key
                for key in adjacency[current_key]
                if key != previous_key and tuple(sorted((current_key, key))) in unused
            ]
            if not candidates:
                break
            following_key = candidates[0]
            unused.remove(tuple(sorted((current_key, following_key))))
            previous_key, current_key = current_key, following_key
            guard += 1
        points = _dedupe_polygon_points([key_points[key] for key in loop_keys if key in key_points])
        if len(points) >= 3:
            loops.append(points)
    return sorted(loops, key=lambda loop: abs(_polygon_area(loop)), reverse=True)


def outer_boundary_polygon_xy(face_ids: set[int], faces: dict[int, FaceGeometry]) -> list[Point2]:
    loops = boundary_loops_xy(face_ids, faces)
    if loops:
        return loops[0]
    x_min, y_min, x_max, y_max = face_ids_bounds_xy(face_ids, faces)
    return [(x_min, y_min), (x_max, y_min), (x_max, y_max), (x_min, y_max)]


def boundary_polygon_with_holes_xy(face_ids: set[int], faces: dict[int, FaceGeometry]) -> tuple[list[Point2], list[list[Point2]]]:
    loops = boundary_loops_xy(face_ids, faces)
    if not loops:
        return outer_boundary_polygon_xy(face_ids, faces), []
    outer = loops[0]
    # 划分边界只用外轮廓；孔洞作为排除区写入 manifest，路径采样时自然不会进入孔内。
    holes = [loop for loop in loops[1:] if _point_in_polygon(_polygon_centroid(loop), outer)]
    return outer, holes


def _ring_stations(ring: list[Point2]) -> tuple[list[float], float]:
    stations = [0.0]
    total = 0.0
    for index, point in enumerate(ring):
        following = ring[(index + 1) % len(ring)]
        total += _distance2(point, following) ** 0.5
        stations.append(total)
    return stations, total


def _nearest_ring_projection(point: Point2, ring: list[Point2]) -> tuple[float, Point2]:
    stations, total = _ring_stations(ring)
    best_distance = float("inf")
    best_station = 0.0
    best_point = ring[0]
    for index, start in enumerate(ring):
        end = ring[(index + 1) % len(ring)]
        segment = (end[0] - start[0], end[1] - start[1])
        length_sq = segment[0] * segment[0] + segment[1] * segment[1]
        if length_sq <= 1e-12:
            projected = start
            t = 0.0
        else:
            t = max(0.0, min(1.0, ((point[0] - start[0]) * segment[0] + (point[1] - start[1]) * segment[1]) / length_sq))
            projected = (start[0] + segment[0] * t, start[1] + segment[1] * t)
        distance = _distance2(point, projected)
        if distance < best_distance:
            best_distance = distance
            best_station = min(total, stations[index] + (length_sq ** 0.5) * t)
            best_point = projected
    return best_station, best_point


def _ring_path(ring: list[Point2], from_station: float, from_point: Point2, to_station: float, to_point: Point2, forward: bool) -> list[Point2]:
    stations, total = _ring_stations(ring)
    if total <= 1e-12:
        return [from_point, to_point]

    entries = [(station, point) for station, point in zip(stations[:-1], ring)]
    entries.extend([(from_station % total, from_point), (to_station % total, to_point)])
    entries.sort(key=lambda item: item[0])

    if forward:
        if from_station <= to_station:
            points = [point for station, point in entries if from_station <= station <= to_station]
        else:
            points = [point for station, point in entries if station >= from_station or station <= to_station]
    else:
        if to_station <= from_station:
            points = [point for station, point in entries if to_station <= station <= from_station]
        else:
            points = [point for station, point in entries if station <= from_station or station >= to_station]
        points.reverse()
    if not points or _distance2(points[0], from_point) > 1e-8:
        points.insert(0, from_point)
    if _distance2(points[-1], to_point) > 1e-8:
        points.append(to_point)
    return _dedupe_polygon_points(points)


def side_polygon_from_barrier(boundary: list[Point2], barrier: BarrierLine, region_center_x: float) -> list[Point2]:
    start, end = barrier
    start_station, start_on_boundary = _nearest_ring_projection(start, boundary)
    end_station, end_on_boundary = _nearest_ring_projection(end, boundary)
    path_forward = _ring_path(boundary, end_station, end_on_boundary, start_station, start_on_boundary, forward=True)
    path_backward = _ring_path(boundary, end_station, end_on_boundary, start_station, start_on_boundary, forward=False)
    candidates = [
        _dedupe_polygon_points([start, end, *path_forward]),
        _dedupe_polygon_points([start, end, *path_backward]),
    ]
    midpoint_x = (start[0] + end[0]) * 0.5
    return max(candidates, key=_polygon_centroid_x) if midpoint_x >= region_center_x else min(candidates, key=_polygon_centroid_x)


def clip_partitions_from_barriers_slab(
    source_region: int,
    face_ids: set[int],
    faces: dict[int, FaceGeometry],
    barriers: list[BarrierLine],
    padding_ratio: float = 0.04,
) -> list[ClipPartition]:
    x_min, y_min, x_max, y_max = face_ids_bounds_xy(face_ids, faces)
    span = max(x_max - x_min, y_max - y_min, 1.0)
    padding = span * padding_ratio
    base_polygon: list[Point2] = [
        (x_min - padding, y_min - padding),
        (x_max + padding, y_min - padding),
        (x_max + padding, y_max + padding),
        (x_min - padding, y_max + padding),
    ]
    valid_barriers = [
        barrier
        for barrier in barriers
        if abs(barrier[0][0] - barrier[1][0]) + abs(barrier[0][1] - barrier[1][1]) > 1e-9
    ]
    if not valid_barriers:
        return [ClipPartition(f"{source_region}_1", source_region, base_polygon, (None, None))]

    # 贯穿式模式保留旧版逻辑：把拉线视为一组近似平行的无限分割线，
    # 沿法向切出规则 slab，适合快速做左右/中间粗分区。
    directions = [(end[0] - start[0], end[1] - start[1]) for start, end in valid_barriers]
    average_direction = _normalize2((sum(item[0] for item in directions), sum(item[1] for item in directions)))
    axis = _normalize2((-average_direction[1], average_direction[0]))
    thresholds = sorted(_dot2(((start[0] + end[0]) * 0.5, (start[1] + end[1]) * 0.5), axis) for start, end in valid_barriers)

    partitions: list[ClipPartition] = []
    lower: float | None = None
    for index, upper in enumerate([*thresholds, None], 1):
        polygon = list(base_polygon)
        if lower is not None:
            polygon = _clip_polygon_by_half_plane(polygon, axis, lower, keep_less_equal=False)
        if upper is not None:
            polygon = _clip_polygon_by_half_plane(polygon, axis, upper, keep_less_equal=True)
        if len(polygon) >= 3:
            partitions.append(ClipPartition(f"{source_region}_{index}", source_region, polygon, (lower, upper)))
        lower = upper
    return partitions or [ClipPartition(f"{source_region}_1", source_region, base_polygon, (None, None))]


def clip_partitions_from_barriers_boundary(
    source_region: int,
    face_ids: set[int],
    faces: dict[int, FaceGeometry],
    barriers: list[BarrierLine],
) -> list[ClipPartition]:
    x_min, _y_min, x_max, _y_max = face_ids_bounds_xy(face_ids, faces)
    base_polygon, hole_polygons = boundary_polygon_with_holes_xy(face_ids, faces)
    if not barriers:
        return [ClipPartition(f"{source_region}_1", source_region, base_polygon, (None, None), exclude_polygons_model_xy=hole_polygons)]

    region_center_x = (x_min + x_max) * 0.5
    side_polygons = [
        side_polygon_from_barrier(base_polygon, barrier, region_center_x)
        for barrier in barriers
        if abs(barrier[0][0] - barrier[1][0]) + abs(barrier[0][1] - barrier[1][1]) > 1e-9
    ]
    side_polygons = [polygon for polygon in side_polygons if len(polygon) >= 3 and abs(_polygon_area(polygon)) > 1e-6]
    if not side_polygons:
        return [ClipPartition(f"{source_region}_1", source_region, base_polygon, (None, None), exclude_polygons_model_xy=hole_polygons)]

    # 主区仍然是完整面域，但排除两侧由真实轮廓闭合出来的侧区；路径采样点本身只来自面域。
    partitions = [ClipPartition("", source_region, base_polygon, (None, None), exclude_polygons_model_xy=[*side_polygons, *hole_polygons])]
    partitions.extend(ClipPartition("", source_region, polygon, (None, None), exclude_polygons_model_xy=hole_polygons) for polygon in side_polygons)
    partitions.sort(key=lambda partition: _polygon_centroid_x(partition.clip_polygon_model_xy))
    return [
        ClipPartition(
            f"{source_region}_{index}",
            partition.source_region,
            partition.clip_polygon_model_xy,
            partition.barrier_range,
            partition.exclude_polygons_model_xy,
        )
        for index, partition in enumerate(partitions, 1)
    ]


def clip_partitions_from_barriers(
    source_region: int,
    face_ids: set[int],
    faces: dict[int, FaceGeometry],
    barriers: list[BarrierLine],
    padding_ratio: float = 0.04,
    mode: str = PARTITION_MODE_BOUNDARY,
) -> list[ClipPartition]:
    if mode == PARTITION_MODE_SLAB:
        return clip_partitions_from_barriers_slab(source_region, face_ids, faces, barriers, padding_ratio)
    if mode == PARTITION_MODE_BOUNDARY:
        return clip_partitions_from_barriers_boundary(source_region, face_ids, faces, barriers)
    raise ValueError(f"unknown manual partition mode: {mode}")


def clip_partitions_from_picked_polygons(
    source_region: int,
    face_ids: set[int],
    faces: dict[int, FaceGeometry],
    picked_polygons: list[list[Point2]],
) -> list[ClipPartition]:
    _outer_polygon, hole_polygons = boundary_polygon_with_holes_xy(face_ids, faces)
    partitions: list[ClipPartition] = []
    for index, polygon in enumerate(picked_polygons, 1):
        polygon = _dedupe_polygon_points(list(polygon))
        if len(polygon) < 3 or abs(_polygon_area(polygon)) <= 1e-6:
            continue
        partitions.append(
            ClipPartition(
                f"{source_region}_{index}",
                source_region,
                polygon,
                (None, None),
                exclude_polygons_model_xy=hole_polygons,
            )
        )
    return partitions


def manual_clip_manifest_records(
    regions: list[list[int]],
    selected_region_numbers: set[int],
    faces: dict[int, FaceGeometry],
    barriers: list[BarrierLine],
    mode: str = PARTITION_MODE_BOUNDARY,
) -> list[dict]:
    if mode not in {PARTITION_MODE_BOUNDARY, PARTITION_MODE_SLAB}:
        raise ValueError(f"manual line partition mode must be boundary or slab: {mode}")
    records: list[dict] = []
    for region_index, raw_region in enumerate(regions, 1):
        if region_index not in selected_region_numbers:
            continue
        face_ids = {int(face_id) for face_id in raw_region}
        partitions = clip_partitions_from_barriers(region_index, face_ids, faces, barriers, mode=mode)
        records.append(
            {
                "original_region": region_index,
                "reason": "manual_uv_boundary_clip" if mode == PARTITION_MODE_BOUNDARY else "manual_uv_slab_clip",
                "partition_mode": mode,
                "output_patch_count": len(partitions),
                "patches": [
                    {
                        "label": partition.label,
                        "source_region": partition.source_region,
                        "partition_mode": mode,
                        "clip_space": "model_xy",
                        "clip_polygon": [list(point) for point in partition.clip_polygon_model_xy],
                        "exclude_polygons": [
                            [list(point) for point in polygon]
                            for polygon in (partition.exclude_polygons_model_xy or [])
                        ],
                        "barrier_range": list(partition.barrier_range),
                        "face_count": len(face_ids),
                        "face_ids": sorted(face_ids),
                    }
                    for partition in partitions
                ],
            }
        )
    return records


def manual_pick_manifest_records(
    regions: list[list[int]],
    selected_region_numbers: set[int],
    faces: dict[int, FaceGeometry],
    picked_polygons: list[list[Point2]],
) -> list[dict]:
    records: list[dict] = []
    for region_index, raw_region in enumerate(regions, 1):
        if region_index not in selected_region_numbers:
            continue
        face_ids = {int(face_id) for face_id in raw_region}
        partitions = clip_partitions_from_picked_polygons(region_index, face_ids, faces, picked_polygons)
        records.append(
            {
                "original_region": region_index,
                "reason": "manual_uv_pick_clip",
                "partition_mode": PARTITION_MODE_PICK,
                "output_patch_count": len(partitions),
                "patches": [
                    {
                        "label": partition.label,
                        "source_region": partition.source_region,
                        "partition_mode": PARTITION_MODE_PICK,
                        "clip_space": "model_xy",
                        "clip_polygon": [list(point) for point in partition.clip_polygon_model_xy],
                        "exclude_polygons": [
                            [list(point) for point in polygon]
                            for polygon in (partition.exclude_polygons_model_xy or [])
                        ],
                        "barrier_range": list(partition.barrier_range),
                        "face_count": len(face_ids),
                        "face_ids": sorted(face_ids),
                    }
                    for partition in partitions
                ],
            }
        )
    return records
