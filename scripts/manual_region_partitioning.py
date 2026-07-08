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


def clip_partitions_from_barriers(
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
    if not barriers:
        return [ClipPartition(f"{source_region}_1", source_region, base_polygon, (None, None))]

    # 用户画的是分界线。这里把多条分界线看成同一组近似平行的 UV 裁剪边界，
    # 沿分界线法向排序后生成 slab polygon，轨迹采样再用这些 polygon 做裁剪。
    directions = [(end[0] - start[0], end[1] - start[1]) for start, end in barriers]
    average_direction = _normalize2((sum(item[0] for item in directions), sum(item[1] for item in directions)))
    axis = _normalize2((-average_direction[1], average_direction[0]))
    thresholds = sorted(_dot2(((start[0] + end[0]) * 0.5, (start[1] + end[1]) * 0.5), axis) for start, end in barriers)

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


def manual_clip_manifest_records(
    regions: list[list[int]],
    selected_region_numbers: set[int],
    faces: dict[int, FaceGeometry],
    barriers: list[BarrierLine],
) -> list[dict]:
    records: list[dict] = []
    for region_index, raw_region in enumerate(regions, 1):
        if region_index not in selected_region_numbers:
            continue
        face_ids = {int(face_id) for face_id in raw_region}
        partitions = clip_partitions_from_barriers(region_index, face_ids, faces, barriers)
        records.append(
            {
                "original_region": region_index,
                "reason": "manual_uv_clip",
                "output_patch_count": len(partitions),
                "patches": [
                    {
                        "label": partition.label,
                        "source_region": partition.source_region,
                        "clip_space": "model_xy",
                        "clip_polygon": [list(point) for point in partition.clip_polygon_model_xy],
                        "barrier_range": list(partition.barrier_range),
                        "face_count": len(face_ids),
                        "face_ids": sorted(face_ids),
                    }
                    for partition in partitions
                ],
            }
        )
    return records
