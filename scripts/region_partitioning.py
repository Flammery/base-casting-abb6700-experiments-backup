from __future__ import annotations

from collections import defaultdict, deque
from dataclasses import dataclass, field
import math
import sys
from pathlib import Path

EXPERIMENT_DIR = Path(__file__).resolve().parents[1]
ROOT = EXPERIMENT_DIR.parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from robot_studio_qt.core.geometry import cross, dot, normalize
from robot_studio_qt.path_planning.mesh_raster import MeshTriangle, average_normal

Vector3 = tuple[float, float, float]


@dataclass(frozen=True)
class PartitionSettings:
    hard_edge_angle_deg: float = 15.0
    planar_normal_deg: float = 8.0
    planar_rms_mm: float = 2.0
    scan_spacing_mm: float = 50.0
    interval_overlap_ratio: float = 0.2
    neck_width_ratio: float = 0.25
    min_neck_lines: int = 2
    min_patch_faces: int = 20
    min_patch_area_ratio: float = 0.05
    point_quantization: float = 1_000_000.0
    planar_seed_min_faces: int = 6
    left_cut_x: float | None = None
    right_cut_x: float | None = None
    left_angles: tuple[int, ...] = (0,)
    right_angles: tuple[int, ...] = (180,)
    center_angles: tuple[int, ...] = (270,)
    max_base_x_span_mm: float = 1000.0
    max_base_y_span_mm: float = 2000.0
    turn_histogram_bin_mm: float = 75.0
    max_size_split_depth: int = 8


@dataclass(frozen=True)
class FaceGeometry:
    face_id: int
    triangles: tuple[MeshTriangle, ...]
    area: float
    normal: Vector3
    centroid: Vector3


@dataclass(frozen=True)
class RegionPatch:
    original_region: int
    label: str
    kind: str
    face_ids: list[int]
    area: float
    source: str
    turn_zone: str = "unknown"
    allowed_angles: tuple[int, ...] = ()
    nominal_angle: int | None = None
    surface_class: str = "unknown"
    split_reason: str = ""
    collision_status: str = "not_evaluated"


@dataclass(frozen=True)
class RegionPartitionRecord:
    original_region: int
    input_face_count: int
    output_patch_count: int
    unchanged: bool
    reason: str
    patches: list[RegionPatch] = field(default_factory=list)


@dataclass(frozen=True)
class PartitionedRegions:
    regions: list[list[int]]
    records: list[RegionPartitionRecord]


@dataclass(frozen=True)
class _IntervalNode:
    index: int
    line_index: int
    u_min: float
    u_max: float
    face_ids: frozenset[int]

    @property
    def width(self) -> float:
        return abs(self.u_max - self.u_min)


@dataclass
class _PatchCandidate:
    kind: str
    face_ids: set[int]
    source: str
    turn_zone: str
    allowed_angles: tuple[int, ...]
    nominal_angle: int | None
    surface_class: str
    split_reason: str


def point_key(point: Vector3, quantization: float = 1_000_000.0) -> tuple[int, int, int]:
    return tuple(round(value * quantization) for value in point)  # type: ignore[return-value]


def angle_between_degrees(a: Vector3, b: Vector3) -> float:
    value = max(-1.0, min(1.0, dot(normalize(a), normalize(b))))
    return math.degrees(math.acos(value))


def triangle_centroid(triangle: MeshTriangle) -> Vector3:
    return tuple(sum(point[index] for point in triangle.points) / 3.0 for index in range(3))  # type: ignore[return-value]


def weighted_centroid(faces: list[FaceGeometry]) -> Vector3:
    total = sum(face.area for face in faces)
    if total <= 1e-12:
        return (0.0, 0.0, 0.0)
    return (
        sum(face.centroid[0] * face.area for face in faces) / total,
        sum(face.centroid[1] * face.area for face in faces) / total,
        sum(face.centroid[2] * face.area for face in faces) / total,
    )


def weighted_normal(faces: list[FaceGeometry]) -> Vector3:
    total = (
        sum(face.normal[0] * face.area for face in faces),
        sum(face.normal[1] * face.area for face in faces),
        sum(face.normal[2] * face.area for face in faces),
    )
    return normalize(total)


def face_geometries_from_triangles(triangles: list[MeshTriangle]) -> dict[int, FaceGeometry]:
    by_face: dict[int, list[MeshTriangle]] = defaultdict(list)
    for triangle in triangles:
        by_face[int(triangle.face_id)].append(triangle)

    faces: dict[int, FaceGeometry] = {}
    for face_id, face_triangles in by_face.items():
        area = sum(triangle.area for triangle in face_triangles)
        if area <= 1e-12:
            continue
        centroid = (
            sum(triangle_centroid(triangle)[0] * triangle.area for triangle in face_triangles) / area,
            sum(triangle_centroid(triangle)[1] * triangle.area for triangle in face_triangles) / area,
            sum(triangle_centroid(triangle)[2] * triangle.area for triangle in face_triangles) / area,
        )
        normal = normalize(
            (
                sum(triangle.normal[0] * triangle.area for triangle in face_triangles),
                sum(triangle.normal[1] * triangle.area for triangle in face_triangles),
                sum(triangle.normal[2] * triangle.area for triangle in face_triangles),
            )
        )
        faces[face_id] = FaceGeometry(face_id, tuple(face_triangles), area, normal, centroid)
    return faces


def build_face_adjacency(faces: dict[int, FaceGeometry], settings: PartitionSettings) -> dict[int, set[int]]:
    edge_faces: dict[tuple[tuple[int, int, int], tuple[int, int, int]], set[int]] = defaultdict(set)
    for face in faces.values():
        for triangle in face.triangles:
            points = triangle.points
            for start, end in ((points[0], points[1]), (points[1], points[2]), (points[2], points[0])):
                key = tuple(sorted((point_key(start, settings.point_quantization), point_key(end, settings.point_quantization))))
                edge_faces[key].add(face.face_id)  # type: ignore[arg-type]

    adjacency = {face_id: set() for face_id in faces}
    for shared_faces in edge_faces.values():
        ids = sorted(shared_faces)
        if len(ids) < 2:
            continue
        for index, face_id in enumerate(ids):
            for other_id in ids[index + 1 :]:
                adjacency[face_id].add(other_id)
                adjacency[other_id].add(face_id)
    return adjacency


def plane_rms(faces: list[FaceGeometry], origin: Vector3, normal: Vector3) -> float:
    total_area = sum(face.area for face in faces)
    if total_area <= 1e-12:
        return 0.0
    error = 0.0
    for face in faces:
        distance = dot(
            (
                face.centroid[0] - origin[0],
                face.centroid[1] - origin[1],
                face.centroid[2] - origin[2],
            ),
            normal,
        )
        error += distance * distance * face.area
    return math.sqrt(error / total_area)


def is_planar_patch(face_ids: set[int], faces: dict[int, FaceGeometry], settings: PartitionSettings) -> bool:
    patch_faces = [faces[face_id] for face_id in face_ids]
    normal = weighted_normal(patch_faces)
    origin = weighted_centroid(patch_faces)
    max_angle = max(angle_between_degrees(faces[face_id].normal, normal) for face_id in face_ids)
    return max_angle <= settings.planar_normal_deg and plane_rms(patch_faces, origin, normal) <= settings.planar_rms_mm


def connected_components(face_ids: set[int], adjacency: dict[int, set[int]]) -> list[set[int]]:
    remaining = set(face_ids)
    components: list[set[int]] = []
    while remaining:
        seed = min(remaining)
        queue = deque([seed])
        remaining.remove(seed)
        component = {seed}
        while queue:
            current = queue.popleft()
            for neighbor in sorted(adjacency.get(current, set())):
                if neighbor not in remaining:
                    continue
                remaining.remove(neighbor)
                component.add(neighbor)
                queue.append(neighbor)
        components.append(component)
    return components


def hard_edge_adjacency(face_ids: set[int], faces: dict[int, FaceGeometry], adjacency: dict[int, set[int]], settings: PartitionSettings) -> dict[int, set[int]]:
    filtered = {face_id: set() for face_id in face_ids}
    for face_id in face_ids:
        for neighbor in adjacency.get(face_id, set()) & face_ids:
            if angle_between_degrees(faces[face_id].normal, faces[neighbor].normal) <= settings.hard_edge_angle_deg:
                filtered[face_id].add(neighbor)
    return filtered


def _median_float(values: list[float]) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return (ordered[middle - 1] + ordered[middle]) * 0.5


def _angle_tuple(values: tuple[int, ...]) -> tuple[int, ...]:
    return tuple(int(value) % 360 for value in values)


def allowed_angles_for_zone(zone: str, settings: PartitionSettings) -> tuple[int, ...]:
    if zone == "left":
        return _angle_tuple(settings.left_angles)
    if zone == "right":
        return _angle_tuple(settings.right_angles)
    return _angle_tuple(settings.center_angles)


def nominal_angle_for_zone(zone: str, settings: PartitionSettings) -> int | None:
    values = allowed_angles_for_zone(zone, settings)
    return values[0] if values else None


def _rotate_xy(point: Vector3, angle_degrees: float) -> tuple[float, float]:
    angle = math.radians(angle_degrees)
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    return point[0] * cos_a - point[1] * sin_a, point[0] * sin_a + point[1] * cos_a


def _base_axis_value(point: Vector3, angle_degrees: float, axis: str) -> float:
    x_value, y_value = _rotate_xy(point, angle_degrees)
    return x_value if axis == "x" else y_value


def _axis_span(face_ids: set[int], faces: dict[int, FaceGeometry], angle_degrees: float, axis: str) -> float:
    values = [_base_axis_value(vertex, angle_degrees, axis) for face_id in face_ids for triangle in faces[face_id].triangles for vertex in triangle.points]
    if not values:
        return 0.0
    return max(values) - min(values)


def _find_histogram_valley(
    buckets: list[float],
    x_min: float,
    bin_width: float,
    start_ratio: float,
    end_ratio: float,
) -> float:
    if not buckets:
        return x_min
    start = max(0, min(len(buckets) - 1, int(len(buckets) * start_ratio)))
    end = max(start, min(len(buckets) - 1, int(len(buckets) * end_ratio)))
    candidates: list[tuple[float, float, int]] = []
    for index in range(start, end + 1):
        window = buckets[max(0, index - 2) : min(len(buckets), index + 3)]
        local_median = _median([value for value in window if value > 0.0])
        score = buckets[index] / max(local_median, 1e-9)
        candidates.append((score, buckets[index], index))
    _score, _area, best_index = min(candidates)
    return x_min + (best_index + 0.5) * bin_width


def auto_turn_zone_cuts(face_ids: set[int], faces: dict[int, FaceGeometry], settings: PartitionSettings) -> tuple[float, float]:
    centroids = [faces[face_id].centroid[0] for face_id in face_ids if face_id in faces]
    if not centroids:
        return (0.0, 0.0)
    x_min = min(centroids)
    x_max = max(centroids)
    span = max(x_max - x_min, 1e-9)
    bin_width = max(settings.turn_histogram_bin_mm, span / 80.0, 1e-6)
    bin_count = max(12, int(math.ceil(span / bin_width)))
    bin_width = span / bin_count
    buckets = [0.0 for _ in range(bin_count)]
    for face_id in face_ids:
        face = faces[face_id]
        index = min(bin_count - 1, max(0, int((face.centroid[0] - x_min) / bin_width)))
        buckets[index] += face.area

    # 柱子把左右和中间隔开后，X 投影面积会在两个过渡位置形成“谷”。
    # 默认自动找谷；如果现场验证发现某个铸件不稳定，可用 CLI cut 参数覆盖。
    left_cut = settings.left_cut_x if settings.left_cut_x is not None else _find_histogram_valley(buckets, x_min, bin_width, 0.25, 0.45)
    right_cut = settings.right_cut_x if settings.right_cut_x is not None else _find_histogram_valley(buckets, x_min, bin_width, 0.55, 0.75)
    if left_cut > right_cut:
        left_cut, right_cut = right_cut, left_cut
    return left_cut, right_cut


def split_by_turn_zone(face_ids: set[int], faces: dict[int, FaceGeometry], adjacency: dict[int, set[int]], settings: PartitionSettings) -> list[tuple[str, set[int]]]:
    left_cut, right_cut = auto_turn_zone_cuts(face_ids, faces, settings)
    buckets = {"left": set(), "center": set(), "right": set()}
    for face_id in face_ids:
        x_value = faces[face_id].centroid[0]
        if x_value < left_cut:
            buckets["left"].add(face_id)
        elif x_value > right_cut:
            buckets["right"].add(face_id)
        else:
            buckets["center"].add(face_id)

    results: list[tuple[str, set[int]]] = []
    for zone in ("left", "center", "right"):
        zone_faces = buckets[zone]
        if not zone_faces:
            continue
        zone_adjacency = {face_id: adjacency.get(face_id, set()) & zone_faces for face_id in zone_faces}
        for component in connected_components(zone_faces, zone_adjacency):
            results.append((zone, component))
    return results or [("center", set(face_ids))]


def classify_main_plane_and_slope(face_ids: set[int], faces: dict[int, FaceGeometry], adjacency: dict[int, set[int]], settings: PartitionSettings) -> list[tuple[str, str, set[int]]]:
    patch_faces = [faces[face_id] for face_id in face_ids]
    if not patch_faces:
        return []
    origin = weighted_centroid(patch_faces)
    normal = weighted_normal(patch_faces)
    main_faces: set[int] = set()
    slope_faces: set[int] = set()
    for face_id in face_ids:
        face = faces[face_id]
        distance = abs(dot((face.centroid[0] - origin[0], face.centroid[1] - origin[1], face.centroid[2] - origin[2]), normal))
        angle = angle_between_degrees(face.normal, normal)
        if angle <= settings.planar_normal_deg and distance <= settings.planar_rms_mm:
            main_faces.add(face_id)
        else:
            slope_faces.add(face_id)

    total_area = sum(faces[face_id].area for face_id in face_ids)
    main_area = sum(faces[face_id].area for face_id in main_faces)
    minimum_support = min(settings.planar_seed_min_faces, max(1, len(face_ids)))
    if not main_faces or len(main_faces) < minimum_support or main_area < total_area * settings.min_patch_area_ratio:
        main_faces.clear()
        slope_faces = set(face_ids)

    # 这里保留 kind=planar/curved 的兼容语义，但实际含义改为主平面/斜面。
    # 先按姿态区切开，再做该判断，避免把左右和中间不同可达方向混在一起。
    results: list[tuple[str, str, set[int]]] = []
    for kind, surface_class, subset in (("planar", "main_plane", main_faces), ("curved", "slope", slope_faces)):
        if not subset:
            continue
        subset_adjacency = {face_id: adjacency.get(face_id, set()) & subset for face_id in subset}
        for component in connected_components(subset, subset_adjacency):
            results.append((kind, surface_class, component))
    return results or [("curved", "slope", set(face_ids))]


def split_patch_by_base_window(
    face_ids: set[int],
    faces: dict[int, FaceGeometry],
    adjacency: dict[int, set[int]],
    nominal_angle: int | None,
    settings: PartitionSettings,
    depth: int = 0,
) -> list[tuple[set[int], str]]:
    if nominal_angle is None or len(face_ids) < settings.min_patch_faces * 2 or depth >= settings.max_size_split_depth:
        return [(set(face_ids), "size_ok")]
    x_span = _axis_span(face_ids, faces, float(nominal_angle), "x")
    y_span = _axis_span(face_ids, faces, float(nominal_angle), "y")
    x_limit = max(settings.max_base_x_span_mm, 1e-9)
    y_limit = max(settings.max_base_y_span_mm, 1e-9)
    if x_span <= x_limit and y_span <= y_limit:
        return [(set(face_ids), "size_ok")]

    split_axis = "x" if x_span / x_limit >= y_span / y_limit else "y"
    values = [(face_id, _base_axis_value(faces[face_id].centroid, float(nominal_angle), split_axis)) for face_id in face_ids]
    cut_value = _median_float([value for _face_id, value in values])
    lower = {face_id for face_id, value in values if value <= cut_value}
    upper = set(face_ids) - lower
    if not lower or not upper:
        return [(set(face_ids), f"size_unsplit_{split_axis}")]

    # 尺寸切分使用代表转角下的 base X/Y，而不是模型坐标。
    # 切完后再按真实 mesh 邻接拆连通分量，避免一刀跨过孔洞或断开的岛。
    split_results: list[tuple[set[int], str]] = []
    for subset in (lower, upper):
        subset_adjacency = {face_id: adjacency.get(face_id, set()) & subset for face_id in subset}
        for component in connected_components(subset, subset_adjacency):
            for child, reason in split_patch_by_base_window(component, faces, adjacency, nominal_angle, settings, depth + 1):
                if reason == "size_ok":
                    split_results.append((child, f"size_split_base_{split_axis}"))
                else:
                    split_results.append((child, reason))
    return split_results or [(set(face_ids), f"size_unsplit_{split_axis}")]


def _cleanup_small_candidates(
    candidates: list[_PatchCandidate],
    faces: dict[int, FaceGeometry],
    adjacency: dict[int, set[int]],
    original_face_ids: set[int],
    settings: PartitionSettings,
) -> list[_PatchCandidate]:
    if len(candidates) <= 1:
        return candidates
    total_area = sum(faces[face_id].area for face_id in original_face_ids)
    patches = [_PatchCandidate(candidate.kind, set(candidate.face_ids), candidate.source, candidate.turn_zone, candidate.allowed_angles, candidate.nominal_angle, candidate.surface_class, candidate.split_reason) for candidate in candidates if candidate.face_ids]

    changed = True
    while changed and len(patches) > 1:
        changed = False
        for index, candidate in list(enumerate(patches)):
            area = sum(faces[face_id].area for face_id in candidate.face_ids)
            if len(candidate.face_ids) >= settings.min_patch_faces and area >= total_area * settings.min_patch_area_ratio:
                continue
            target_scores: list[tuple[int, float, int]] = []
            for other_index, other in enumerate(patches):
                if other_index == index:
                    continue
                shared = sum(1 for face_id in candidate.face_ids for neighbor in adjacency.get(face_id, set()) if neighbor in other.face_ids)
                other_area = sum(faces[face_id].area for face_id in other.face_ids)
                target_scores.append((shared, other_area, other_index))
            if not target_scores:
                continue
            _shared, _area, target_index = max(target_scores)
            target = patches[target_index]
            target.face_ids.update(candidate.face_ids)
            reason_parts = [part for part in target.split_reason.split("+") if part]
            if "merged_small" not in reason_parts:
                reason_parts.append("merged_small")
            target.split_reason = "+".join(reason_parts)
            # 小片合并到相邻大区时，保留大区的姿态区和角度组，避免小噪声改变后续加工方向。
            del patches[index]
            changed = True
            break
    return patches


def partition_by_curvature(face_ids: set[int], faces: dict[int, FaceGeometry], adjacency: dict[int, set[int]], settings: PartitionSettings) -> list[tuple[str, set[int]]]:
    planar_faces = dominant_planar_faces(face_ids, faces, settings)
    if not planar_faces:
        return [("curved", set(face_ids))]
    curved_faces = set(face_ids) - planar_faces
    patches: list[tuple[str, set[int]]] = []
    if planar_faces:
        patches.append(("planar", planar_faces))
    if curved_faces:
        patches.append(("curved", curved_faces))
    return patches


def dominant_planar_faces(face_ids: set[int], faces: dict[int, FaceGeometry], settings: PartitionSettings) -> set[int]:
    """Classify the dominant stable plane once, leaving all non-plane faces curved.

    The previous local region growing could create several same-kind planar patches.
    Here every face is judged against one dominant plane candidate, so the first
    stage only separates planar from curved.
    """
    if not face_ids:
        return set()
    candidates: list[tuple[float, int, set[int]]] = []
    for seed_id in face_ids:
        seed = faces[seed_id]
        support: set[int] = set()
        for face_id in face_ids:
            face = faces[face_id]
            distance = abs(
                dot(
                    (
                        face.centroid[0] - seed.centroid[0],
                        face.centroid[1] - seed.centroid[1],
                        face.centroid[2] - seed.centroid[2],
                    ),
                    seed.normal,
                )
            )
            if angle_between_degrees(face.normal, seed.normal) <= settings.planar_normal_deg and distance <= settings.planar_rms_mm:
                support.add(face_id)
        area = sum(faces[face_id].area for face_id in support)
        candidates.append((area, len(support), support))
    _area, count, best = max(candidates, key=lambda item: (item[0], item[1]))
    minimum_support = min(settings.planar_seed_min_faces, max(1, len(face_ids) // 2))
    if count < minimum_support:
        return set()
    return best


def can_add_to_planar_patch(candidate: int, patch: set[int], faces: dict[int, FaceGeometry], settings: PartitionSettings) -> bool:
    patch_faces = [faces[face_id] for face_id in patch]
    normal = weighted_normal(patch_faces)
    if angle_between_degrees(faces[candidate].normal, normal) > settings.planar_normal_deg:
        return False
    next_patch = set(patch)
    next_patch.add(candidate)
    return is_planar_patch(next_patch, faces, settings)


def grow_local_planar_patch(seed: int, available: set[int], faces: dict[int, FaceGeometry], adjacency: dict[int, set[int]], settings: PartitionSettings) -> set[int]:
    patch = {seed}
    changed = True
    while changed:
        changed = False
        candidates = sorted(
            {neighbor for face_id in patch for neighbor in adjacency.get(face_id, set()) if neighbor in available and neighbor not in patch},
            key=lambda face_id: (-faces[face_id].area, face_id),
        )
        for candidate in candidates:
            if can_add_to_planar_patch(candidate, patch, faces, settings):
                patch.add(candidate)
                changed = True
    return patch


def merge_adjacent_patches(patches: list[set[int]], faces: dict[int, FaceGeometry], adjacency: dict[int, set[int]], settings: PartitionSettings) -> list[set[int]]:
    changed = True
    while changed:
        changed = False
        for index, patch in enumerate(patches):
            for other_index in range(index + 1, len(patches)):
                other = patches[other_index]
                if not any(neighbor in other for face_id in patch for neighbor in adjacency.get(face_id, set())):
                    continue
                merged = patch | other
                if not is_planar_patch(merged, faces, settings):
                    continue
                patches[index] = merged
                del patches[other_index]
                changed = True
                break
            if changed:
                break
    return patches


def partition_component_by_local_planarity(component: set[int], faces: dict[int, FaceGeometry], adjacency: dict[int, set[int]], settings: PartitionSettings) -> list[tuple[str, set[int]]]:
    if is_planar_patch(component, faces, settings):
        return [("planar", component)]

    remaining = set(component)
    planar_patches: list[set[int]] = []
    while remaining:
        seed = max(remaining, key=lambda face_id: (faces[face_id].area, len(adjacency.get(face_id, set()))))
        patch = grow_local_planar_patch(seed, remaining, faces, adjacency, settings)
        if len(patch) < settings.planar_seed_min_faces:
            remaining.remove(seed)
            continue
        planar_patches.append(patch)
        remaining -= patch

    planar_patches = merge_adjacent_patches(planar_patches, faces, adjacency, settings)
    planar_faces = set().union(*planar_patches) if planar_patches else set()
    total_area = sum(faces[face_id].area for face_id in component)

    accepted_planar: list[set[int]] = []
    rejected_planar: set[int] = set()
    for patch in planar_patches:
        area = sum(faces[face_id].area for face_id in patch)
        if len(patch) >= settings.min_patch_faces and area >= total_area * settings.min_patch_area_ratio:
            accepted_planar.append(patch)
        else:
            rejected_planar.update(patch)

    curved_faces = (component - planar_faces) | rejected_planar
    results: list[tuple[str, set[int]]] = [("planar", patch) for patch in accepted_planar]
    if curved_faces:
        curved_adjacency = {face_id: adjacency.get(face_id, set()) & curved_faces for face_id in curved_faces}
        for curved_component in connected_components(curved_faces, curved_adjacency):
            results.append(("curved", curved_component))
    return results or [("curved", component)]


def boundary_uv_axes(face_ids: set[int], faces: dict[int, FaceGeometry]) -> tuple[Vector3, Vector3]:
    triangles = [triangle for face_id in face_ids for triangle in faces[face_id].triangles]
    normal = average_normal(triangles)
    candidates: list[tuple[float, Vector3]] = []
    edge_map: dict[tuple[tuple[int, int, int], tuple[int, int, int]], list[tuple[Vector3, Vector3]]] = defaultdict(list)
    for triangle in triangles:
        points = triangle.points
        for start, end in ((points[0], points[1]), (points[1], points[2]), (points[2], points[0])):
            key = tuple(sorted((point_key(start), point_key(end))))
            edge_map[key].append((start, end))  # type: ignore[arg-type]
    for entries in edge_map.values():
        if len(entries) != 1:
            continue
        start, end = entries[0]
        vector = (end[0] - start[0], end[1] - start[1], end[2] - start[2])
        projected = (
            vector[0] - dot(vector, normal) * normal[0],
            vector[1] - dot(vector, normal) * normal[1],
            vector[2] - dot(vector, normal) * normal[2],
        )
        length_sq = dot(projected, projected)
        if length_sq > 1e-8:
            candidates.append((math.sqrt(length_sq), normalize(projected)))

    if not candidates:
        reference = (1.0, 0.0, 0.0) if abs(normal[0]) < 0.9 else (0.0, 1.0, 0.0)
        primary = normalize(cross(normal, reference))
    else:
        def alignment_score(axis: Vector3) -> float:
            return sum(length * abs(dot(axis, candidate_axis)) ** 4 for length, candidate_axis in candidates)

        primary = max((axis for _length, axis in candidates), key=alignment_score)
    secondary = normalize(cross(normal, primary), fallback=(0.0, 1.0, 0.0))
    return primary, secondary


def _project_point(point: Vector3, origin: Vector3, u_axis: Vector3, v_axis: Vector3) -> tuple[float, float]:
    relative = (point[0] - origin[0], point[1] - origin[1], point[2] - origin[2])
    return dot(relative, u_axis), dot(relative, v_axis)


def scanline_interval_nodes(face_ids: set[int], faces: dict[int, FaceGeometry], u_axis: Vector3, v_axis: Vector3, settings: PartitionSettings) -> list[_IntervalNode]:
    patch_faces = [faces[face_id] for face_id in face_ids]
    origin = weighted_centroid(patch_faces)
    projected: list[tuple[int, tuple[tuple[float, float], tuple[float, float], tuple[float, float]]]] = []
    for face_id in face_ids:
        for triangle in faces[face_id].triangles:
            projected.append((face_id, tuple(_project_point(point, origin, u_axis, v_axis) for point in triangle.points)))  # type: ignore[arg-type]

    if not projected:
        return []
    all_v = [point[1] for _face_id, points in projected for point in points]
    v_min = min(all_v)
    v_max = max(all_v)
    if v_max < v_min:
        return []
    spacing = max(settings.scan_spacing_mm, 1e-6)
    line_values: list[float] = []
    current = v_min
    while current <= v_max + 1e-9:
        line_values.append(current)
        current += spacing
    if not line_values or abs(line_values[-1] - v_max) > spacing * 0.25:
        line_values.append(v_max)

    nodes: list[_IntervalNode] = []
    for line_index, v in enumerate(line_values):
        raw: list[tuple[float, float, int]] = []
        for face_id, points in projected:
            intersections: list[float] = []
            for index in range(3):
                a = points[index]
                b = points[(index + 1) % 3]
                if abs(a[1] - b[1]) <= 1e-12:
                    continue
                if v < min(a[1], b[1]) or v >= max(a[1], b[1]):
                    continue
                ratio = (v - a[1]) / (b[1] - a[1])
                intersections.append(a[0] + ratio * (b[0] - a[0]))
            if len(intersections) >= 2:
                raw.append((min(intersections), max(intersections), face_id))
        raw.sort()
        merged: list[tuple[float, float, set[int]]] = []
        for start, end, face_id in raw:
            if end < start:
                start, end = end, start
            if not merged or start > merged[-1][1] + 1e-6:
                merged.append((start, end, {face_id}))
            else:
                old_start, old_end, old_faces = merged[-1]
                old_faces.add(face_id)
                merged[-1] = (old_start, max(old_end, end), old_faces)
        for start, end, interval_faces in merged:
            if end - start > 1e-6:
                nodes.append(_IntervalNode(len(nodes), line_index, start, end, frozenset(interval_faces)))
    return nodes


def _line_widths(nodes: list[_IntervalNode]) -> dict[int, float]:
    widths: dict[int, float] = defaultdict(float)
    for node in nodes:
        widths[node.line_index] += node.width
    return widths


def _median(values: list[float]) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return (ordered[middle - 1] + ordered[middle]) * 0.5


def neck_barrier_lines(nodes: list[_IntervalNode], settings: PartitionSettings) -> set[int]:
    widths = _line_widths(nodes)
    if len(widths) < max(settings.min_neck_lines + 2, 4):
        return set()
    line_ids = sorted(widths)
    global_median = _median([widths[value] for value in line_ids])
    interior_line_ids = set(line_ids[2:-2]) if len(line_ids) > 4 else set(line_ids)
    candidates: set[int] = set()
    strong_candidates: set[int] = set()
    for offset, line_id in enumerate(line_ids):
        if line_id not in interior_line_ids:
            continue
        window_ids = line_ids[max(0, offset - 3) : min(len(line_ids), offset + 4)]
        local_median = _median([widths[value] for value in window_ids])
        if local_median > 1e-9 and widths[line_id] <= local_median * settings.neck_width_ratio:
            candidates.add(line_id)
        if global_median > 1e-9 and widths[line_id] <= global_median * settings.neck_width_ratio:
            candidates.add(line_id)
            strong_candidates.add(line_id)

    barriers: set[int] = set()
    run: list[int] = []
    previous: int | None = None
    for line_id in sorted(candidates):
        if previous is None or line_id == previous + 1:
            run.append(line_id)
        else:
            if len(run) >= settings.min_neck_lines:
                barriers.update(run)
            run = [line_id]
        previous = line_id
    if len(run) >= settings.min_neck_lines:
        barriers.update(run)
    barriers.update(strong_candidates)
    return barriers


def interval_overlap_ratio(a: _IntervalNode, b: _IntervalNode) -> float:
    overlap = min(a.u_max, b.u_max) - max(a.u_min, b.u_min)
    if overlap <= 0.0:
        return 0.0
    return overlap / max(min(a.width, b.width), 1e-9)


def interval_components(nodes: list[_IntervalNode], barriers: set[int], settings: PartitionSettings) -> list[set[int]]:
    by_line: dict[int, list[_IntervalNode]] = defaultdict(list)
    for node in nodes:
        by_line[node.line_index].append(node)
    adjacency: dict[int, set[int]] = {node.index: set() for node in nodes}
    for line_id in sorted(by_line):
        if line_id in barriers or line_id + 1 in barriers:
            continue
        for node in by_line[line_id]:
            for other in by_line.get(line_id + 1, []):
                if interval_overlap_ratio(node, other) >= settings.interval_overlap_ratio:
                    adjacency[node.index].add(other.index)
                    adjacency[other.index].add(node.index)
    return connected_components(set(adjacency), adjacency)


def _component_face_sets(nodes: list[_IntervalNode], components: list[set[int]], original_face_ids: set[int]) -> list[set[int]]:
    node_by_index = {node.index: node for node in nodes}
    face_sets: list[set[int]] = []
    for component in components:
        faces = {face_id for index in component for face_id in node_by_index[index].face_ids}
        if faces:
            face_sets.append(faces & original_face_ids)
    return [faces for faces in face_sets if faces]


def _merge_small_patches(face_sets: list[set[int]], faces: dict[int, FaceGeometry], adjacency: dict[int, set[int]], original_face_ids: set[int], settings: PartitionSettings) -> list[set[int]]:
    if not face_sets:
        return [set(original_face_ids)]
    total_area = sum(faces[face_id].area for face_id in original_face_ids)
    assigned = set().union(*face_sets)
    missing = original_face_ids - assigned
    if missing:
        largest = max(range(len(face_sets)), key=lambda index: sum(faces[face_id].area for face_id in face_sets[index]))
        face_sets[largest].update(missing)

    changed = True
    while changed and len(face_sets) > 1:
        changed = False
        for index, face_set in list(enumerate(face_sets)):
            area = sum(faces[face_id].area for face_id in face_set)
            if len(face_set) >= settings.min_patch_faces and area >= total_area * settings.min_patch_area_ratio:
                continue
            target_scores: list[tuple[int, float, int]] = []
            for other_index, other_set in enumerate(face_sets):
                if other_index == index:
                    continue
                shared_edges = sum(1 for face_id in face_set for neighbor in adjacency.get(face_id, set()) if neighbor in other_set)
                other_area = sum(faces[face_id].area for face_id in other_set)
                target_scores.append((shared_edges, other_area, other_index))
            if not target_scores:
                continue
            _shared, _area, target = max(target_scores)
            face_sets[target].update(face_set)
            del face_sets[index]
            changed = True
            break
    unique: list[set[int]] = []
    seen: set[frozenset[int]] = set()
    for face_set in face_sets:
        key = frozenset(face_set)
        if key and key not in seen:
            seen.add(key)
            unique.append(set(face_set))
    return unique or [set(original_face_ids)]


def cleanup_small_output_patches(
    raw_patches: list[tuple[str, set[int], str]],
    faces: dict[int, FaceGeometry],
    adjacency: dict[int, set[int]],
    original_face_ids: set[int],
    settings: PartitionSettings,
) -> list[tuple[str, set[int], str]]:
    if len(raw_patches) <= 1:
        return raw_patches
    total_area = sum(faces[face_id].area for face_id in original_face_ids)
    patches = [(kind, set(patch), source) for kind, patch, source in raw_patches if patch]

    changed = True
    while changed and len(patches) > 1:
        changed = False
        for index, (kind, patch, source) in list(enumerate(patches)):
            area = sum(faces[face_id].area for face_id in patch)
            if len(patch) >= settings.min_patch_faces and area >= total_area * settings.min_patch_area_ratio:
                continue
            candidates: list[tuple[int, float, int]] = []
            for other_index, (_other_kind, other_patch, _other_source) in enumerate(patches):
                if other_index == index:
                    continue
                shared = sum(1 for face_id in patch for neighbor in adjacency.get(face_id, set()) if neighbor in other_patch)
                other_area = sum(faces[face_id].area for face_id in other_patch)
                candidates.append((shared, other_area, other_index))
            if not candidates:
                continue
            _shared, _area, target_index = max(candidates)
            target_kind, target_patch, target_source = patches[target_index]
            target_patch.update(patch)
            combined_kind = target_kind if target_kind == kind else target_kind
            combined_source = target_source if target_source == source else f"{target_source}+merged_small"
            patches[target_index] = (combined_kind, target_patch, combined_source)
            del patches[index]
            changed = True
            break
    return patches


def split_planar_patch_by_neck(face_ids: set[int], faces: dict[int, FaceGeometry], adjacency: dict[int, set[int]], settings: PartitionSettings) -> list[set[int]]:
    if len(face_ids) < settings.min_patch_faces * 2:
        return [set(face_ids)]
    primary, secondary = boundary_uv_axes(face_ids, faces)
    best_sets = [set(face_ids)]
    best_count = 1
    for u_axis, v_axis in ((primary, secondary), (secondary, primary)):
        nodes = scanline_interval_nodes(face_ids, faces, u_axis, v_axis, settings)
        barriers = neck_barrier_lines(nodes, settings)
        if not barriers:
            continue
        components = interval_components(nodes, barriers, settings)
        face_sets = _component_face_sets(nodes, components, face_ids)
        merged = _merge_small_patches(face_sets, faces, adjacency, face_ids, settings)
        if len(merged) > best_count:
            best_sets = merged
            best_count = len(merged)
    return sorted(best_sets, key=lambda patch: min(patch))


def partition_region(original_region: int, face_ids: set[int], faces: dict[int, FaceGeometry], adjacency: dict[int, set[int]], settings: PartitionSettings) -> RegionPartitionRecord:
    available = set(face_id for face_id in face_ids if face_id in faces)
    if not available:
        return RegionPartitionRecord(original_region, len(face_ids), 0, True, "no available mesh faces", [])

    candidates: list[_PatchCandidate] = []
    for turn_zone, zone_faces in split_by_turn_zone(available, faces, adjacency, settings):
        allowed_angles = allowed_angles_for_zone(turn_zone, settings)
        nominal_angle = nominal_angle_for_zone(turn_zone, settings)
        for kind, surface_class, surface_faces in classify_main_plane_and_slope(zone_faces, faces, adjacency, settings):
            neck_sets = split_planar_patch_by_neck(surface_faces, faces, adjacency, settings) if kind == "planar" else [surface_faces]
            for neck_faces in neck_sets:
                for sized_faces, size_reason in split_patch_by_base_window(neck_faces, faces, adjacency, nominal_angle, settings):
                    source_parts = ["turn_zone", "surface"]
                    if kind == "planar" and len(neck_sets) > 1:
                        source_parts.append("neck")
                    if size_reason != "size_ok":
                        source_parts.append("size")
                    candidates.append(
                        _PatchCandidate(
                            kind=kind,
                            face_ids=sized_faces,
                            source="+".join(source_parts),
                            turn_zone=turn_zone,
                            allowed_angles=allowed_angles,
                            nominal_angle=nominal_angle,
                            surface_class=surface_class,
                            split_reason=size_reason,
                        )
                    )

    candidates = [candidate for candidate in candidates if candidate.face_ids]
    candidates = _cleanup_small_candidates(candidates, faces, adjacency, available, settings)
    if not candidates:
        return RegionPartitionRecord(original_region, len(face_ids), 0, True, "no output patches", [])

    if len(candidates) == 1 and candidates[0].face_ids == available:
        candidate = candidates[0]
        area = sum(faces[face_id].area for face_id in candidate.face_ids)
        patch_record = RegionPatch(
            original_region,
            str(original_region),
            candidate.kind,
            sorted(candidate.face_ids),
            area,
            candidate.source,
            candidate.turn_zone,
            candidate.allowed_angles,
            candidate.nominal_angle,
            candidate.surface_class,
            candidate.split_reason,
        )
        return RegionPartitionRecord(original_region, len(face_ids), 1, True, "unchanged", [patch_record])

    def patch_sort_key(candidate: _PatchCandidate) -> tuple[int, float, float, int]:
        zone_order = {"left": 0, "center": 1, "right": 2}.get(candidate.turn_zone, 3)
        centroid = weighted_centroid([faces[face_id] for face_id in candidate.face_ids])
        return zone_order, centroid[1], centroid[0], min(candidate.face_ids)

    patches: list[RegionPatch] = []
    for sub_index, candidate in enumerate(sorted(candidates, key=patch_sort_key), 1):
        area = sum(faces[face_id].area for face_id in candidate.face_ids)
        patches.append(
            RegionPatch(
                original_region,
                f"{original_region}.{sub_index}",
                candidate.kind,
                sorted(candidate.face_ids),
                area,
                candidate.source,
                candidate.turn_zone,
                candidate.allowed_angles,
                candidate.nominal_angle,
                candidate.surface_class,
                candidate.split_reason,
            )
        )
    return RegionPartitionRecord(original_region, len(face_ids), len(patches), False, "partitioned", patches)


def partition_selected_regions(
    regions: list[list[int]],
    selected_regions: set[int],
    faces: dict[int, FaceGeometry],
    settings: PartitionSettings | None = None,
) -> PartitionedRegions:
    settings = settings or PartitionSettings()
    adjacency = build_face_adjacency(faces, settings)
    output_regions: list[list[int]] = []
    records: list[RegionPartitionRecord] = []

    for region_index, raw_region in enumerate(regions, 1):
        face_ids = set(int(face_id) for face_id in raw_region)
        if region_index not in selected_regions:
            output_regions.append(sorted(face_ids))
            records.append(
                RegionPartitionRecord(
                    region_index,
                    len(face_ids),
                    1 if face_ids else 0,
                    True,
                    "not selected",
                    [RegionPatch(region_index, str(region_index), "original", sorted(face_ids), sum(faces[face_id].area for face_id in face_ids if face_id in faces), "passthrough")]
                    if face_ids
                    else [],
                )
            )
            continue

        record = partition_region(region_index, face_ids, faces, adjacency, settings)
        records.append(record)
        if record.patches:
            output_regions.extend([patch.face_ids for patch in record.patches])
        else:
            output_regions.append(sorted(face_ids))

    return PartitionedRegions(output_regions, records)


def record_to_manifest(record: RegionPartitionRecord) -> dict:
    return {
        "original_region": record.original_region,
        "input_face_count": record.input_face_count,
        "output_patch_count": record.output_patch_count,
        "unchanged": record.unchanged,
        "reason": record.reason,
        "patches": [
            {
                "label": patch.label,
                "kind": patch.kind,
                "source": patch.source,
                "turn_zone": patch.turn_zone,
                "allowed_angles": list(patch.allowed_angles),
                "nominal_angle": patch.nominal_angle,
                "surface_class": patch.surface_class,
                "split_reason": patch.split_reason,
                "collision_status": patch.collision_status,
                "face_count": len(patch.face_ids),
                "area": patch.area,
                "face_ids": patch.face_ids,
            }
            for patch in record.patches
        ],
    }
