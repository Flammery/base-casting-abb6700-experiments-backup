"""Seeded geometric support-surface recovery for wall-avoidance trials.

The active path supplies mesh-cell seeds.  Region growing follows adjacent,
near-coplanar triangles and stops at wall/fillet transitions.  Recovered support
cells are excluded only from the experimental wall-distance mesh.
"""

from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass
import math

from vtkmodules.vtkCommonCore import vtkIdList

from robot_studio_qt.core.geometry import cross, dot, length, normalize, subtract
from robot_studio_qt.kinematics.model import WorkpiecePlacement
from robot_studio_qt.path_planning.transforms import WorkpieceTransform
from robot_studio_qt.tools.reachability.collision import CollisionMesh, Triangle


Vector3 = tuple[float, float, float]


@dataclass(frozen=True)
class SupportGrowthSettings:
    local_normal_angle_deg: float = 12.0
    reference_normal_angle_deg: float = 12.0
    reference_plane_distance_mm: float = 3.0
    max_support_cells: int = 250_000
    max_obstacle_triangles: int = 6_000
    obstacle_priority_margin_mm: float = 800.0


@dataclass(frozen=True)
class CellGeometry:
    centroid: Vector3
    normal: Vector3
    area: float


@dataclass(frozen=True)
class SupportSurfaceResult:
    seed_cell_ids: frozenset[int]
    support_cell_ids: frozenset[int]
    reference_origin: Vector3
    reference_normal: Vector3
    max_normal_angle_deg: float
    max_plane_distance_mm: float
    settings: SupportGrowthSettings

    def as_dict(self, total_cell_count: int | None = None) -> dict:
        payload = {
            "seed_cell_count": len(self.seed_cell_ids),
            "support_cell_count": len(self.support_cell_ids),
            "reference_origin_model": list(self.reference_origin),
            "reference_normal_model": list(self.reference_normal),
            "max_normal_angle_deg": self.max_normal_angle_deg,
            "max_plane_distance_mm": self.max_plane_distance_mm,
            "settings": asdict(self.settings),
        }
        if total_cell_count is not None:
            payload["obstacle_cell_count"] = max(0, int(total_cell_count) - len(self.support_cell_ids))
        return payload


@dataclass(frozen=True)
class ObstacleMeshTemplate:
    triangles_model: tuple[Triangle, ...]
    source_cell_count: int
    support_cell_count: int
    priority_obstacle_cell_count: int

    def to_collision_mesh(self, placement: WorkpiecePlacement) -> CollisionMesh:
        transform = WorkpieceTransform(placement)
        triangles_world = [
            tuple(transform.model_point_to_world(point) for point in triangle)
            for triangle in self.triangles_model
        ]
        return CollisionMesh(triangles_world)  # type: ignore[arg-type]


def grow_support_surface(
    polydata,
    seed_cell_ids: set[int] | frozenset[int],
    settings: SupportGrowthSettings | None = None,
) -> SupportSurfaceResult:
    settings = settings or SupportGrowthSettings()
    cell_count = int(polydata.GetNumberOfCells())
    seeds = frozenset(int(cell_id) for cell_id in seed_cell_ids if 0 <= int(cell_id) < cell_count)
    if not seeds:
        raise ValueError("支撑面区域生长没有有效的路径 face_id 种子")

    seed_geometries = [_cell_geometry(polydata, cell_id) for cell_id in sorted(seeds)]
    reference_origin = _weighted_centroid(seed_geometries)
    reference_normal = _weighted_normal(seed_geometries)
    if length(reference_normal) <= 1e-12:
        raise ValueError("支撑面种子无法计算稳定法向")

    polydata.BuildLinks()
    accepted = set(seeds)
    queue = deque(sorted(seeds))
    geometry_cache = {cell_id: geometry for cell_id, geometry in zip(sorted(seeds), seed_geometries)}

    def geometry_for(cell_id: int) -> CellGeometry:
        if cell_id not in geometry_cache:
            geometry_cache[cell_id] = _cell_geometry(polydata, cell_id)
        return geometry_cache[cell_id]

    while queue:
        current_id = queue.popleft()
        current = geometry_for(current_id)
        for neighbor_id in _cell_neighbors(polydata, current_id):
            if neighbor_id in accepted:
                continue
            neighbor = geometry_for(neighbor_id)
            if not _belongs_to_support(neighbor, current, reference_origin, reference_normal, settings):
                continue
            accepted.add(neighbor_id)
            queue.append(neighbor_id)
            if len(accepted) > settings.max_support_cells:
                raise RuntimeError(
                    f"支撑面区域生长超过 {settings.max_support_cells} cells；请检查法向/共面阈值"
                )

    support_geometries = [geometry_for(cell_id) for cell_id in accepted]
    max_normal = max(_normal_angle_degrees(item.normal, reference_normal) for item in support_geometries)
    max_plane = max(_plane_distance(item.centroid, reference_origin, reference_normal) for item in support_geometries)
    return SupportSurfaceResult(
        seed_cell_ids=seeds,
        support_cell_ids=frozenset(accepted),
        reference_origin=reference_origin,
        reference_normal=reference_normal,
        max_normal_angle_deg=max_normal,
        max_plane_distance_mm=max_plane,
        settings=settings,
    )


def build_obstacle_mesh_template(polydata, support: SupportSurfaceResult) -> ObstacleMeshTemplate:
    """Sample walls after removing support, prioritizing nearby obstacle cells."""

    cell_count = int(polydata.GetNumberOfCells())
    obstacle_count = max(0, cell_count - len(support.support_cell_ids))
    if obstacle_count <= 0:
        raise ValueError("支撑面覆盖整个工件，无法建立墙体障碍网格")
    limit = support.settings.max_obstacle_triangles
    support_bounds = _cells_bounds(polydata, support.support_cell_ids)
    priority_bounds = (
        tuple(value - support.settings.obstacle_priority_margin_mm for value in support_bounds[0]),
        tuple(value + support.settings.obstacle_priority_margin_mm for value in support_bounds[1]),
    )
    priority_ids: list[int] = []
    other_ids: list[int] = []
    for cell_id in range(cell_count):
        if cell_id in support.support_cell_ids:
            continue
        target = priority_ids if _bounds_overlap(_cell_bounds(polydata, cell_id), priority_bounds) else other_ids
        target.append(cell_id)

    if limit > 0:
        priority_budget = min(len(priority_ids), max(1, round(limit * 0.85)))
        other_budget = min(len(other_ids), max(0, limit - priority_budget))
        remaining = limit - priority_budget - other_budget
        if remaining > 0:
            extra_priority = min(len(priority_ids) - priority_budget, remaining)
            priority_budget += extra_priority
            remaining -= extra_priority
        if remaining > 0:
            other_budget += min(len(other_ids) - other_budget, remaining)
        selected_ids = [
            *_evenly_sample_ids(priority_ids, priority_budget),
            *_evenly_sample_ids(other_ids, other_budget),
        ]
    else:
        selected_ids = [*priority_ids, *other_ids]

    triangles: list[Triangle] = []
    for cell_id in selected_ids:
        cell = polydata.GetCell(cell_id)
        point_count = int(cell.GetNumberOfPoints())
        if point_count < 3:
            continue
        points = [
            tuple(float(value) for value in polydata.GetPoint(cell.GetPointId(index)))
            for index in range(point_count)
        ]
        for triangle_index in range(1, point_count - 1):
            triangles.append((points[0], points[triangle_index], points[triangle_index + 1]))
            if limit > 0 and len(triangles) >= limit:
                break
        if limit > 0 and len(triangles) >= limit:
            break
    if not triangles:
        raise ValueError("墙体障碍网格没有可用三角形")
    return ObstacleMeshTemplate(
        tuple(triangles),
        cell_count,
        len(support.support_cell_ids),
        len(priority_ids),
    )


def path_seed_cell_ids(path) -> set[int]:
    return {int(waypoint.face_id) for waypoint in path.waypoints if int(waypoint.face_id) >= 0}


def _belongs_to_support(
    candidate: CellGeometry,
    current: CellGeometry,
    reference_origin: Vector3,
    reference_normal: Vector3,
    settings: SupportGrowthSettings,
) -> bool:
    if _normal_angle_degrees(candidate.normal, current.normal) > settings.local_normal_angle_deg:
        return False
    if _normal_angle_degrees(candidate.normal, reference_normal) > settings.reference_normal_angle_deg:
        return False
    return _plane_distance(candidate.centroid, reference_origin, reference_normal) <= settings.reference_plane_distance_mm


def _cell_neighbors(polydata, cell_id: int) -> set[int]:
    cell = polydata.GetCell(cell_id)
    point_count = int(cell.GetNumberOfPoints())
    neighbors: set[int] = set()
    for index in range(point_count):
        edge = vtkIdList()
        edge.InsertNextId(cell.GetPointId(index))
        edge.InsertNextId(cell.GetPointId((index + 1) % point_count))
        found = vtkIdList()
        polydata.GetCellNeighbors(cell_id, edge, found)
        for neighbor_index in range(found.GetNumberOfIds()):
            neighbors.add(int(found.GetId(neighbor_index)))
    return neighbors


def _cell_geometry(polydata, cell_id: int) -> CellGeometry:
    cell = polydata.GetCell(cell_id)
    point_count = int(cell.GetNumberOfPoints())
    if point_count < 3:
        return CellGeometry((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0)
    points = [
        tuple(float(value) for value in polydata.GetPoint(cell.GetPointId(index)))
        for index in range(point_count)
    ]
    origin = points[0]
    weighted_normal = (0.0, 0.0, 0.0)
    area = 0.0
    for triangle_index in range(1, point_count - 1):
        first = subtract(points[triangle_index], origin)
        second = subtract(points[triangle_index + 1], origin)
        normal_cross = cross(first, second)
        twice_area = length(normal_cross)
        if twice_area <= 1e-12:
            continue
        weighted_normal = (
            weighted_normal[0] + normal_cross[0],
            weighted_normal[1] + normal_cross[1],
            weighted_normal[2] + normal_cross[2],
        )
        area += twice_area * 0.5
    centroid = tuple(sum(point[axis] for point in points) / point_count for axis in range(3))
    return CellGeometry(centroid, normalize(weighted_normal), area)  # type: ignore[arg-type]


def _weighted_centroid(items: list[CellGeometry]) -> Vector3:
    total = sum(max(item.area, 1e-12) for item in items)
    return tuple(
        sum(item.centroid[axis] * max(item.area, 1e-12) for item in items) / total
        for axis in range(3)
    )  # type: ignore[return-value]


def _weighted_normal(items: list[CellGeometry]) -> Vector3:
    reference = next((item.normal for item in items if length(item.normal) > 1e-12), (0.0, 0.0, 0.0))
    total = [0.0, 0.0, 0.0]
    for item in items:
        sign = -1.0 if dot(item.normal, reference) < 0.0 else 1.0
        weight = max(item.area, 1e-12)
        for axis in range(3):
            total[axis] += item.normal[axis] * sign * weight
    return normalize(tuple(total))


def _normal_angle_degrees(first: Vector3, second: Vector3) -> float:
    if length(first) <= 1e-12 or length(second) <= 1e-12:
        return 180.0
    cosine = max(-1.0, min(1.0, dot(normalize(first), normalize(second))))
    return math.degrees(math.acos(cosine))


def _plane_distance(point: Vector3, origin: Vector3, normal: Vector3) -> float:
    return abs(dot(subtract(point, origin), normalize(normal)))


def _cell_bounds(polydata, cell_id: int) -> tuple[Vector3, Vector3]:
    bounds = [0.0] * 6
    polydata.GetCellBounds(cell_id, bounds)
    return (
        (float(bounds[0]), float(bounds[2]), float(bounds[4])),
        (float(bounds[1]), float(bounds[3]), float(bounds[5])),
    )


def _cells_bounds(polydata, cell_ids: frozenset[int]) -> tuple[Vector3, Vector3]:
    minimum = [math.inf, math.inf, math.inf]
    maximum = [-math.inf, -math.inf, -math.inf]
    for cell_id in cell_ids:
        lower, upper = _cell_bounds(polydata, cell_id)
        for axis in range(3):
            minimum[axis] = min(minimum[axis], lower[axis])
            maximum[axis] = max(maximum[axis], upper[axis])
    return tuple(minimum), tuple(maximum)  # type: ignore[return-value]


def _bounds_overlap(
    first: tuple[Vector3, Vector3],
    second: tuple[Vector3, Vector3],
) -> bool:
    return all(first[0][axis] <= second[1][axis] and first[1][axis] >= second[0][axis] for axis in range(3))


def _evenly_sample_ids(values: list[int], limit: int) -> list[int]:
    if limit <= 0:
        return []
    if len(values) <= limit:
        return list(values)
    if limit == 1:
        return [values[len(values) // 2]]
    return [values[round(index * (len(values) - 1) / (limit - 1))] for index in range(limit)]
