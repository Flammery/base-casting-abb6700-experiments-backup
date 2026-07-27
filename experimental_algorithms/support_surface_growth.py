"""Seeded geometric support-surface recovery for wall-avoidance trials.

The active path supplies mesh-cell seeds.  Region growing follows adjacent,
near-coplanar triangles and stops at wall/fillet transitions.  Recovered support
cells are excluded only from the experimental wall-distance mesh.
"""

from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path

import numpy as np
from vtkmodules.vtkCommonCore import vtkIdList
from vtkmodules.util.numpy_support import vtk_to_numpy

from robot_studio_qt.core.geometry import cross, dot, length, normalize, subtract
from robot_studio_qt.kinematics.model import WorkpiecePlacement
from robot_studio_qt.path_planning.transforms import WorkpieceTransform
from robot_studio_qt.tools.reachability.collision import CollisionMesh, Triangle


Vector3 = tuple[float, float, float]
AVOIDANCE_VOLUME_SCHEMA = "base_casting_abb6700.avoidance_volume_settings"
AVOIDANCE_VOLUME_VERSION = 1
DEFAULT_U_EXPAND_PERCENT = 30.0
DEFAULT_V_EXPAND_PERCENT = 30.0


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
class AvoidanceVolumeSettings:
    u_expand_percent: float = DEFAULT_U_EXPAND_PERCENT
    v_expand_percent: float = DEFAULT_V_EXPAND_PERCENT
    n_plus_mm: float = 0.0
    n_minus_mm: float = 0.0

    def __post_init__(self) -> None:
        values = (
            self.u_expand_percent,
            self.v_expand_percent,
            self.n_plus_mm,
            self.n_minus_mm,
        )
        if any(not math.isfinite(float(value)) or float(value) < 0.0 for value in values):
            raise ValueError("避障范围的 U/V 扩大比例和 N+/N- 高度必须是非负有限数值")


@dataclass(frozen=True)
class AvoidanceVolumeFrame:
    origin: Vector3
    u_axis: Vector3
    v_axis: Vector3
    n_axis: Vector3

    def as_dict(self) -> dict:
        return {
            "origin_model": list(self.origin),
            "u_axis_model": list(self.u_axis),
            "v_axis_model": list(self.v_axis),
            "n_axis_model": list(self.n_axis),
        }

    @classmethod
    def from_dict(cls, payload: dict) -> "AvoidanceVolumeFrame":
        return cls(
            _vector3(payload.get("origin_model"), "origin_model"),
            _vector3(payload.get("u_axis_model"), "u_axis_model"),
            _vector3(payload.get("v_axis_model"), "v_axis_model"),
            _vector3(payload.get("n_axis_model"), "n_axis_model"),
        )


@dataclass(frozen=True)
class AvoidanceVolumeResult:
    frame: AvoidanceVolumeFrame
    settings: AvoidanceVolumeSettings
    support_bounds_uvn: tuple[Vector3, Vector3]
    volume_bounds_uvn: tuple[Vector3, Vector3]
    obstacle_cell_ids: frozenset[int]
    outside_cell_count: int

    @property
    def vertices_model(self) -> tuple[Vector3, ...]:
        lower, upper = self.volume_bounds_uvn
        return tuple(
            _point_from_uvn(self.frame, u_value, v_value, n_value)
            for n_value in (lower[2], upper[2])
            for v_value in (lower[1], upper[1])
            for u_value in (lower[0], upper[0])
        )

    def as_dict(self) -> dict:
        return {
            "frame": self.frame.as_dict(),
            "settings": asdict(self.settings),
            "support_bounds_uvn": [list(self.support_bounds_uvn[0]), list(self.support_bounds_uvn[1])],
            "volume_bounds_uvn": [list(self.volume_bounds_uvn[0]), list(self.volume_bounds_uvn[1])],
            "obstacle_cell_count": len(self.obstacle_cell_ids),
            "outside_cell_count": self.outside_cell_count,
        }


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


def build_obstacle_mesh_template(
    polydata,
    support: SupportSurfaceResult,
    obstacle_cell_ids: set[int] | frozenset[int] | None = None,
) -> ObstacleMeshTemplate:
    """Sample walls after removing support, prioritizing nearby obstacle cells."""

    cell_count = int(polydata.GetNumberOfCells())
    explicit_obstacles = (
        sorted(
            int(cell_id)
            for cell_id in obstacle_cell_ids
            if 0 <= int(cell_id) < cell_count and int(cell_id) not in support.support_cell_ids
        )
        if obstacle_cell_ids is not None
        else None
    )
    obstacle_count = (
        len(explicit_obstacles)
        if explicit_obstacles is not None
        else max(0, cell_count - len(support.support_cell_ids))
    )
    if obstacle_count <= 0:
        if explicit_obstacles is not None:
            raise ValueError("当前 UVN 避障范围内没有非支撑墙体，无法建立障碍网格")
        raise ValueError("支撑面覆盖整个工件，无法建立墙体障碍网格")
    limit = support.settings.max_obstacle_triangles
    if explicit_obstacles is not None:
        priority_ids = explicit_obstacles
        other_ids: list[int] = []
        selected_ids = _evenly_sample_ids(priority_ids, limit) if limit > 0 else list(priority_ids)
    else:
        support_bounds = _cells_bounds(polydata, support.support_cell_ids)
        priority_bounds = (
            tuple(value - support.settings.obstacle_priority_margin_mm for value in support_bounds[0]),
            tuple(value + support.settings.obstacle_priority_margin_mm for value in support_bounds[1]),
        )
        priority_ids = []
        other_ids = []
        for cell_id in range(cell_count):
            if cell_id in support.support_cell_ids:
                continue
            target = priority_ids if _bounds_overlap(_cell_bounds(polydata, cell_id), priority_bounds) else other_ids
            target.append(cell_id)

    if explicit_obstacles is None and limit > 0:
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
    elif explicit_obstacles is None:
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


def build_avoidance_volume(
    polydata,
    support: SupportSurfaceResult,
    settings: AvoidanceVolumeSettings,
    *,
    raster_chart: dict | None = None,
    frame: AvoidanceVolumeFrame | None = None,
    cell_bounds_uvn: np.ndarray | None = None,
) -> AvoidanceVolumeResult:
    """Build a UVN rectangular obstacle volume around a recovered support."""

    frame = frame or avoidance_volume_frame(polydata, support, raster_chart=raster_chart)
    support_bounds = _cells_bounds_in_frame(polydata, support.support_cell_ids, frame)
    u_span = max(0.0, support_bounds[1][0] - support_bounds[0][0])
    v_span = max(0.0, support_bounds[1][1] - support_bounds[0][1])
    u_padding = u_span * settings.u_expand_percent / 200.0
    v_padding = v_span * settings.v_expand_percent / 200.0
    volume_bounds = (
        (
            support_bounds[0][0] - u_padding,
            support_bounds[0][1] - v_padding,
            support_bounds[0][2] - settings.n_minus_mm,
        ),
        (
            support_bounds[1][0] + u_padding,
            support_bounds[1][1] + v_padding,
            support_bounds[1][2] + settings.n_plus_mm,
        ),
    )
    bounds_array = (
        avoidance_cell_bounds_uvn(polydata, frame)
        if cell_bounds_uvn is None
        else np.asarray(cell_bounds_uvn)
    )
    cell_count = int(polydata.GetNumberOfCells())
    if bounds_array.shape != (cell_count, 6):
        raise ValueError(
            f"避障 UVN cell bounds 形状错误: {bounds_array.shape}; 期望 {(cell_count, 6)}"
        )
    lower = np.asarray(volume_bounds[0], dtype=float)
    upper = np.asarray(volume_bounds[1], dtype=float)
    overlap = np.all(bounds_array[:, :3] <= upper, axis=1) & np.all(
        bounds_array[:, 3:] >= lower,
        axis=1,
    )
    if support.support_cell_ids:
        overlap[np.fromiter(support.support_cell_ids, dtype=np.int64)] = False
    obstacle_indices = np.flatnonzero(overlap)
    obstacles = frozenset(int(value) for value in obstacle_indices)
    outside = cell_count - len(support.support_cell_ids) - len(obstacles)
    return AvoidanceVolumeResult(
        frame=frame,
        settings=settings,
        support_bounds_uvn=support_bounds,
        volume_bounds_uvn=volume_bounds,
        obstacle_cell_ids=obstacles,
        outside_cell_count=outside,
    )


def avoidance_cell_bounds_uvn(polydata, frame: AvoidanceVolumeFrame) -> np.ndarray:
    """Cacheable per-cell UVN bounds used for responsive range previews."""

    cell_count = int(polydata.GetNumberOfCells())
    polygon_count = int(polydata.GetNumberOfPolys())
    if (
        cell_count == polygon_count
        and int(polydata.GetNumberOfVerts()) == 0
        and int(polydata.GetNumberOfLines()) == 0
        and int(polydata.GetNumberOfStrips()) == 0
    ):
        points = vtk_to_numpy(polydata.GetPoints().GetData()).astype(np.float64, copy=False)
        axes = np.asarray((frame.u_axis, frame.v_axis, frame.n_axis), dtype=np.float64)
        projected = (points - np.asarray(frame.origin, dtype=np.float64)) @ axes.T
        polygons = polydata.GetPolys()
        offsets = vtk_to_numpy(polygons.GetOffsetsArray()).astype(np.int64, copy=False)
        connectivity = vtk_to_numpy(polygons.GetConnectivityArray()).astype(np.int64, copy=False)
        starts = offsets[:-1]
        connected = projected[connectivity]
        minimum = np.column_stack(
            [np.minimum.reduceat(connected[:, axis], starts) for axis in range(3)]
        )
        maximum = np.column_stack(
            [np.maximum.reduceat(connected[:, axis], starts) for axis in range(3)]
        )
        return np.column_stack((minimum, maximum))

    rows = np.empty((cell_count, 6), dtype=np.float64)
    for cell_id in range(cell_count):
        lower, upper = _cell_bounds_in_frame(polydata, cell_id, frame)
        rows[cell_id, :3] = lower
        rows[cell_id, 3:] = upper
    return rows


def avoidance_volume_frame(
    polydata,
    support: SupportSurfaceResult,
    *,
    raster_chart: dict | None = None,
) -> AvoidanceVolumeFrame:
    """Return a stable right-handed UVN frame in model coordinates."""

    if raster_chart:
        origin = _vector3(raster_chart.get("origin"), "raster_chart.origin")
        normal = normalize(_vector3(raster_chart.get("normal"), "raster_chart.normal"))
        requested_u = _vector3(raster_chart.get("u_axis"), "raster_chart.u_axis")
        u_axis = _project_axis_to_plane(requested_u, normal)
        v_axis = normalize(cross(normal, u_axis))
        requested_v = _vector3(raster_chart.get("v_axis"), "raster_chart.v_axis")
        if dot(v_axis, requested_v) < 0.0:
            u_axis = tuple(-value for value in u_axis)
            v_axis = tuple(-value for value in v_axis)
        return AvoidanceVolumeFrame(origin, u_axis, v_axis, normal)  # type: ignore[arg-type]

    origin = support.reference_origin
    normal = normalize(support.reference_normal)
    points = _cell_points(polydata, support.support_cell_ids)
    candidates: list[Vector3] = []
    for base_axis in ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)):
        projected = _project_axis_to_plane(base_axis, normal, allow_zero=True)
        if length(projected) > 1e-12:
            candidates.append(projected)
    if not candidates:
        raise ValueError("无法为支撑面建立稳定的 UV 轴")

    def projected_span(axis: Vector3) -> float:
        values = [dot(subtract(point, origin), axis) for point in points]
        return max(values) - min(values) if values else 0.0

    u_axis = max(candidates, key=projected_span)
    v_axis = normalize(cross(normal, u_axis))
    return AvoidanceVolumeFrame(origin, u_axis, v_axis, normal)  # type: ignore[arg-type]


def default_normal_heights_mm(
    polydata,
    support: SupportSurfaceResult,
    frame: AvoidanceVolumeFrame,
) -> tuple[float, float]:
    """Return N+/N- values that initially span the complete model."""

    support_bounds = _cells_bounds_in_frame(polydata, support.support_cell_ids, frame)
    values = [
        dot(subtract(tuple(float(value) for value in polydata.GetPoint(index)), frame.origin), frame.n_axis)
        for index in range(int(polydata.GetNumberOfPoints()))
    ]
    if not values:
        return 0.0, 0.0
    return (
        max(0.0, max(values) - support_bounds[1][2]),
        max(0.0, support_bounds[0][2] - min(values)),
    )


def avoidance_settings_path_for(project_path: Path) -> Path:
    name = project_path.name
    stem = name[: -len(".rsp.json")] if name.endswith(".rsp.json") else project_path.stem
    return project_path.with_name(f"{stem}_avoidance.json")


def load_avoidance_settings(path: Path) -> dict:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if (
        payload.get("schema") != AVOIDANCE_VOLUME_SCHEMA
        or int(payload.get("version", 0)) != AVOIDANCE_VOLUME_VERSION
    ):
        raise ValueError(f"不是支持的避障范围设置文件: {path}")
    records = payload.get("regions")
    if not isinstance(records, list):
        raise ValueError("避障范围设置缺少 regions")
    return payload


def write_avoidance_settings(
    path: Path,
    *,
    input_project: Path,
    selectors: list[str],
    records: list[dict],
) -> None:
    payload = {
        "schema": AVOIDANCE_VOLUME_SCHEMA,
        "version": AVOIDANCE_VOLUME_VERSION,
        "input_project": str(input_project),
        "selectors": list(selectors),
        "regions": records,
    }
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def avoidance_setting_for_label(payload: dict, label: str) -> dict | None:
    return next(
        (record for record in payload.get("regions", []) if str(record.get("region_label")) == str(label)),
        None,
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


def _cell_points(polydata, cell_ids) -> list[Vector3]:
    point_ids: set[int] = set()
    for cell_id in cell_ids:
        cell = polydata.GetCell(int(cell_id))
        for index in range(int(cell.GetNumberOfPoints())):
            point_ids.add(int(cell.GetPointId(index)))
    return [
        tuple(float(value) for value in polydata.GetPoint(point_id))
        for point_id in sorted(point_ids)
    ]  # type: ignore[return-value]


def _cell_bounds_in_frame(
    polydata,
    cell_id: int,
    frame: AvoidanceVolumeFrame,
) -> tuple[Vector3, Vector3]:
    cell = polydata.GetCell(int(cell_id))
    projected = [
        _point_to_uvn(
            tuple(float(value) for value in polydata.GetPoint(cell.GetPointId(index))),
            frame,
        )
        for index in range(int(cell.GetNumberOfPoints()))
    ]
    if not projected:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    return (
        tuple(min(point[axis] for point in projected) for axis in range(3)),
        tuple(max(point[axis] for point in projected) for axis in range(3)),
    )  # type: ignore[return-value]


def _cells_bounds_in_frame(
    polydata,
    cell_ids,
    frame: AvoidanceVolumeFrame,
) -> tuple[Vector3, Vector3]:
    minimum = [math.inf, math.inf, math.inf]
    maximum = [-math.inf, -math.inf, -math.inf]
    for cell_id in cell_ids:
        lower, upper = _cell_bounds_in_frame(polydata, int(cell_id), frame)
        for axis in range(3):
            minimum[axis] = min(minimum[axis], lower[axis])
            maximum[axis] = max(maximum[axis], upper[axis])
    if any(not math.isfinite(value) for value in (*minimum, *maximum)):
        raise ValueError("支撑面没有可用于构建避障范围的几何点")
    return tuple(minimum), tuple(maximum)  # type: ignore[return-value]


def _point_to_uvn(point: Vector3, frame: AvoidanceVolumeFrame) -> Vector3:
    relative = subtract(point, frame.origin)
    return (
        dot(relative, frame.u_axis),
        dot(relative, frame.v_axis),
        dot(relative, frame.n_axis),
    )


def _point_from_uvn(
    frame: AvoidanceVolumeFrame,
    u_value: float,
    v_value: float,
    n_value: float,
) -> Vector3:
    return tuple(
        frame.origin[axis]
        + u_value * frame.u_axis[axis]
        + v_value * frame.v_axis[axis]
        + n_value * frame.n_axis[axis]
        for axis in range(3)
    )  # type: ignore[return-value]


def _project_axis_to_plane(axis: Vector3, normal: Vector3, *, allow_zero: bool = False) -> Vector3:
    projected = tuple(axis[index] - dot(axis, normal) * normal[index] for index in range(3))
    if length(projected) <= 1e-12:
        if allow_zero:
            return (0.0, 0.0, 0.0)
        raise ValueError("避障范围 U 轴与支撑面法向平行")
    return normalize(projected)


def _vector3(value, name: str) -> Vector3:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{name} 必须包含三个数值")
    result = tuple(float(item) for item in value)
    if any(not math.isfinite(item) for item in result):
        raise ValueError(f"{name} 包含非有限数值")
    return result  # type: ignore[return-value]


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
