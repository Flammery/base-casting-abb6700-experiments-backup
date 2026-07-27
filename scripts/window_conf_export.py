from __future__ import annotations

import csv
import json
import math
import sys
from dataclasses import replace
from pathlib import Path

# 实验目录独立于 src：本脚本负责项目专属批量测试，src 只作为稳定软件库调用。
EXPERIMENT_DIR = Path(__file__).resolve().parents[1]
ROOT = EXPERIMENT_DIR.parents[1]
SRC = ROOT / "src"
SCRIPT_DIR = Path(__file__).resolve().parent
EXPERIMENTAL_DIR = EXPERIMENT_DIR / "experimental_algorithms"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
if str(EXPERIMENTAL_DIR) not in sys.path:
    sys.path.insert(0, str(EXPERIMENTAL_DIR))

from robot_studio_qt.cad.import_service import CadImportService
from robot_studio_qt.cad.mesh_io import create_mesh_reader
from robot_studio_qt.core.formatting import format_number
from robot_studio_qt.core.geometry import cross, dot, normalize
from robot_studio_qt.kinematics.orientation import Quaternion, euler_xyz_degrees_to_quaternion, matrix_to_quaternion
from robot_studio_qt.path_planning.mesh_raster import (
    average_normal,
    encoded_raster_line_id,
    mesh_centroid,
    project_triangle,
    raster_base_line_id,
    raster_segment_id,
    read_triangles,
    sample_projected_mesh,
)
from robot_studio_qt.path_planning.models import (
    PathResult,
    PathSource,
    RasterFeedDirection,
    RasterPlannerSettings,
    StartCorner,
    Waypoint,
)
from robot_studio_qt.path_planning.transforms import WorkpieceTransform
from robot_studio_qt.polishing_tool import tool_to_rapid
from robot_studio_qt.project import load_project_file, save_project_file
from robot_studio_qt.tools.reachability.collision import CollisionMesh, CollisionSettings
from raster_domain import point_to_uv, raster_samples
from hole_aware_raster import (  # noqa: F401 - runner API
    hole_aware_raster_samples,
    polygon_has_relevant_holes,
    projected_raster_cell_samples,
)
from region_selectors import parse_region_selectors, selector_matches, validate_selectors  # noqa: F401 - shared UI/runner API
from robot_pose_avoidance import (  # noqa: F401 - experimental runner API
    DEFAULT_SAMPLE_LIMIT,
    DEFAULT_MIN_CLEARANCE_MM,
    EXPERIMENT_CLEARANCE_MM,
    EXPERIMENT_LINK_RADIUS_MM,
    EXPERIMENT_USE_SEGMENT_RADIUS,
    POSE_ROLL_DEGREES,
    select_robot_pose,
)
from robot_config_override import load_robot_config_override  # noqa: F401 - shared UI/runner API
from support_surface_growth import (  # noqa: F401 - experimental runner/UI API
    SupportGrowthSettings,
    build_obstacle_mesh_template,
    grow_support_surface,
    path_seed_cell_ids,
)


PARTITIONED_PROJECT_PATH = EXPERIMENT_DIR / "inputs" / "latest_partitioned.rsp.json"
SCRIPT_TEST_PROJECT_PATH = EXPERIMENT_DIR / "inputs" / "latest_script_test.rsp.json"
FALLBACK_PROJECT_PATH = ROOT / "project" / "test-0704-selected.rsp.json"


def resolve_default_project_path(experiment_dir: Path = EXPERIMENT_DIR, root: Path = ROOT) -> Path:
    # 分区预处理会写 latest_partitioned；否则使用软件导出的 latest_script_test，再回退到旧实验项目。
    partitioned_path = experiment_dir / "inputs" / "latest_partitioned.rsp.json"
    if partitioned_path.exists():
        return partitioned_path
    script_test_path = experiment_dir / "inputs" / "latest_script_test.rsp.json"
    if script_test_path.exists():
        return script_test_path
    return root / "project" / "test-0704-selected.rsp.json"


# 默认输入项目。重新选面后，优先在软件中点击“导出到脚本测试”，脚本会自动读取 latest 快照。
PROJECT_PATH = resolve_default_project_path()
# 默认输出目录。批量实验时可在调用前覆盖 OUTDIR，把结果按安装位姿分开保存。
OUTDIR = EXPERIMENT_DIR / "results" / "window_conf_uv_long_short_x1500_2500_x3500_z500_step30"
# 工件安装位姿参数：X/Y/Z 是安装位置，RZ 由 ANGLES 批量扫描模拟转台角度。
MODEL_X = 3500.0
MODEL_Y = 0.0
MODEL_Z = 500.0
ANGLES = list(range(0, 360, 30))
# 当前第一阶段采用保守类矩形加工窗口，只导出整块 region 完全落入窗口的路径。
WINDOW_SHAPE = "box"
WINDOW_X = (1500.0, 2500.0)
WINDOW_Y = (-1050.0, 1050.0)
SPACING = 50.0
POINT_STEP = 50.0
BOUNDARY_MARGIN = 6.0
SAFE_DISTANCE = 150.0
CONF_Y_NEGATIVE = (-1, -1, 0, 1)
CONF_Y_NONNEGATIVE = (0, 0, -1, 1)
TOOL_LOAD_MASS_KG = 1.0
RAPID_LOAD_PLACEHOLDER = "[1,[0,0,100],[1,0,0,0],0.01,0.01,0.01]"
ORIENTATION_MODE = "base_y_aligned"
AXIS_MODE = "region_boundary_uv_long_short"
FEED_VARIANTS = [
    ("long_side", RasterFeedDirection.LONG_SIDE),
    ("short_side", RasterFeedDirection.SHORT_SIDE),
]


def partition_manifest_path_for(project_path: Path) -> Path:
    name = project_path.name
    stem = name[: -len(".rsp.json")] if name.endswith(".rsp.json") else project_path.stem
    return project_path.with_name(f"{stem}_manifest.json")


def point_in_polygon_xy(point: tuple[float, float], polygon: list[list[float]]) -> bool:
    if len(polygon) < 3:
        return True
    inside = False
    x_value, y_value = point
    previous = polygon[-1]
    for current in polygon:
        xi, yi = float(current[0]), float(current[1])
        xj, yj = float(previous[0]), float(previous[1])
        intersects = (yi > y_value) != (yj > y_value)
        if intersects:
            x_cross = (xj - xi) * (y_value - yi) / max(yj - yi, 1e-12) + xi
            if x_value < x_cross:
                inside = not inside
        previous = current
    return inside


def point_allowed_by_clip(
    point: tuple[float, float],
    clip_polygon: list[list[float]] | None,
    exclude_polygons: list[list[list[float]]] | None = None,
) -> bool:
    if clip_polygon and not point_in_polygon_xy(point, clip_polygon):
        return False
    return not any(point_in_polygon_xy(point, polygon) for polygon in (exclude_polygons or []))


def manual_clip_regions(project_path: Path, regions: list[set[int]]) -> list[dict]:
    manifest_path = partition_manifest_path_for(project_path)
    if not manifest_path.exists():
        return [
            {
                "source_region": index,
                "label": str(index),
                "face_ids": region,
                "clip_polygon": None,
                "exclude_polygons": [],
                "raster_chart": None,
            }
            for index, region in enumerate(regions, 1)
        ]
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except Exception:
        manifest = {}
    # Automatic preprocessing materializes patches as sequential face-id regions.
    # Restore its human labels/source region here so selectors such as ``1-1``
    # work for both automatic patches and manual-v2 raster patches.
    if (
        manifest.get("schema") == "base_casting_abb6700.region_partition_manifest"
        and int(manifest.get("version", 0)) == 2
    ):
        flattened: list[dict] = []
        for record in manifest.get("records", []):
            source_region = int(record.get("original_region", 0))
            patches = list(record.get("patches") or [])
            if not patches:
                patches = [{"label": str(source_region)}]
            for patch in patches:
                flattened.append({"source_region": source_region, "label": str(patch.get("label") or source_region)})
        if len(flattened) == len(regions):
            return [
                {
                    "source_region": flattened[index]["source_region"],
                    "label": flattened[index]["label"],
                    "face_ids": region,
                    "clip_polygon": None,
                    "exclude_polygons": [],
                    "raster_chart": None,
                }
                for index, region in enumerate(regions)
            ]
    # Version 1 materialized partitions directly into selected_path_face_regions.
    # It shares the historical schema name with the version-2 UV clip manifest,
    # so treating v1 records as clip polygons duplicates region 1 and corrupts
    # paths. Only v2 records contain authoritative clip/exclusion geometry.
    if (
        manifest.get("schema") != "base_casting_abb6700.manual_region_partition_manifest"
        or int(manifest.get("version", 0)) != 2
    ):
        return [
            {
                "source_region": index,
                "label": str(index),
                "face_ids": region,
                "clip_polygon": None,
                "exclude_polygons": [],
                "raster_chart": None,
            }
            for index, region in enumerate(regions, 1)
        ]

    clip_regions: list[dict] = []
    patched_sources: set[int] = set()
    for record in manifest.get("records", []):
        source_region = int(record.get("original_region", 0))
        if source_region <= 0 or source_region > len(regions):
            continue
        appended = False
        for patch in record.get("patches", []):
            clip_polygon = patch.get("clip_polygon")
            if not isinstance(clip_polygon, list) or len(clip_polygon) < 3:
                continue
            clip_regions.append(
                {
                    "source_region": source_region,
                    "label": str(patch.get("label", f"{source_region}_1")),
                    "face_ids": set(regions[source_region - 1]),
                    "clip_polygon": clip_polygon,
                    "exclude_polygons": patch.get("exclude_polygons") or [],
                    "raster_chart": patch.get("raster_chart") or record.get("raster_chart"),
                }
            )
            appended = True
        if appended:
            patched_sources.add(source_region)
    for index, region in enumerate(regions, 1):
        if index in patched_sources:
            continue
        clip_regions.append({"source_region": index, "label": str(index), "face_ids": region, "clip_polygon": None, "exclude_polygons": [], "raster_chart": None})
    return clip_regions


def split_discontinuous_raster_segments(samples, point_step: float):
    """Give scanline intervals separated by a hole independent motion segments.

    The mesh sampler intentionally returns multiple intervals for a scanline
    crossing a hole. Historically those intervals shared one line/segment id,
    causing preview and RAPID MoveL motion to bridge the empty area. A same-line
    jump larger than the normal point step starts a new processing segment;
    build_motion() will then add safe departure/approach waypoints.
    """
    if not samples:
        return samples
    output = []
    segment_id = 0
    previous = None
    previous_source_segment = None
    jump_limit = max(float(point_step) * 1.5, 1e-6)
    for line_id, point_id, face_id, point_model, normal_model in samples:
        source_segment = raster_segment_id(line_id)
        if previous_source_segment is not None and source_segment != previous_source_segment:
            segment_id += 1
        if previous is not None and raster_base_line_id(line_id) == raster_base_line_id(previous[0]):
            distance = math.dist(point_model, previous[3])
            if distance > jump_limit:
                segment_id += 1
        output.append(
            (
                encoded_raster_line_id(segment_id, raster_base_line_id(line_id)),
                point_id,
                face_id,
                point_model,
                normal_model,
            )
        )
        previous = (line_id, point_id, face_id, point_model, normal_model)
        previous_source_segment = source_segment
    return output


def path_has_split_scanlines(path) -> bool:
    """Detect multiple disconnected runs on one raster line after normal sampling."""
    segments_by_line: dict[int, set[int]] = {}
    for waypoint in path.waypoints:
        segments_by_line.setdefault(raster_base_line_id(waypoint.line_id), set()).add(raster_segment_id(waypoint.line_id))
    return any(len(segments) > 1 for segments in segments_by_line.values())


def placement_for(base, picked_origin, model_x: float, model_y: float, model_z: float, model_rz: float):
    # 同步更新模型安装位姿和 wobj：路径点用工件坐标导出，但窗口判断使用基座/世界坐标。
    placement = base.clone()
    placement.model_x = float(model_x)
    placement.model_y = float(model_y)
    placement.model_z = float(model_z)
    placement.model_rz = float(model_rz)
    placement.wobj_rz = float(model_rz)
    angle = math.radians(model_rz)
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    px, py, pz = picked_origin
    placement.wobj_x = placement.model_x + cos_a * px - sin_a * py
    placement.wobj_y = placement.model_y + sin_a * px + cos_a * py
    placement.wobj_z = placement.model_z + pz
    return placement


def in_window_xy(point: tuple[float, float, float]) -> bool:
    # 第一阶段只支持 box 窗口；后续梯形/椭球窗口可以在这里扩展。
    if WINDOW_SHAPE != "box":
        raise ValueError(f"Unsupported machining window shape: {WINDOW_SHAPE}")
    return WINDOW_X[0] <= point[0] <= WINDOW_X[1] and WINDOW_Y[0] <= point[1] <= WINDOW_Y[1]


def region_vertices_by_id(polydata, regions: list[set[int]]) -> dict[int, list[tuple[float, float, float]]]:
    by_region: dict[int, list[tuple[float, float, float]]] = {}
    for region_index, region in enumerate(regions, 1):
        vertices: list[tuple[float, float, float]] = []
        for triangle in read_triangles(polydata, region):
            vertices.extend(triangle.points)
        by_region[region_index] = vertices
    return by_region


def clip_region_vertices(
    polydata,
    face_ids: set[int],
    clip_polygon: list[list[float]] | None,
    exclude_polygons: list[list[list[float]]] | None = None,
    raster_chart: dict | None = None,
) -> list[tuple[float, float, float]]:
    vertices: list[tuple[float, float, float]] = []
    for triangle in read_triangles(polydata, face_ids):
        centroid = (
            sum(point[0] for point in triangle.points) / 3.0,
            sum(point[1] for point in triangle.points) / 3.0,
        )
        center_3d = (*centroid, sum(point[2] for point in triangle.points) / 3.0)
        clip_point = point_to_uv(center_3d, raster_chart) if raster_chart else centroid
        if not point_allowed_by_clip(clip_point, clip_polygon, exclude_polygons):
            continue
        vertices.extend(triangle.points)
    return vertices


def region_inside_window(vertices_model: list[tuple[float, float, float]], transform: WorkpieceTransform) -> bool:
    # 必须整块 region 都在加工窗口内才导出，避免边缘大角度或干涉风险。
    if not vertices_model:
        return False
    return all(in_window_xy(transform.model_point_to_world(point)) for point in vertices_model)


def confdata_for_world_y(y_value: float) -> tuple[int, int, int, int]:
    # RobotStudio 手动验证得到的两套稳定构型：按基座 Y 正负直接写 confdata。
    return CONF_Y_NEGATIVE if y_value < 0.0 else CONF_Y_NONNEGATIVE


def approach_waypoint(placement, waypoint, distance: float, index: int):
    position = (
        waypoint.position_world[0] + waypoint.normal_world[0] * distance,
        waypoint.position_world[1] + waypoint.normal_world[1] * distance,
        waypoint.position_world[2] + waypoint.normal_world[2] * distance,
    )
    transform = WorkpieceTransform(placement)
    return replace(waypoint, index=index, position_world=position, position_wobj=transform.world_point_to_wobj(position))


def build_motion(placement, path):
    # 每个面单独加安全接近和离开点；面内路径不做频繁翻腕。
    motion = []
    start = 0
    while start < len(path.waypoints):
        segment = raster_segment_id(path.waypoints[start].line_id)
        end = start + 1
        while end < len(path.waypoints) and raster_segment_id(path.waypoints[end].line_id) == segment:
            end += 1
        segment_waypoints = path.waypoints[start:end]
        motion.append(approach_waypoint(placement, segment_waypoints[0], SAFE_DISTANCE, -1))
        motion.extend(segment_waypoints)
        motion.append(approach_waypoint(placement, segment_waypoints[-1], SAFE_DISTANCE, len(path.waypoints)))
        start = end
    return motion


def build_hole_aware_motion(placement, path):
    """Finish each raster cell, then retract and transfer above the surface.

    Hole-aware line segment ids identify complete cells rather than individual
    scanline runs.  build_motion() therefore emits exactly one approach and one
    departure per cell.  RAPID uses MoveL for the local retract/approach and
    MoveJ between the two lifted endpoints, so the tool never needs a supported
    on-surface connector across a hole or mesh gap.
    """
    return build_motion(placement, path)


def base_y_aligned_quaternion(normal_world: tuple[float, float, float], previous: Quaternion | None) -> Quaternion:
    # 姿态锁定在基座方向，而不是锁定到光栅切向，保证工具头整体保持“朝前/超前”。
    z_axis = normalize((-normal_world[0], -normal_world[1], -normal_world[2]))
    y_ref = (0.0, 1.0, 0.0)
    y_projected = (
        y_ref[0] - dot(y_ref, z_axis) * z_axis[0],
        y_ref[1] - dot(y_ref, z_axis) * z_axis[1],
        y_ref[2] - dot(y_ref, z_axis) * z_axis[2],
    )
    if dot(y_projected, y_projected) <= 1e-10:
        x_ref = (1.0, 0.0, 0.0)
        y_projected = (
            x_ref[0] - dot(x_ref, z_axis) * z_axis[0],
            x_ref[1] - dot(x_ref, z_axis) * z_axis[1],
            x_ref[2] - dot(x_ref, z_axis) * z_axis[2],
        )
    y_axis = normalize(y_projected, fallback=(0.0, 1.0, 0.0))
    x_axis = normalize(cross(y_axis, z_axis), fallback=(-1.0, 0.0, 0.0))
    y_axis = normalize(cross(z_axis, x_axis), fallback=y_axis)
    matrix = (
        (x_axis[0], y_axis[0], z_axis[0]),
        (x_axis[1], y_axis[1], z_axis[1]),
        (x_axis[2], y_axis[2], z_axis[2]),
    )
    quaternion = matrix_to_quaternion(matrix)
    if previous is not None and quaternion.dot(previous) < 0.0:
        quaternion = quaternion.negated()
    return quaternion.normalized()


def rapid_experiment_metadata(placement, region_label: str) -> dict:
    """Return self-contained scene-placement metadata for one RAPID module."""
    return {
        "schema": "robot_studio_qt.experiment_installation",
        "version": 1,
        "model_x": placement.model_x,
        "model_y": placement.model_y,
        "model_z": placement.model_z,
        "model_rx": placement.model_rx,
        "model_ry": placement.model_ry,
        "model_rz": placement.model_rz,
        "region_label": region_label,
        "workpiece_name": placement.name,
        "workpiece_file_path": placement.file_path,
        "picked_origin": list(placement.picked_origin),
        "wobj_rx": placement.wobj_rx,
        "wobj_ry": placement.wobj_ry,
    }


def rapid_text(module_name: str, placement, tool, path, motion_waypoints, region_label: str = "") -> str:
    # RAPID robtarget 的位置和姿态都写在 wobj 坐标系下；世界姿态要先转换到 wobj 相对姿态。
    ext_axes = "9E9,9E9,9E9,9E9,9E9,9E9"
    q_wobj = euler_xyz_degrees_to_quaternion(placement.wobj_rx, placement.wobj_ry, placement.wobj_rz)
    q_world_to_wobj = q_wobj.conjugated()
    experiment_metadata = rapid_experiment_metadata(placement, region_label)
    lines = [
        f"MODULE {module_name}",
        f"    ! RSP_EXPERIMENT_META_V1 {json.dumps(experiment_metadata, ensure_ascii=False, separators=(',', ':'))}",
        f"    {tool_to_rapid(tool)}",
        (
            f"    PERS wobjdata {path.workobject_name}:=[FALSE,TRUE,\"\","
            f"[[{format_number(placement.wobj_x)},{format_number(placement.wobj_y)},{format_number(placement.wobj_z)}],"
            f"[{format_number(q_wobj.w)},{format_number(q_wobj.x)},{format_number(q_wobj.y)},{format_number(q_wobj.z)}]],"
            "[[0,0,0],[1,0,0,0]]];"
        ),
    ]
    for seq, waypoint in enumerate(motion_waypoints, 1):
        p = waypoint.position_wobj
        q = q_world_to_wobj.multiplied(waypoint.quaternion).normalized()
        conf = ",".join(str(value) for value in confdata_for_world_y(waypoint.position_world[1]))
        lines.append(
            f"    CONST robtarget p{seq:04d}:="
            f"[[{format_number(p[0])},{format_number(p[1])},{format_number(p[2])}],"
            f"[{format_number(q.w)},{format_number(q.x)},{format_number(q.y)},{format_number(q.z)}],"
            f"[{conf}],[{ext_axes}]];"
        )
    lines.append("    PROC main()")
    # 固定构型策略配合 ConfL Off，避免 ABB 在路径中强制按 confdata 做不必要的线性构型检查。
    lines.append("        ConfL \\Off;")
    if motion_waypoints:
        lines.append(f"        MoveJ p0001,v100,z10,{tool.name}\\WObj:={path.workobject_name};")
        for seq in range(2, len(motion_waypoints) + 1):
            waypoint = motion_waypoints[seq - 1]
            if waypoint.index == -1:
                lines.append(f"        MoveJ p{seq:04d},v100,z10,{tool.name}\\WObj:={path.workobject_name};")
            else:
                zone = "z10" if seq == len(motion_waypoints) else path.settings.zone
                lines.append(f"        MoveL p{seq:04d},v{format_number(path.settings.speed)},{zone},{tool.name}\\WObj:={path.workobject_name};")
    lines.extend(["    ENDPROC", "ENDMODULE", ""])
    return "\n".join(lines)


def write_points_csv(path_file: Path, motion_waypoints) -> None:
    with path_file.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "seq",
                "face_id",
                "line_id",
                "point_id",
                "world_x",
                "world_y",
                "world_z",
                "wobj_x",
                "wobj_y",
                "wobj_z",
                "q1",
                "q2",
                "q3",
                "q4",
                "confdata",
            ]
        )
        for seq, waypoint in enumerate(motion_waypoints, 1):
            q = waypoint.quaternion
            writer.writerow(
                [
                    seq,
                    waypoint.face_id,
                    waypoint.line_id,
                    waypoint.point_id,
                    *[format_number(value) for value in waypoint.position_world],
                    *[format_number(value) for value in waypoint.position_wobj],
                    format_number(q.w),
                    format_number(q.x),
                    format_number(q.y),
                    format_number(q.z),
                    " ".join(str(value) for value in confdata_for_world_y(waypoint.position_world[1])),
                ]
            )


def normalize_tool_load(row: dict) -> None:
    # RobotStudio 不接受完全未定义载荷；这里给实验用占位载荷，真实项目可替换为实测 tooldata。
    for key in ("module", "txt"):
        if not row.get(key):
            continue
        path = Path(row[key])
        text = path.read_text(encoding="utf-8")
        text = text.replace("[1,[0,0,0],[1,0,0,0],0,0,0]", RAPID_LOAD_PLACEHOLDER)
        path.write_text(text, encoding="utf-8")


def module_coord_label(value: float) -> str:
    # ABB MODULE 名不能包含负号；负数坐标用 M 前缀表示，例如 y=-250 -> YM250。
    integer = int(round(value))
    if integer < 0:
        return f"M{abs(integer)}"
    return str(integer)


def pose_file_label(model_x: float, model_y: float, model_z: float, angle: int, region_index: int | None = None) -> str:
    label = (
        f"x{module_coord_label(model_x)}_"
        f"y{module_coord_label(model_y)}_"
        f"z{module_coord_label(model_z)}_"
        f"rz{angle:03d}"
    )
    if region_index is not None:
        label = f"{label}_R{region_index:02d}"
    return label


def rapid_module_name(region_index: int) -> str:
    return f"MODULE_R{region_index:02d}"


def safe_region_label(label: str) -> str:
    cleaned = "".join(character if character.isalnum() or character in {"_", "-"} else "_" for character in str(label).strip())
    return cleaned or "region"


def stable_sign(axis: tuple[float, float, float]) -> tuple[float, float, float]:
    dominant = max(range(3), key=lambda index: abs(axis[index]))
    if axis[dominant] < 0.0:
        return (-axis[0], -axis[1], -axis[2])
    return axis


def point_key(point: tuple[float, float, float]) -> tuple[int, int, int]:
    return tuple(round(value * 1_000_000) for value in point)  # type: ignore[return-value]


def boundary_edges(triangles) -> list[tuple[tuple[float, float, float], tuple[float, float, float]]]:
    # 取 region 外边界边，用于恢复真实局部 UV 方向，避免点云 PCA 把梯形面算成斜光栅。
    edge_map: dict[
        tuple[tuple[int, int, int], tuple[int, int, int]],
        list[tuple[tuple[float, float, float], tuple[float, float, float]]],
    ] = {}
    for triangle in triangles:
        points = triangle.points
        for start, end in ((points[0], points[1]), (points[1], points[2]), (points[2], points[0])):
            start_key = point_key(start)
            end_key = point_key(end)
            key = tuple(sorted((start_key, end_key)))  # type: ignore[assignment]
            edge_map.setdefault(key, []).append((start, end))
    return [entries[0] for entries in edge_map.values() if len(entries) == 1]


def tangent_basis(normal: tuple[float, float, float]) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    candidates: list[tuple[float, tuple[float, float, float]]] = []
    for reference in ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)):
        projected = (
            reference[0] - dot(reference, normal) * normal[0],
            reference[1] - dot(reference, normal) * normal[1],
            reference[2] - dot(reference, normal) * normal[2],
        )
        candidates.append((dot(projected, projected), projected))
    first = normalize(max(candidates, key=lambda item: item[0])[1], fallback=(1.0, 0.0, 0.0))
    second = normalize(cross(normal, first), fallback=(0.0, 1.0, 0.0))
    return first, second


def projected_range(triangles, axis: tuple[float, float, float]) -> float:
    values = [dot(point, axis) for triangle in triangles for point in triangle.points]
    return max(values) - min(values)


def pca_axes_from_region(triangles) -> tuple[tuple[float, float, float], tuple[float, float, float], float, float]:
    # 兜底方案：只有找不到可靠边界边时才使用 PCA 主方向。
    normal = average_normal(triangles)
    origin = mesh_centroid(triangles)
    basis_u, basis_v = tangent_basis(normal)
    coords: list[tuple[float, float]] = []
    for triangle in triangles:
        for point in triangle.points:
            relative = (point[0] - origin[0], point[1] - origin[1], point[2] - origin[2])
            coords.append((dot(relative, basis_u), dot(relative, basis_v)))
    if len(coords) < 2:
        return stable_sign(basis_u), stable_sign(basis_v), 0.0, 0.0
    mean_u = sum(point[0] for point in coords) / len(coords)
    mean_v = sum(point[1] for point in coords) / len(coords)
    cov_uu = sum((point[0] - mean_u) ** 2 for point in coords) / len(coords)
    cov_vv = sum((point[1] - mean_v) ** 2 for point in coords) / len(coords)
    cov_uv = sum((point[0] - mean_u) * (point[1] - mean_v) for point in coords) / len(coords)
    theta = 0.5 * math.atan2(2.0 * cov_uv, cov_uu - cov_vv)
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    axis_a = normalize(
        (
            basis_u[0] * cos_t + basis_v[0] * sin_t,
            basis_u[1] * cos_t + basis_v[1] * sin_t,
            basis_u[2] * cos_t + basis_v[2] * sin_t,
        ),
        fallback=basis_u,
    )
    axis_b = normalize(cross(normal, axis_a), fallback=basis_v)

    ext_a = [dot(point, axis_a) for triangle in triangles for point in triangle.points]
    ext_b = [dot(point, axis_b) for triangle in triangles for point in triangle.points]
    range_a = max(ext_a) - min(ext_a)
    range_b = max(ext_b) - min(ext_b)
    if range_b > range_a:
        axis_a, axis_b = axis_b, normalize(cross(normal, axis_b), fallback=axis_a)
        range_a, range_b = range_b, range_a
    return stable_sign(axis_a), stable_sign(axis_b), range_a, range_b


def uv_axes_from_region(triangles) -> tuple[tuple[float, float, float], tuple[float, float, float], float, float]:
    # 优先根据边界 UV 计算长短边；这是当前实验避免斜线路径的核心规则。
    normal = average_normal(triangles)
    candidates: list[tuple[float, tuple[float, float, float]]] = []
    for start, end in boundary_edges(triangles):
        vector = (end[0] - start[0], end[1] - start[1], end[2] - start[2])
        projected = (
            vector[0] - dot(vector, normal) * normal[0],
            vector[1] - dot(vector, normal) * normal[1],
            vector[2] - dot(vector, normal) * normal[2],
        )
        length_sq = dot(projected, projected)
        if length_sq <= 1e-8:
            continue
        candidates.append((math.sqrt(length_sq), stable_sign(normalize(projected))))

    if not candidates:
        return pca_axes_from_region(triangles)

    def alignment_score(axis: tuple[float, float, float]) -> float:
        return sum(length * abs(dot(axis, candidate_axis)) ** 4 for length, candidate_axis in candidates)

    primary = max((axis for _length, axis in candidates), key=alignment_score)
    secondary = normalize(cross(normal, primary), fallback=(0.0, 1.0, 0.0))
    range_primary = projected_range(triangles, primary)
    range_secondary = projected_range(triangles, secondary)
    if range_secondary > range_primary:
        primary, secondary = secondary, normalize(cross(normal, secondary), fallback=primary)
        range_primary, range_secondary = range_secondary, range_primary
    return stable_sign(primary), stable_sign(secondary), range_primary, range_secondary


def plan_region_uv(
    polydata,
    placement,
    base_settings: RasterPlannerSettings,
    region: set[int],
    feed_variant: RasterFeedDirection,
    clip_polygon: list[list[float]] | None = None,
    exclude_polygons: list[list[list[float]]] | None = None,
    raster_chart: dict | None = None,
) -> PathResult:
    # 对单个 region 独立生成光栅路径，路径姿态由 base_y_aligned 统一决定。
    triangles = read_triangles(polydata, region)
    if not triangles:
        return PathResult(PathSource.MESH, placement.name, base_settings, message="Mesh has no selected triangular surface cells.")
    settings = replace(base_settings, feed_direction=feed_variant)
    if raster_chart and clip_polygon:
        domain_samples = raster_samples(
            clip_polygon,
            exclude_polygons or [],
            triangles,
            raster_chart,
            settings.spacing,
            settings.point_step,
            settings.boundary_margin,
            settings.bidirectional,
            feed_variant == RasterFeedDirection.LONG_SIDE,
        )
        samples = [
            (encoded_raster_line_id(segment_id, line_id), point_id, face_id, point_model, normal_model)
            for segment_id, line_id, point_id, face_id, point_model, normal_model in domain_samples
        ]
    else:
        normal = average_normal(triangles)
        origin = mesh_centroid(triangles)
        long_axis, short_axis, _long_range, _short_range = uv_axes_from_region(triangles)
        if feed_variant == RasterFeedDirection.LONG_SIDE:
            u_axis = long_axis
            v_axis = normalize(cross(normal, u_axis), fallback=short_axis)
        else:
            u_axis = short_axis
            v_axis = normalize(cross(normal, u_axis), fallback=long_axis)
        projected = [project_triangle(triangle, origin, u_axis, v_axis) for triangle in triangles]
        samples = sample_projected_mesh(projected, settings)
        if clip_polygon or exclude_polygons:
            samples = [sample for sample in samples if point_allowed_by_clip((sample[3][0], sample[3][1]), clip_polygon, exclude_polygons)]
        samples = split_discontinuous_raster_segments(samples, settings.point_step)
    if not samples:
        return PathResult(PathSource.MESH, placement.name, settings, message="No raster samples were generated.")

    transform = WorkpieceTransform(placement)
    previous = None
    waypoints: list[Waypoint] = []
    for index, (line_id, point_id, face_id, point_model, normal_model) in enumerate(samples):
        point_world = transform.model_point_to_world(point_model)
        normal_world = transform.model_vector_to_world(normal_model)
        quaternion = base_y_aligned_quaternion(normal_world, previous)
        previous = quaternion
        waypoints.append(
            Waypoint(
                index=index,
                source=PathSource.MESH,
                region_id=0,
                face_id=face_id,
                line_id=line_id,
                point_id=point_id,
                position_model=point_model,
                position_world=point_world,
                position_wobj=transform.world_point_to_wobj(point_world),
                normal_world=normal_world,
                normal_wobj=transform.world_vector_to_wobj(normal_world),
                quaternion=quaternion,
            )
        )
    return PathResult(PathSource.MESH, placement.name or "wobj0", settings, waypoints, f"Generated {len(waypoints)} UV {feed_variant.value} raster waypoints.")


def plan_region_uv_hole_aware(
    polydata,
    placement,
    base_settings: RasterPlannerSettings,
    region: set[int],
    feed_variant: RasterFeedDirection,
    clip_polygon: list[list[float]] | None = None,
    exclude_polygons: list[list[list[float]]] | None = None,
    raster_chart: dict | None = None,
) -> PathResult:
    """Plan complete raster cells with lifted transfers between cells."""
    settings = replace(base_settings, feed_direction=feed_variant)
    has_chart_domain = bool(raster_chart and clip_polygon)
    has_partial_domain = bool(raster_chart) != bool(clip_polygon)
    if has_partial_domain or (not has_chart_domain and bool(exclude_polygons)):
        return PathResult(
            PathSource.MESH,
            placement.name,
            settings,
            message="Hole-aware planning requires raster_chart and clip_polygon together when a manual domain is supplied.",
        )
    triangles = read_triangles(polydata, region)
    if not triangles:
        return PathResult(PathSource.MESH, placement.name, settings, message="Mesh has no selected triangular surface cells.")
    if has_chart_domain:
        domain_kind = "manual-v2"
        domain_samples, diagnostics = hole_aware_raster_samples(
            clip_polygon,
            exclude_polygons or [],
            triangles,
            raster_chart,
            settings.spacing,
            settings.point_step,
            settings.boundary_margin,
            settings.bidirectional,
            feed_variant == RasterFeedDirection.LONG_SIDE,
        )
    else:
        domain_kind = "projected-face-id"
        normal = average_normal(triangles)
        origin = mesh_centroid(triangles)
        long_axis, short_axis, _long_range, _short_range = uv_axes_from_region(triangles)
        if feed_variant == RasterFeedDirection.LONG_SIDE:
            u_axis = long_axis
            v_axis = normalize(cross(normal, u_axis), fallback=short_axis)
        else:
            u_axis = short_axis
            v_axis = normalize(cross(normal, u_axis), fallback=long_axis)
        projected = [project_triangle(triangle, origin, u_axis, v_axis) for triangle in triangles]
        projected_samples = sample_projected_mesh(projected, settings)
        projected_samples = split_discontinuous_raster_segments(projected_samples, settings.point_step)
        domain_samples, diagnostics = projected_raster_cell_samples(
            projected_samples,
            origin,
            u_axis,
            settings.spacing,
        )
    if not domain_samples:
        return PathResult(
            PathSource.MESH,
            placement.name,
            settings,
            message=f"Hole-aware planning failed: {diagnostics.get('reason', 'no valid samples')}",
        )
    transform = WorkpieceTransform(placement)
    previous = None
    waypoints: list[Waypoint] = []
    for index, (segment_id, line_id, point_id, face_id, point_model, normal_model) in enumerate(domain_samples):
        point_world = transform.model_point_to_world(point_model)
        normal_world = transform.model_vector_to_world(normal_model)
        quaternion = base_y_aligned_quaternion(normal_world, previous)
        previous = quaternion
        waypoints.append(
            Waypoint(
                index=index,
                source=PathSource.MESH,
                region_id=0,
                face_id=face_id,
                line_id=encoded_raster_line_id(segment_id, line_id),
                point_id=point_id,
                position_model=point_model,
                position_world=point_world,
                position_wobj=transform.world_point_to_wobj(point_world),
                normal_world=normal_world,
                normal_wobj=transform.world_vector_to_wobj(normal_world),
                quaternion=quaternion,
            )
        )
    return PathResult(
        PathSource.MESH,
        placement.name or "wobj0",
        settings,
        waypoints,
        (
            f"Generated {len(waypoints)} hole-aware waypoints in "
            f"{diagnostics['cell_count']} cells with {diagnostics['transfer_count']} lifted transfers "
            f"from {domain_kind}."
        ),
    )


def plan_region_uv_auto(
    polydata,
    placement,
    base_settings: RasterPlannerSettings,
    region: set[int],
    feed_variant: RasterFeedDirection,
    clip_polygon: list[list[float]] | None = None,
    exclude_polygons: list[list[list[float]]] | None = None,
    raster_chart: dict | None = None,
) -> tuple[PathResult, bool, str]:
    """Select regular raster or cell-lift planning and report the reason.

    Explicit excludes with positive-area overlap include both a true hole fully
    contained by the clip polygon and an exclusion that cuts through its edge.
    Even without an explicit exclude, multiple runs on one scanline reveal an
    unsupported surface gap and switch to the same safe cell-transfer motion.
    """
    holes = exclude_polygons or []
    if polygon_has_relevant_holes(clip_polygon, holes):
        return (
            plan_region_uv_hole_aware(
                polydata,
                placement,
                base_settings,
                region,
                feed_variant,
                clip_polygon,
                holes,
                raster_chart,
            ),
            True,
            "exclude-overlap",
        )

    regular_path = plan_region_uv(
        polydata,
        placement,
        base_settings,
        region,
        feed_variant,
        clip_polygon,
        holes,
        raster_chart,
    )
    if path_has_split_scanlines(regular_path):
        return (
            plan_region_uv_hole_aware(
                polydata,
                placement,
                base_settings,
                region,
                feed_variant,
                clip_polygon,
                holes,
                raster_chart,
            ),
            True,
            "split-scanline",
        )
    return regular_path, False, "regular-raster"


def export_path_variant(
    project,
    placement,
    path,
    angle: int,
    region_index: int,
    variant: str,
    region_label: str | None = None,
    hole_aware: bool = False,
    planner_label: str | None = None,
) -> dict:
    # 每个 region、每个角度、每个进给变体独立保存，便于 RobotStudio 单独验证。
    motion = build_hole_aware_motion(placement, path) if hole_aware else build_motion(placement, path)
    label = safe_region_label(region_label or f"{region_index}")
    folder = OUTDIR / f"rz{angle:03d}" / label / variant
    folder.mkdir(parents=True, exist_ok=True)
    module_name = rapid_module_name(region_index)
    file_stem = label
    rapid = rapid_text(module_name, placement, project.polishing_tool, path, motion, label)
    txt_path = folder / f"{file_stem}.txt"
    txt_path.write_text(rapid, encoding="utf-8")
    row = {
        "angle_deg": angle,
        "region": region_index,
        "region_label": label,
        "feed_variant": variant,
        "targets": len(motion),
        "negative_y_targets": sum(1 for waypoint in motion if waypoint.position_world[1] < 0.0),
        "nonnegative_y_targets": sum(1 for waypoint in motion if waypoint.position_world[1] >= 0.0),
        "module": "",
        "txt": str(txt_path),
        "points_csv": "",
        "orientation_mode": ORIENTATION_MODE,
        "axis_mode": AXIS_MODE,
        "planner": planner_label or ("hole-aware" if hole_aware else "legacy"),
        "motion_strategy": "cell-lift-transfer" if hole_aware else "segment-lift-transfer",
    }
    normalize_tool_load(row)
    return row


def write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    keys: list[str] = []
    for row in rows:
        for key in row:
            if key not in keys:
                keys.append(key)
    with path.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    # 主流程：读取已保存选面 -> 扫描转台角度 -> 加工窗口筛 region -> 导出 RAPID 和汇总表。
    OUTDIR.mkdir(parents=True, exist_ok=True)
    project = load_project_file(PROJECT_PATH)
    regions = [set(region) for region in project.selected_path_face_regions]
    if not regions:
        raise RuntimeError(f"No selected_path_face_regions in {PROJECT_PATH}")
    planning_regions = manual_clip_regions(PROJECT_PATH, regions)
    if project.polishing_tool.mass_kg <= 0.0:
        project.polishing_tool.mass_kg = TOOL_LOAD_MASS_KG

    importer = CadImportService().import_model(project.workpiece.file_path)
    reader = create_mesh_reader(importer.display_path, importer.display_format)
    reader.SetFileName(str(importer.display_path))
    reader.Update()
    polydata = reader.GetOutput()

    settings = RasterPlannerSettings(
        spacing=SPACING,
        point_step=POINT_STEP,
        angle_degrees=0.0,
        boundary_margin=BOUNDARY_MARGIN,
        bidirectional=True,
        feed_direction=RasterFeedDirection.LONG_SIDE,
        start_corner=StartCorner.LOWER_LEFT,
        tool_axis="-z",
        speed=100.0,
        zone="z1",
        tool_name=project.polishing_tool.name,
    )
    vertices_by_region = {
        index: clip_region_vertices(polydata, item["face_ids"], item.get("clip_polygon"), item.get("exclude_polygons"), item.get("raster_chart"))
        for index, item in enumerate(planning_regions, 1)
    }

    adjusted_project = project
    adjusted_project.workpiece = placement_for(project.workpiece, project.workpiece.picked_origin, MODEL_X, MODEL_Y, MODEL_Z, 0.0)
    save_project_file(
        OUTDIR / f"{pose_file_label(MODEL_X, MODEL_Y, MODEL_Z, 0)}.rsp.json",
        adjusted_project,
    )

    coverage_rows: list[dict] = []
    matrix_rows: list[dict] = []
    export_rows: list[dict] = []
    deferred_rows: list[dict] = []

    for angle in ANGLES:
        # 每个转台角度下重新计算工件位姿，并在基座坐标下判断哪些 region 完整落入窗口。
        placement = placement_for(project.workpiece, project.workpiece.picked_origin, MODEL_X, MODEL_Y, MODEL_Z, float(angle))
        transform = WorkpieceTransform(placement)
        inside_regions: list[int] = []
        matrix_row = {"angle_deg": angle}
        for region_index, planning_region in enumerate(planning_regions, 1):
            inside = region_inside_window(vertices_by_region[region_index], transform)
            region_label = planning_region["label"]
            matrix_row[f"region_{region_index:02d}"] = region_label if inside else ""
            if inside:
                inside_regions.append(region_index)
                for variant_name, feed_variant in FEED_VARIANTS:
                    # 第一阶段通常只跑 long_side；如果需要复核，可把 FEED_VARIANTS 改为长短边两套。
                    path = plan_region_uv(
                        polydata,
                        placement,
                        settings,
                        planning_region["face_ids"],
                        feed_variant,
                        planning_region.get("clip_polygon"),
                        planning_region.get("exclude_polygons"),
                        planning_region.get("raster_chart"),
                    )
                    if path.waypoints:
                        row = export_path_variant(project, placement, path, angle, region_index, variant_name, region_label)
                        row["source_region"] = planning_region["source_region"]
                        export_rows.append(row)
                    else:
                        deferred_rows.append({"angle_deg": angle, "region": region_index, "region_label": region_label, "feed_variant": variant_name, "reason": path.message})
        matrix_rows.append(matrix_row)
        coverage_rows.append(
            {
                "angle_deg": angle,
                "region_count": len(inside_regions),
                "region_ids": " ".join(str(planning_regions[value - 1]["label"]) for value in inside_regions),
            }
        )

    write_csv(OUTDIR / "angle_region_table.csv", matrix_rows)
    write_csv(OUTDIR / "coverage_by_angle.csv", coverage_rows)
    write_csv(OUTDIR / "exported_paths.csv", export_rows)
    write_csv(OUTDIR / "deferred_paths.csv", deferred_rows)

    summary = {
        "input_project": str(PROJECT_PATH),
        "selected_region_count": len(regions),
        "planning_region_count": len(planning_regions),
        "output_dir": str(OUTDIR),
        "model_x": MODEL_X,
        "model_y": MODEL_Y,
        "model_z": MODEL_Z,
        "angles_deg": ANGLES,
        "window_base_xy": {"x": WINDOW_X, "y": WINDOW_Y},
        "window_shape": WINDOW_SHAPE,
        "spacing_mm": SPACING,
        "point_step_mm": POINT_STEP,
        "boundary_margin_mm": BOUNDARY_MARGIN,
        "orientation_mode": ORIENTATION_MODE,
        "axis_mode": AXIS_MODE,
        "feed_variants": [variant for variant, _feed in FEED_VARIANTS],
        "conf_y_negative": CONF_Y_NEGATIVE,
        "conf_y_nonnegative": CONF_Y_NONNEGATIVE,
        "tool_load_placeholder": RAPID_LOAD_PLACEHOLDER,
        "confl_off": True,
        "coverage": coverage_rows,
        "exported_count": len(export_rows),
        "deferred_count": len(deferred_rows),
    }
    (OUTDIR / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")

    report_lines = [
        "# Window confdata export - UV long/short raster",
        "",
        f"- Input project: `{PROJECT_PATH}`",
        f"- Selected regions: {len(regions)}",
        f"- Workpiece model pose: x={MODEL_X:.0f}, y={MODEL_Y:.0f}, z={MODEL_Z:.0f}, rz={ANGLES[0]}..{ANGLES[-1]} step {ANGLES[1] - ANGLES[0] if len(ANGLES) > 1 else 0}.",
        f"- Base window: x[{WINDOW_X[0]:.0f},{WINDOW_X[1]:.0f}], y[{WINDOW_Y[0]:.0f},{WINDOW_Y[1]:.0f}].",
        f"- Window shape: `{WINDOW_SHAPE}`.",
        "- Raster axes: per-region boundary UV long edge and short edge; PCA is only a fallback.",
        "- Orientation: base_y_aligned from the previous accepted version.",
        f"- Confdata: y<0 -> {CONF_Y_NEGATIVE}; y>=0 -> {CONF_Y_NONNEGATIVE}.",
        "- RAPID main includes `ConfL \\Off;`.",
        f"- Tool load placeholder: `{RAPID_LOAD_PLACEHOLDER}`.",
        "",
        "## Coverage By Angle",
        "",
        "| Angle | Count | Regions |",
        "|---:|---:|---|",
    ]
    for row in coverage_rows:
        report_lines.append(f"| {row['angle_deg']} | {row['region_count']} | {row['region_ids']} |")
    report_lines.extend(
        [
            "",
            "## Files",
            "",
            "- `angle_region_table.csv`: first column is angle, following columns show regions inside the machining window.",
            "- `exported_paths.csv`: exported RAPID file index, with `feed_variant=long_side/short_side`.",
        ]
    )
    (OUTDIR / "experiment_report.md").write_text("\n".join(report_lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
