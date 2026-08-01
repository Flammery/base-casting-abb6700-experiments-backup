from __future__ import annotations

import json
from pathlib import Path
import re
import sys

EXPERIMENT_DIR = Path(__file__).resolve().parents[1]
ROOT = EXPERIMENT_DIR.parents[1]
DEFAULT_INPUT = EXPERIMENT_DIR / "inputs" / "latest_script_test.rsp.json"
DEFAULT_PARTITIONED = EXPERIMENT_DIR / "inputs" / "latest_partitioned.rsp.json"
PARTITION_SCRIPT = EXPERIMENT_DIR / "scripts" / "region_partition_preprocess.py"
RUNNER_SCRIPT = EXPERIMENT_DIR / "scripts" / "configurable_experiment_runner.py"
SCRIPT_DIR = EXPERIMENT_DIR / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from region_selectors import parse_region_selectors, validate_selectors  # noqa: E402

DEFAULT_X = "3700"
DEFAULT_Y = "-1900,100,1900"
DEFAULT_Z = "440"
DEFAULT_BOUNDARY_MARGIN = "6"
DEFAULT_TURNTABLE_ANGLES = "270"


def parse_region_text(raw: str) -> list[int]:
    if not raw.strip():
        return []
    values: list[int] = []
    seen: set[int] = set()
    for item in raw.split(","):
        item = item.strip()
        if not item:
            continue
        value = int(item)
        if value <= 0:
            raise ValueError("region 编号必须是 1-based 正整数")
        if value not in seen:
            seen.add(value)
            values.append(value)
    return values


def validate_regions(values: list[int], region_count: int) -> None:
    invalid = [value for value in values if value > region_count]
    if invalid:
        raise ValueError(f"region 超出当前输入范围: {invalid}; 当前共有 {region_count} 个 region")


def region_text(values: list[int]) -> str:
    return ",".join(str(value) for value in values)


def parse_coordinate_text(raw: str, default: str = "") -> tuple[list[float], bool]:
    text = default if not raw.strip() else raw.strip()
    if not text:
        raise ValueError("坐标不能为空")
    parts = [item.strip() for item in text.split(",") if item.strip()]
    if len(parts) == 1:
        return [float(parts[0])], False
    if len(parts) != 3:
        raise ValueError("坐标格式应为固定值或 start,step,stop")
    start = float(parts[0])
    step = float(parts[1])
    stop = float(parts[2])
    if step <= 0:
        raise ValueError("坐标范围 step 必须大于 0")
    if stop < start:
        raise ValueError("坐标范围 stop 必须大于等于 start")
    values: list[float] = []
    current = start
    epsilon = abs(step) * 1e-9
    while current <= stop + epsilon:
        values.append(round(current, 10))
        current += step
    return values, len(values) > 1


def scan_axis_for_coordinates(x_text: str, y_text: str, z_text: str) -> str:
    parsed = {
        "x": parse_coordinate_text(x_text, DEFAULT_X),
        "y": parse_coordinate_text(y_text, DEFAULT_Y),
        "z": parse_coordinate_text(z_text, DEFAULT_Z),
    }
    ranged = [axis for axis, (_values, is_range) in parsed.items() if is_range]
    if len(ranged) > 1:
        raise ValueError("X/Y/Z 每次只能有一个范围输入")
    return ranged[0] if ranged else "none"


def parse_turntable_angle_text(raw: str, default: str = DEFAULT_TURNTABLE_ANGLES) -> list[int]:
    text = default if not raw.strip() else raw.strip()
    range_match = re.fullmatch(r"\s*(\d+)\s*[-:]\s*(\d+)\s*[-:]\s*(\d+)\s*", text)
    if range_match is not None:
        start, step, stop = (int(value) for value in range_match.groups())
        if step <= 0:
            raise ValueError("转台角度范围的 step 必须大于 0")
        if stop < start:
            raise ValueError("转台角度范围的 stop 必须大于等于 start")
        tokens = [str(value) for value in range(start, stop + 1, step)]
    else:
        tokens = re.split(r"[,，、;；]+", text)

    values: list[int] = []
    seen: set[int] = set()
    for item in tokens:
        token = item.strip()
        if not token:
            continue
        try:
            value = int(token) % 360
        except ValueError as exc:
            raise ValueError(
                "转台角度应为逗号分隔的整数或 start-step-stop，例如 0,180 或 0-30-330"
            ) from exc
        if value not in seen:
            seen.add(value)
            values.append(value)
    if not values:
        raise ValueError("至少填写一个转台角度，例如 270、0,180 或 0-30-330")
    return values


def turntable_angle_args(raw: str) -> list[str]:
    values = parse_turntable_angle_text(raw)
    return ["--experiment-mode", "turntable", "--angles", ",".join(str(value) for value in values)]


def validated_avoidance_settings(
    payload: dict,
    project_path: Path,
    planning_regions: list[dict],
) -> tuple[list[str], list[str]]:
    """Validate a saved avoidance sidecar against the current partition."""

    configured_project = Path(str(payload.get("input_project", "")))
    if configured_project.resolve() != Path(project_path).resolve():
        raise ValueError("设置对应的输入项目与当前项目不一致")
    selectors = [str(value) for value in payload.get("selectors", [])]
    validate_selectors(selectors, planning_regions)
    labels = [str(record.get("region_label", "")).replace("_", "-") for record in payload.get("regions", [])]
    return selectors, labels


def parse_custom_window_text(raw: str) -> dict[str, tuple[float, float] | None]:
    text = raw.strip()
    if not text:
        return {"x": None, "y": None, "z": None}
    chunks = [chunk.strip() for chunk in text.split(";")]
    if len(chunks) not in (2, 3):
        raise ValueError("加工范围格式应为 xMin,xMax;yMin,yMax 或 xMin,xMax;yMin,yMax;zMin,zMax")
    if len(chunks) == 2:
        chunks.append("")

    result: dict[str, tuple[float, float] | None] = {}
    for axis, chunk in zip(("x", "y", "z"), chunks):
        if not chunk:
            result[axis] = None
            continue
        values = [item.strip() for item in chunk.split(",")]
        if len(values) != 2 or not values[0] or not values[1]:
            raise ValueError(f"{axis} 轴加工范围需要 min,max")
        minimum = float(values[0])
        maximum = float(values[1])
        if maximum < minimum:
            raise ValueError(f"{axis} 轴 max 必须大于等于 min")
        result[axis] = (minimum, maximum)
    return result


def parse_boundary_margin_text(raw: str, default: str = DEFAULT_BOUNDARY_MARGIN) -> float:
    text = default if not raw.strip() else raw.strip()
    value = float(text)
    if value < 0.0:
        raise ValueError("边缘余量必须大于等于 0")
    return value


def read_region_count(path: Path) -> int:
    data = json.loads(path.read_text(encoding="utf-8"))
    regions = data.get("selected_path_face_regions") or []
    return len(regions)


def partition_command(
    python_exe: str,
    input_path: Path,
    output_path: Path,
    regions_raw: str,
    region_count: int | None = None,
) -> list[str]:
    regions = parse_region_text(regions_raw)
    if not regions:
        raise ValueError("未填写需要分区的 region")
    if region_count is not None:
        validate_regions(regions, region_count)
    return [
        python_exe,
        str(PARTITION_SCRIPT),
        "--regions",
        region_text(regions),
        "--input",
        str(input_path),
        "--output",
        str(output_path),
    ]


def runner_command(
    python_exe: str,
    project_path: Path,
    model_x: str,
    model_y: str,
    model_z: str,
    turntable_angles: str,
    window_text: str,
    boundary_margin_text: str = DEFAULT_BOUNDARY_MARGIN,
    planner: str = "legacy",
    avoidance_regions: str = "",
    robot_config_path: Path | None = None,
    avoidance_settings_path: Path | None = None,
) -> list[str]:
    scan_axis_for_coordinates(model_x, model_y, model_z)
    boundary_margin = parse_boundary_margin_text(boundary_margin_text)
    command = [
        python_exe,
        str(RUNNER_SCRIPT),
        "--project",
        str(project_path),
        f"--model-x={model_x.strip() or DEFAULT_X}",
        f"--model-y={model_y.strip() or DEFAULT_Y}",
        f"--model-z={model_z.strip() or DEFAULT_Z}",
        f"--boundary-margin={boundary_margin:g}",
        *turntable_angle_args(turntable_angles),
    ]
    if planner not in {"legacy", "auto", "hole-aware"}:
        raise ValueError(f"未知路径策略: {planner}")
    if planner != "legacy":
        command.extend(["--planner", planner])
    avoidance_selectors = parse_region_selectors(avoidance_regions)
    if avoidance_selectors:
        command.extend(["--avoidance-regions", ",".join(avoidance_selectors)])
    if avoidance_settings_path is not None:
        settings_path = Path(avoidance_settings_path)
        if not settings_path.is_file():
            raise ValueError(f"避障设置文件不存在: {settings_path}")
        command.extend(["--avoidance-settings", str(settings_path)])
    if robot_config_path is not None:
        config_path = Path(robot_config_path)
        if not config_path.is_file():
            raise ValueError(f"杆系配置文件不存在: {config_path}")
        command.extend(["--robot-config", str(config_path)])
    limits = parse_custom_window_text(window_text)
    if all(limit is None for limit in limits.values()):
        command.extend(["--window-mode", "unlimited"])
    else:
        command.extend(["--window-mode", "custom"])
        for axis in ("x", "y", "z"):
            limit = limits[axis]
            if limit is None:
                continue
            command.extend([f"--{axis}-min", str(limit[0]), f"--{axis}-max", str(limit[1])])
    return command
