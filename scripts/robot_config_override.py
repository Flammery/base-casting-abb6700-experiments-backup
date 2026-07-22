"""Load a main-application robot configuration for avoidance experiments."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import json
from pathlib import Path

from robot_studio_qt.kinematics.kinematics import display_radius_for_joint
from robot_studio_qt.kinematics.model import JointState, RobotConfiguration
from robot_studio_qt.project import load_mechanism_configuration_file
from robot_studio_qt.tools.reachability.collision import CollisionSettings


EXPECTED_SCHEMA = "robot_studio_mechanism_config"
SUPPORTED_JOINT_COUNT = 6


@dataclass(frozen=True)
class RobotConfigOverride:
    path: Path
    robot_config: RobotConfiguration
    joint_state: JointState

    @property
    def name(self) -> str:
        return str(self.robot_config.name)

    def collision_settings(self) -> CollisionSettings:
        """Use radii derived from the imported per-link envelope fields."""

        return CollisionSettings(
            link_radius=0.0,
            clearance=0.0,
            ignore_tcp_segment=False,
            use_segment_radius=True,
        )

    def envelope_rows(self) -> list[dict]:
        return [
            {
                "joint": index,
                "label": joint.label,
                "envelope_width_mm": float(joint.envelope_width),
                "envelope_height_mm": float(joint.envelope_height),
                "collision_radius_mm": float(display_radius_for_joint(joint, index - 1)),
            }
            for index, joint in enumerate(self.robot_config.active_joints(), 1)
        ]

    def apply_to_project(self, project) -> None:
        """Apply the same serial configuration and seed state as the main UI import."""

        project.robot_config = deepcopy(self.robot_config)
        project.joint_state = deepcopy(self.joint_state)
        project.joint_state.ensure_count(project.robot_config.joint_count)


def load_robot_config_override(path: str | Path) -> RobotConfigOverride:
    config_path = Path(path).resolve()
    if not config_path.is_file():
        raise ValueError(f"杆系配置文件不存在: {config_path}")

    try:
        payload = json.loads(config_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"杆系配置不是有效的 UTF-8 JSON: {exc}") from exc
    if payload.get("schema") != EXPECTED_SCHEMA:
        raise ValueError(f"杆系配置 schema 必须为 {EXPECTED_SCHEMA}")

    document = load_mechanism_configuration_file(config_path)
    config = document.robot_config
    if config.kinematic_model != "mdh":
        raise ValueError("当前 ABB 6700 避障实验只接受 kinematic_model=mdh 的杆系配置")
    if config.joint_count != SUPPORTED_JOINT_COUNT:
        raise ValueError(f"当前 ABB 6700 避障实验要求 {SUPPORTED_JOINT_COUNT} 轴杆系配置")

    missing_envelopes = [
        joint.label
        for joint in config.active_joints()
        if joint.envelope_width <= 0.0 or joint.envelope_height <= 0.0
    ]
    if missing_envelopes:
        raise ValueError(f"以下杆件缺少有效碰撞包络尺寸: {', '.join(missing_envelopes)}")

    return RobotConfigOverride(config_path, config, document.joint_state)
