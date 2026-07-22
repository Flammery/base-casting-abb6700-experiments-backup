from __future__ import annotations

from pathlib import Path
import sys


EXPERIMENT_DIR = Path(__file__).resolve().parents[1]
ROOT = EXPERIMENT_DIR.parents[1]
for folder in (ROOT / "src", EXPERIMENT_DIR / "scripts"):
    if str(folder) not in sys.path:
        sys.path.insert(0, str(folder))

import pytest  # noqa: E402

from robot_config_override import load_robot_config_override  # noqa: E402
from robot_studio_qt.kinematics.mechanism import (  # noqa: E402
    mechanism_from_robot_configuration,
    mechanism_state_from_robot_state,
)
from robot_studio_qt.kinematics.model import JointState, RobotConfiguration  # noqa: E402
from robot_studio_qt.project import save_mechanism_configuration_file  # noqa: E402


def _write_config(path: Path, config: RobotConfiguration | None = None) -> Path:
    robot_config = config or RobotConfiguration()
    state = JointState([0.0] * robot_config.joint_count)
    save_mechanism_configuration_file(
        path,
        robot_config,
        state,
        mechanism_from_robot_configuration(robot_config),
        mechanism_state_from_robot_state(state, robot_config),
    )
    return path


def test_load_override_uses_exported_mdh_and_link_envelopes(tmp_path) -> None:
    override = load_robot_config_override(_write_config(tmp_path / "abb.rsc.json"))

    assert override.name == "ABB 6700 Style"
    assert override.robot_config.kinematic_model == "mdh"
    assert override.collision_settings().use_segment_radius is True
    assert [row["collision_radius_mm"] for row in override.envelope_rows()] == pytest.approx(
        [198.0, 148.5, 144.0, 117.0, 103.5, 90.0]
    )


def test_load_override_rejects_missing_envelope_dimensions(tmp_path) -> None:
    config = RobotConfiguration()
    config.joints[2].envelope_width = 0.0

    with pytest.raises(ValueError, match="缺少有效碰撞包络尺寸"):
        load_robot_config_override(_write_config(tmp_path / "invalid.rsc.json", config))
