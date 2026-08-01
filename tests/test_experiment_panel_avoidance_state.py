from __future__ import annotations

from pathlib import Path
import sys
from types import SimpleNamespace


UI_DIR = Path(__file__).resolve().parents[1] / "ui"
if str(UI_DIR) not in sys.path:
    sys.path.insert(0, str(UI_DIR))

import experiment_panel as panel_module  # noqa: E402


class _FakeLabel:
    def __init__(self) -> None:
        self.text = ""
        self.tooltip = ""

    def setText(self, value: str) -> None:
        self.text = value

    def setToolTip(self, value: str) -> None:
        self.tooltip = value


def _bare_panel() -> panel_module.ExperimentPanel:
    panel = panel_module.ExperimentPanel.__new__(panel_module.ExperimentPanel)
    panel.avoidance_settings_label = _FakeLabel()
    panel._clear_avoidance_settings()
    return panel


def test_saved_sidecar_is_visible_but_not_armed(monkeypatch, tmp_path) -> None:
    project_path = tmp_path / "input.rsp.json"
    settings_path = tmp_path / "input_avoidance.json"
    settings_path.write_text("{}", encoding="utf-8")
    panel = _bare_panel()

    monkeypatch.setattr(
        panel_module.path_preview_backend,
        "avoidance_settings_path_for",
        lambda _project_path: settings_path,
    )
    monkeypatch.setattr(
        panel_module.path_preview_backend,
        "load_avoidance_settings",
        lambda _settings_path: {"regions": [{"region_label": "6_1"}]},
    )

    assert not panel._refresh_avoidance_settings(project_path, activate=False)
    assert not panel._avoidance_armed
    assert panel.avoidance_settings_path is None
    assert panel.avoidance_selectors == []
    assert "本次未启用" in panel.avoidance_settings_label.text


def test_apply_arms_once_and_refresh_disarms_without_deleting_sidecar(monkeypatch, tmp_path) -> None:
    project_path = tmp_path / "input.rsp.json"
    settings_path = tmp_path / "input_avoidance.json"
    settings_path.write_text("{}", encoding="utf-8")
    panel = _bare_panel()
    payload = {"regions": [{"region_label": "6_1"}]}

    monkeypatch.setattr(
        panel_module.path_preview_backend,
        "avoidance_settings_path_for",
        lambda _project_path: settings_path,
    )
    monkeypatch.setattr(
        panel_module.path_preview_backend,
        "load_avoidance_settings",
        lambda _settings_path: payload,
    )
    monkeypatch.setattr(
        panel_module.path_preview_backend,
        "load_project_file",
        lambda _project_path: SimpleNamespace(selected_path_face_regions=[{1, 2}]),
    )
    monkeypatch.setattr(
        panel_module.path_preview_backend,
        "manual_clip_regions",
        lambda _project_path, _regions: [{"label": "6_1", "source_region": 6}],
    )
    monkeypatch.setattr(
        panel_module,
        "validated_avoidance_settings",
        lambda _payload, _project_path, _planning_regions: (["6_1"], ["6-1"]),
    )

    assert panel._refresh_avoidance_settings(project_path, activate=True)
    assert panel._avoidance_armed
    assert panel.avoidance_settings_path == settings_path
    assert panel.avoidance_selectors == ["6_1"]
    assert "下一次运行启用避障" in panel.avoidance_settings_label.text

    assert not panel._refresh_avoidance_settings(project_path, activate=False)
    assert not panel._avoidance_armed
    assert panel.avoidance_settings_path is None
    assert settings_path.exists()
    assert "本次未启用" in panel.avoidance_settings_label.text
