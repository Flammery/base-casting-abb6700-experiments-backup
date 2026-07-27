from __future__ import annotations

from dataclasses import asdict
from pathlib import Path
import sys

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import (
    QDialog,
    QDoubleSpinBox,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
)


UI_DIR = Path(__file__).resolve().parent
SCRIPT_DIR = UI_DIR.parent / "scripts"
if str(UI_DIR) not in sys.path:
    sys.path.insert(0, str(UI_DIR))
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from region_viewer import RegionPreview  # noqa: E402
import window_conf_export as backend  # noqa: E402


class AvoidanceSettingsDialog(QDialog):
    """Configure one UVN obstacle volume for every resolved planning patch."""

    def __init__(
        self,
        project_path: Path,
        *,
        settings_path: Path | None = None,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle("避障设置")
        self.resize(1120, 820)
        self.project_path = Path(project_path)
        self.settings_path = settings_path or backend.avoidance_settings_path_for(self.project_path)
        self.project = backend.load_project_file(self.project_path)
        self.regions = [set(region) for region in self.project.selected_path_face_regions]
        if not self.regions:
            raise ValueError("当前项目没有已选择的打磨面")
        self.planning_regions = backend.manual_clip_regions(self.project_path, self.regions)
        self.polydata = self._load_polydata()
        self._entries: list[dict] = []
        self._current_index = -1
        self._existing_payload = self._read_existing_payload()

        self.preview = RegionPreview()
        self.preview.setMinimumHeight(500)
        self.preview.load_project(self.project_path)

        self.selector_edit = QLineEdit(",".join(self._existing_payload.get("selectors", [])))
        self.selector_edit.setPlaceholderText("例如 1-2,12")
        self.resolve_button = QPushButton("解析区域")
        self.current_label = QLabel("当前区域：-")
        self.page_label = QLabel("0/0")
        self.previous_button = QPushButton("上一个")
        self.next_button = QPushButton("下一个")

        self.u_expand = self._percent_spin(backend.DEFAULT_U_EXPAND_PERCENT)
        self.v_expand = self._percent_spin(backend.DEFAULT_V_EXPAND_PERCENT)
        self.n_plus = self._distance_spin()
        self.n_minus = self._distance_spin()

        selector_row = QHBoxLayout()
        selector_row.addWidget(QLabel("避障区域"))
        selector_row.addWidget(self.selector_edit, 1)
        selector_row.addWidget(self.resolve_button)

        navigation_row = QHBoxLayout()
        navigation_row.addWidget(self.current_label)
        navigation_row.addWidget(self.page_label)
        navigation_row.addStretch(1)
        navigation_row.addWidget(self.previous_button)
        navigation_row.addWidget(self.next_button)

        parameter_form = QFormLayout()
        parameter_form.addRow("U 扩大比例", self.u_expand)
        parameter_form.addRow("V 扩大比例", self.v_expand)
        parameter_form.addRow("N+ 高度", self.n_plus)
        parameter_form.addRow("N- 高度", self.n_minus)

        self.hint = QLabel(
            "黄色=所选打磨面，绿色=支撑面，红色=范围内墙体，灰色透明体=UVN覆盖范围。"
            "U/V 的 30% 表示最终总宽度扩大到原来的 130%。"
        )
        self.hint.setWordWrap(True)
        self.status = QLabel("请输入区域并解析")
        self.status.setWordWrap(True)
        self.preview_button = QPushButton("预览")
        self.apply_button = QPushButton("应用")
        self.cancel_button = QPushButton("取消")

        action_row = QHBoxLayout()
        action_row.addWidget(self.status, 1)
        action_row.addWidget(self.preview_button)
        action_row.addWidget(self.apply_button)
        action_row.addWidget(self.cancel_button)

        layout = QVBoxLayout(self)
        layout.addWidget(self.preview, 1)
        layout.addLayout(selector_row)
        layout.addLayout(navigation_row)
        layout.addLayout(parameter_form)
        layout.addWidget(self.hint)
        layout.addLayout(action_row)

        self._preview_timer = QTimer(self)
        self._preview_timer.setSingleShot(True)
        self._preview_timer.setInterval(180)
        self._preview_timer.timeout.connect(self._refresh_preview)
        self.resolve_button.clicked.connect(self._resolve_regions)
        self.previous_button.clicked.connect(lambda: self._show_entry(self._current_index - 1))
        self.next_button.clicked.connect(lambda: self._show_entry(self._current_index + 1))
        self.preview_button.clicked.connect(self._refresh_preview)
        self.apply_button.clicked.connect(self._apply)
        self.cancel_button.clicked.connect(self.reject)
        for spin in (self.u_expand, self.v_expand, self.n_plus, self.n_minus):
            spin.valueChanged.connect(lambda _value: self._preview_timer.start())

        self._set_parameter_enabled(False)
        if self.selector_edit.text().strip():
            QTimer.singleShot(0, self._resolve_regions)

    @property
    def resolved_labels(self) -> list[str]:
        return [str(entry["planning_region"]["label"]) for entry in self._entries]

    def _load_polydata(self):
        importer = backend.CadImportService().import_model(self.project.workpiece.file_path)
        reader = backend.create_mesh_reader(importer.display_path, importer.display_format)
        reader.SetFileName(str(importer.display_path))
        reader.Update()
        return reader.GetOutput()

    def _read_existing_payload(self) -> dict:
        if not self.settings_path.exists():
            return {}
        try:
            payload = backend.load_avoidance_settings(self.settings_path)
        except Exception:
            return {}
        if Path(str(payload.get("input_project", ""))).resolve() != self.project_path.resolve():
            return {}
        return payload

    @staticmethod
    def _percent_spin(default: float) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(0.0, 500.0)
        spin.setDecimals(1)
        spin.setSingleStep(5.0)
        spin.setSuffix(" %")
        spin.setValue(default)
        return spin

    @staticmethod
    def _distance_spin() -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(0.0, 100_000.0)
        spin.setDecimals(1)
        spin.setSingleStep(25.0)
        spin.setSuffix(" mm")
        return spin

    def _set_parameter_enabled(self, enabled: bool) -> None:
        for widget in (
            self.u_expand,
            self.v_expand,
            self.n_plus,
            self.n_minus,
            self.previous_button,
            self.next_button,
            self.preview_button,
            self.apply_button,
        ):
            widget.setEnabled(enabled)

    def _resolve_regions(self) -> None:
        try:
            selectors = backend.parse_region_selectors(self.selector_edit.text())
            if not selectors:
                raise ValueError("请填写至少一个避障 region 或 patch")
            backend.validate_selectors(selectors, self.planning_regions)
            matched = [
                item
                for item in self.planning_regions
                if backend.selector_matches(set(selectors), str(item["label"]), int(item["source_region"]))
            ]
            existing_by_label = {
                str(record.get("region_label")): record
                for record in self._existing_payload.get("regions", [])
            }
            entries: list[dict] = []
            bounds_by_frame: dict[tuple[float, ...], object] = {}
            for planning_region in matched:
                label = str(planning_region["label"])
                seed_ids = self._path_seed_ids(planning_region)
                support = backend.grow_support_surface(self.polydata, seed_ids)
                existing = existing_by_label.get(label)
                frame = (
                    backend.AvoidanceVolumeFrame.from_dict(existing["frame"])
                    if existing and isinstance(existing.get("frame"), dict)
                    else backend.avoidance_volume_frame(
                        self.polydata,
                        support,
                        raster_chart=planning_region.get("raster_chart"),
                    )
                )
                frame_key = tuple(
                    round(float(value), 9)
                    for vector in (frame.origin, frame.u_axis, frame.v_axis, frame.n_axis)
                    for value in vector
                )
                if frame_key not in bounds_by_frame:
                    bounds_by_frame[frame_key] = backend.avoidance_cell_bounds_uvn(
                        self.polydata,
                        frame,
                    )
                default_n_plus, default_n_minus = backend.default_normal_heights_mm(
                    self.polydata,
                    support,
                    frame,
                )
                raw_settings = existing.get("settings", {}) if existing else {}
                settings = backend.AvoidanceVolumeSettings(
                    u_expand_percent=float(
                        raw_settings.get("u_expand_percent", backend.DEFAULT_U_EXPAND_PERCENT)
                    ),
                    v_expand_percent=float(
                        raw_settings.get("v_expand_percent", backend.DEFAULT_V_EXPAND_PERCENT)
                    ),
                    n_plus_mm=float(raw_settings.get("n_plus_mm", default_n_plus)),
                    n_minus_mm=float(raw_settings.get("n_minus_mm", default_n_minus)),
                )
                entries.append(
                    {
                        "planning_region": planning_region,
                        "support": support,
                        "frame": frame,
                        "settings": settings,
                        "volume": None,
                        "cell_bounds_uvn": bounds_by_frame[frame_key],
                    }
                )
            self._entries = entries
            self._current_index = -1
            self._set_parameter_enabled(True)
            self._show_entry(0)
            self.status.setText(
                f"已解析 {len(entries)} 个规划区域："
                + ",".join(str(item["planning_region"]["label"]).replace("_", "-") for item in entries)
            )
        except Exception as exc:
            self._entries = []
            self._current_index = -1
            self._set_parameter_enabled(False)
            self.status.setText(f"区域解析失败：{exc}")
            QMessageBox.warning(self, "避障区域解析失败", str(exc))

    def _path_seed_ids(self, planning_region: dict) -> set[int]:
        settings = backend.RasterPlannerSettings(
            spacing=backend.SPACING,
            point_step=backend.POINT_STEP,
            angle_degrees=0.0,
            boundary_margin=backend.BOUNDARY_MARGIN,
            bidirectional=True,
            feed_direction=backend.RasterFeedDirection.LONG_SIDE,
            start_corner=backend.StartCorner.LOWER_LEFT,
            tool_axis="-z",
            speed=100.0,
            zone="z1",
            tool_name=self.project.polishing_tool.name,
        )
        result, _cell_transfer, _reason = backend.plan_region_uv_auto(
            self.polydata,
            self.project.workpiece,
            settings,
            planning_region["face_ids"],
            backend.RasterFeedDirection.LONG_SIDE,
            planning_region.get("clip_polygon"),
            planning_region.get("exclude_polygons"),
            planning_region.get("raster_chart"),
        )
        seed_ids = backend.path_seed_cell_ids(result)
        return seed_ids or set(planning_region["face_ids"])

    def _save_current_values(self) -> None:
        if not (0 <= self._current_index < len(self._entries)):
            return
        self._entries[self._current_index]["settings"] = backend.AvoidanceVolumeSettings(
            u_expand_percent=self.u_expand.value(),
            v_expand_percent=self.v_expand.value(),
            n_plus_mm=self.n_plus.value(),
            n_minus_mm=self.n_minus.value(),
        )

    def _show_entry(self, index: int) -> None:
        if not self._entries:
            return
        self._save_current_values()
        self._current_index = index % len(self._entries)
        entry = self._entries[self._current_index]
        settings = entry["settings"]
        for spin, value in (
            (self.u_expand, settings.u_expand_percent),
            (self.v_expand, settings.v_expand_percent),
            (self.n_plus, settings.n_plus_mm),
            (self.n_minus, settings.n_minus_mm),
        ):
            spin.blockSignals(True)
            spin.setValue(float(value))
            spin.blockSignals(False)
        label = str(entry["planning_region"]["label"]).replace("_", "-")
        self.current_label.setText(f"当前区域：{label}")
        self.page_label.setText(f"{self._current_index + 1}/{len(self._entries)}")
        self.previous_button.setEnabled(len(self._entries) > 1)
        self.next_button.setEnabled(len(self._entries) > 1)
        self._refresh_preview()

    def _refresh_preview(self) -> None:
        if not (0 <= self._current_index < len(self._entries)):
            return
        try:
            self._save_current_values()
            entry = self._entries[self._current_index]
            region = entry["planning_region"]
            volume = backend.build_avoidance_volume(
                self.polydata,
                entry["support"],
                entry["settings"],
                raster_chart=region.get("raster_chart"),
                frame=entry["frame"],
                cell_bounds_uvn=entry["cell_bounds_uvn"],
            )
            entry["volume"] = volume
            self.preview.show_avoidance_volume(
                support_cell_ids=set(entry["support"].support_cell_ids),
                obstacle_cell_ids=set(volume.obstacle_cell_ids),
                selected_face_ids=set(region["face_ids"]),
                volume_vertices=volume.vertices_model,
                clip_polygon=region.get("clip_polygon"),
                exclude_polygons=region.get("exclude_polygons"),
                raster_chart=region.get("raster_chart"),
                label=str(region["label"]).replace("_", "-"),
            )
            self.status.setText(
                f"区域 {str(region['label']).replace('_', '-')}："
                f"支撑面 {len(entry['support'].support_cell_ids)} cells，"
                f"范围内墙体 {len(volume.obstacle_cell_ids)} cells，"
                f"范围外 {volume.outside_cell_count} cells"
            )
        except Exception as exc:
            self.status.setText(f"预览失败：{exc}")

    def _apply(self) -> None:
        if not self._entries:
            QMessageBox.warning(self, "没有避障设置", "请先解析至少一个避障区域")
            return
        try:
            self._save_current_values()
            records: list[dict] = []
            for entry in self._entries:
                region = entry["planning_region"]
                volume = backend.build_avoidance_volume(
                    self.polydata,
                    entry["support"],
                    entry["settings"],
                    raster_chart=region.get("raster_chart"),
                    frame=entry["frame"],
                    cell_bounds_uvn=entry["cell_bounds_uvn"],
                )
                records.append(
                    {
                        "region_label": str(region["label"]),
                        "source_region": int(region["source_region"]),
                        "frame": entry["frame"].as_dict(),
                        "settings": asdict(entry["settings"]),
                        "preview": volume.as_dict(),
                    }
                )
            selectors = backend.parse_region_selectors(self.selector_edit.text())
            backend.write_avoidance_settings(
                self.settings_path,
                input_project=self.project_path,
                selectors=selectors,
                records=records,
            )
        except Exception as exc:
            QMessageBox.critical(self, "避障设置保存失败", str(exc))
            return
        self.accept()
