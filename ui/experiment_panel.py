from __future__ import annotations

import json
import sys
from pathlib import Path

from PySide6.QtCore import QProcess, QProcessEnvironment, QTimer, Qt, QUrl
from PySide6.QtGui import QDesktopServices
from PySide6.QtWidgets import (
    QApplication,
    QDialog,
    QFileDialog,
    QFrame,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

UI_DIR = Path(__file__).resolve().parent
SCRIPT_DIR = UI_DIR.parent / "scripts"
if str(UI_DIR) not in sys.path:
    sys.path.insert(0, str(UI_DIR))
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from experiment_config import (  # noqa: E402
    DEFAULT_INPUT,
    DEFAULT_PARTITIONED,
    DEFAULT_BOUNDARY_MARGIN,
    DEFAULT_TURNTABLE_ANGLES,
    DEFAULT_X,
    DEFAULT_Y,
    DEFAULT_Z,
    ROOT,
    parse_region_text,
    parse_turntable_angle_text,
    read_region_count,
    runner_command,
    validated_avoidance_settings,
    validate_regions,
)
from manual_partition_dialog import (  # noqa: E402
    PARTITION_MODE_BOUNDARY,
    PARTITION_MODE_PICK,
    PARTITION_MODE_SLAB,
    ManualPartitionDialog,
    manual_manifest_path_for,
)
from avoidance_settings_dialog import AvoidanceSettingsDialog  # noqa: E402
from region_viewer import RegionPreview  # noqa: E402
import window_conf_export as path_preview_backend  # noqa: E402
from robotstudio_package import build_package, queue_manifest  # noqa: E402


class ExperimentPanel(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("ABB6700 polishing experiment")
        self.resize(1380, 620)
        self._process: QProcess | None = None
        self._runner_output = ""
        self._process_kind = ""
        self._current_region_count = 0
        self._last_result_dir: Path | None = None
        self._robotstudio_result_dir: Path | None = None
        self._robot_config_override = None
        self.robot_config_path: Path | None = None
        self.avoidance_settings_path: Path | None = None
        self.avoidance_selectors: list[str] = []
        self._avoidance_armed = False
        self._avoidance_applied_project: Path | None = None
        self._avoidance_settings_error: str | None = None

        self.input_path = QLineEdit(str(DEFAULT_INPUT))
        self.input_path.setMinimumWidth(360)
        self.input_button = QPushButton("选择输入")
        self.region_count = QLabel("regions: -")
        self.robot_config_button = QPushButton("导入杆系配置")
        self.robot_config_label = QLabel("杆系: 项目内配置")
        self.robot_config_label.setToolTip("未导入独立 .rsc.json；避障使用输入项目中的杆系配置和实验细杆模型。")

        self.partition_regions = QLineEdit("")
        self.partition_regions.setPlaceholderText("留空=当前唯一region")
        self.partition_regions.setFixedWidth(120)
        # Partition output remains a fixed experiment path; it is intentionally
        # not exposed as the long editable path that previously occupied row 1.
        self.partition_output_path = DEFAULT_PARTITIONED
        self.apply_partition_button = QPushButton("区域划分")
        self.avoidance_settings_button = QPushButton("避障设置")
        self.avoidance_settings_label = QLabel("未配置")
        self.avoidance_settings_label.setToolTip("设置避障 region/patch 和 UVN 墙体覆盖范围")

        self.window_limits = QLineEdit("1500,2500;-1050,1050")
        self.window_limits.setPlaceholderText("空=不限; 1000,2000;-1050,1050")
        self.window_limits.setMinimumWidth(230)
        self.boundary_margin = QLineEdit(DEFAULT_BOUNDARY_MARGIN)
        self.boundary_margin.setPlaceholderText("6")
        self.boundary_margin.setFixedWidth(58)
        self.start_button = QPushButton("开始")
        self.preview_path_button = QPushButton("快速预览路径")
        self.robotstudio_button = QPushButton("导入 RobotStudio 验证")
        self.robotstudio_timer = QTimer(self)
        self.robotstudio_timer.setInterval(1000)

        self.turntable_angles = QLineEdit(DEFAULT_TURNTABLE_ANGLES)
        self.turntable_angles.setPlaceholderText("270、0,180 或 0-30-330")
        self.turntable_angles.setFixedWidth(190)

        self.model_x = self._coord_edit(DEFAULT_X)
        self.model_y = self._coord_edit(DEFAULT_Y)
        self.model_z = self._coord_edit(DEFAULT_Z)

        self.status = QLabel("准备就绪")
        self.status.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.status.setFrameShape(QFrame.Shape.StyledPanel)
        self.status.setMinimumHeight(36)
        self.preview = RegionPreview()

        layout = QVBoxLayout(self)
        first_row = QHBoxLayout()
        first_row.setContentsMargins(0, 0, 0, 0)
        first_row.addWidget(self.input_button)
        first_row.addWidget(self.input_path)
        first_row.addWidget(self.robot_config_button)
        first_row.addWidget(self.robot_config_label)
        first_row.addWidget(self.region_count)
        first_row.addWidget(QLabel("分区"))
        first_row.addWidget(self.partition_regions)
        first_row.addWidget(self.apply_partition_button)
        first_row.addWidget(self.avoidance_settings_button)
        first_row.addWidget(self.avoidance_settings_label)
        first_row.addStretch(1)

        second_row = QHBoxLayout()
        second_row.setContentsMargins(0, 0, 0, 0)
        second_row.addWidget(QLabel("转台"))
        second_row.addWidget(self.turntable_angles)
        second_row.addWidget(QLabel("X"))
        second_row.addWidget(self.model_x)
        second_row.addWidget(QLabel("Y"))
        second_row.addWidget(self.model_y)
        second_row.addWidget(QLabel("Z"))
        second_row.addWidget(self.model_z)
        second_row.addWidget(QLabel("加工范围"))
        second_row.addWidget(self.window_limits)
        second_row.addWidget(QLabel("边缘余量"))
        second_row.addWidget(self.boundary_margin)
        second_row.addWidget(self.preview_path_button)
        second_row.addWidget(self.start_button)
        second_row.addStretch(1)

        layout.addLayout(first_row)
        layout.addLayout(second_row)
        third_row = QHBoxLayout()
        third_row.setContentsMargins(0, 0, 0, 0)
        third_row.addWidget(self.robotstudio_button)
        third_row.addWidget(QLabel("每个最优面生成工作站；场景安装位置与 RAPID 工件坐标相互独立，按面顺序验证"))
        third_row.addWidget(QLabel("快速预览仅显示几何；正式实验执行避障 IK/FK，最终仍需 ABB/RobotStudio 验证"))
        third_row.addStretch(1)
        layout.addLayout(third_row)
        layout.addWidget(self.preview, 1)
        layout.addWidget(self.status)

        self.input_button.clicked.connect(self.choose_input)
        self.robot_config_button.clicked.connect(self.choose_robot_config)
        self.apply_partition_button.clicked.connect(self.apply_partition)
        self.avoidance_settings_button.clicked.connect(self.configure_avoidance)
        self.preview_path_button.clicked.connect(self.preview_paths)
        self.start_button.clicked.connect(self.start_run)
        self.robotstudio_button.clicked.connect(self.export_to_robotstudio)
        self.robotstudio_timer.timeout.connect(self._poll_robotstudio_status)
        self.input_path.editingFinished.connect(self.refresh_region_count)
        self.refresh_region_count()

    def _coord_edit(self, text: str) -> QLineEdit:
        edit = QLineEdit(text)
        edit.setMinimumWidth(120)
        edit.textChanged.connect(lambda value, widget=edit: self._fit_line_edit(widget, value))
        self._fit_line_edit(edit, text)
        return edit

    def _fit_line_edit(self, widget: QLineEdit, text: str) -> None:
        width = widget.fontMetrics().horizontalAdvance(text or widget.placeholderText() or "3700") + 34
        widget.setMinimumWidth(max(105, min(width, 260)))

    def choose_input(self) -> None:
        path, _filter = QFileDialog.getOpenFileName(
            self,
            "选择输入 .rsp.json",
            str(Path(self.input_path.text()).parent),
            "Robot Studio project (*.rsp.json);;JSON (*.json);;All files (*)",
        )
        if path:
            self.input_path.setText(path)
            self.refresh_region_count()

    def choose_robot_config(self) -> None:
        initial_dir = (
            self.robot_config_path.parent
            if self.robot_config_path is not None
            else ROOT / "src"
        )
        path, _filter = QFileDialog.getOpenFileName(
            self,
            "导入杆系配置",
            str(initial_dir),
            "Robot Studio Configuration (*.rsc.json);;JSON (*.json);;All files (*)",
        )
        if not path:
            return
        try:
            override = path_preview_backend.load_robot_config_override(path)
        except Exception as exc:
            self.status.setText(f"杆系配置导入失败: {exc}")
            QMessageBox.warning(self, "杆系配置导入失败", str(exc))
            return

        self._robot_config_override = override
        self.robot_config_path = override.path
        envelope_text = ", ".join(
            f"J{row['joint']}={row['collision_radius_mm']:g}mm"
            for row in override.envelope_rows()
        )
        self.robot_config_label.setText(f"杆系: {override.name}")
        self.robot_config_label.setToolTip(f"{override.path}\n碰撞半径: {envelope_text}")
        self.status.setText(f"已导入杆系配置: {override.name} | {override.path} | {envelope_text}")

    def refresh_region_count(self) -> None:
        path = Path(self.input_path.text())
        if not path.exists():
            self._current_region_count = 0
            self.region_count.setText("regions: 文件不存在")
            self.status.setText(f"当前输入不存在: {path}")
            self._clear_avoidance_settings()
            return
        try:
            count = read_region_count(path)
        except Exception as exc:
            self._current_region_count = 0
            self.region_count.setText("regions: 读取失败")
            self.status.setText(f"输入读取失败: {exc}")
            self._clear_avoidance_settings()
            return
        self._current_region_count = count
        self.region_count.setText(f"regions: {count}")
        self.status.setText(f"当前输入: {path}")
        self.preview.load_project(path)
        active_for_current_project = (
            self._avoidance_armed
            and self._avoidance_applied_project is not None
            and self._avoidance_applied_project == path.resolve()
        )
        if not active_for_current_project:
            self._refresh_avoidance_settings(path, activate=False)

    def _clear_avoidance_settings(self) -> None:
        self.avoidance_settings_path = None
        self.avoidance_selectors = []
        self._avoidance_armed = False
        self._avoidance_applied_project = None
        self._avoidance_settings_error = None
        self.avoidance_settings_label.setText("未配置（避障关闭）")
        self.avoidance_settings_label.setToolTip("设置避障 region/patch 和 UVN 墙体覆盖范围")

    def _refresh_avoidance_settings(self, project_path: Path, *, activate: bool) -> bool:
        """Inspect saved settings, and activate them only after explicit Apply."""

        settings_path = path_preview_backend.avoidance_settings_path_for(project_path)
        self._clear_avoidance_settings()
        if not settings_path.exists():
            return False
        try:
            payload = path_preview_backend.load_avoidance_settings(settings_path)
        except Exception as exc:
            self._avoidance_settings_error = str(exc)
            self.avoidance_settings_label.setText("历史配置无效（本次未启用）")
            self.avoidance_settings_label.setToolTip(f"{settings_path}\n{exc}")
            return False

        labels = [str(record.get("region_label", "")).replace("_", "-") for record in payload.get("regions", [])]
        if not activate:
            count_text = f" {len(labels)} 个区域" if labels else ""
            self.avoidance_settings_label.setText(f"有已保存配置{count_text}（本次未启用）")
            self.avoidance_settings_label.setToolTip(
                f"{settings_path}\n只有在避障设置窗口点击“应用”后，下一次运行才会启用"
            )
            return False

        try:
            project = path_preview_backend.load_project_file(project_path)
            regions = [set(region) for region in project.selected_path_face_regions]
            planning_regions = path_preview_backend.manual_clip_regions(project_path, regions)
            selectors, labels = validated_avoidance_settings(payload, project_path, planning_regions)
        except Exception as exc:
            self._avoidance_settings_error = str(exc)
            self.avoidance_settings_label.setText("配置无效（本次未启用）")
            self.avoidance_settings_label.setToolTip(f"{settings_path}\n{exc}")
            return False

        self.avoidance_settings_path = settings_path
        self.avoidance_selectors = selectors
        self._avoidance_armed = True
        self._avoidance_applied_project = project_path.resolve()
        self._avoidance_settings_error = None
        self.avoidance_settings_label.setText(
            f"已应用：下一次运行启用避障（{len(labels)} 个区域："
            + ",".join(labels[:4])
            + ("..." if len(labels) > 4 else "")
            + "）"
        )
        self.avoidance_settings_label.setToolTip(f"{settings_path}\n本次设置将在下一次 runner 结束后自动解除")
        return True

    def configure_avoidance(self) -> None:
        if self._process is not None:
            QMessageBox.information(self, "正在运行", "请等待当前实验结束后再修改避障设置。")
            return
        project_path = Path(self.input_path.text())
        if not project_path.exists():
            QMessageBox.warning(self, "避障设置", f"当前输入不存在: {project_path}")
            return
        try:
            dialog = AvoidanceSettingsDialog(project_path, parent=self)
        except Exception as exc:
            QMessageBox.critical(self, "避障设置加载失败", str(exc))
            return
        if dialog.exec() != QDialog.DialogCode.Accepted:
            return
        if not self._refresh_avoidance_settings(project_path, activate=True):
            message = self._avoidance_settings_error or "避障设置未能启用"
            self.status.setText(f"避障设置应用失败: {message}")
            QMessageBox.warning(self, "避障设置应用失败", message)
            return
        self.status.setText(
            f"避障设置已应用: {self.avoidance_settings_label.text()} | {self.avoidance_settings_path}"
        )

    @staticmethod
    def _first_coordinate(text: str, fallback: str) -> float:
        return float((text.strip() or fallback).split(",", 1)[0].strip())

    def _preview_angle(self) -> int:
        return parse_turntable_angle_text(self.turntable_angles.text())[0]

    def preview_paths(self) -> None:
        """Use the production planner once and draw its model-coordinate result."""
        if self._process is not None:
            QMessageBox.information(self, "正在运行", "当前实验尚未结束。")
            return
        try:
            project_path = Path(self.input_path.text())
            self.preview.load_project(project_path)
            project = path_preview_backend.load_project_file(project_path)
            if self._robot_config_override is not None:
                self._robot_config_override.apply_to_project(project)
            regions = [set(region) for region in project.selected_path_face_regions]
            if not regions:
                raise ValueError("当前项目没有已选择的加工面。")
            planning_regions = path_preview_backend.manual_clip_regions(project_path, regions)
            avoidance_selectors = list(self.avoidance_selectors)
            path_preview_backend.validate_selectors(avoidance_selectors, planning_regions)
            avoidance_selector_set = set(avoidance_selectors)
            importer = path_preview_backend.CadImportService().import_model(project.workpiece.file_path)
            reader = path_preview_backend.create_mesh_reader(importer.display_path, importer.display_format)
            reader.SetFileName(str(importer.display_path))
            reader.Update()
            polydata = reader.GetOutput()
            placement = path_preview_backend.placement_for(
                project.workpiece,
                project.workpiece.picked_origin,
                self._first_coordinate(self.model_x.text(), DEFAULT_X),
                self._first_coordinate(self.model_y.text(), DEFAULT_Y),
                self._first_coordinate(self.model_z.text(), DEFAULT_Z),
                float(self._preview_angle()),
            )
            settings = path_preview_backend.RasterPlannerSettings(
                spacing=path_preview_backend.SPACING,
                point_step=path_preview_backend.POINT_STEP,
                angle_degrees=0.0,
                boundary_margin=float(self.boundary_margin.text().strip() or DEFAULT_BOUNDARY_MARGIN),
                bidirectional=True,
                feed_direction=path_preview_backend.RasterFeedDirection.LONG_SIDE,
                start_corner=path_preview_backend.StartCorner.LOWER_LEFT,
                tool_axis="-z",
                speed=100.0,
                zone="z1",
                tool_name=project.polishing_tool.name,
            )
            paths = []
            cell_transfer_count = 0
            planner_reasons: dict[str, int] = {}
            avoidance_statuses: dict[str, int] = {}
            for planning_region in planning_regions:
                result, use_cell_transfer, planner_reason = path_preview_backend.plan_region_uv_auto(
                    polydata,
                    placement,
                    settings,
                    planning_region["face_ids"],
                    path_preview_backend.RasterFeedDirection.LONG_SIDE,
                    planning_region.get("clip_polygon"),
                    planning_region.get("exclude_polygons"),
                    planning_region.get("raster_chart"),
                )
                avoidance_selected = path_preview_backend.selector_matches(
                    avoidance_selector_set,
                    planning_region["label"],
                    planning_region["source_region"],
                ) if avoidance_selector_set else False
                if result.waypoints and avoidance_selected:
                    status = "待正式运行IK/FK"
                    avoidance_statuses[status] = avoidance_statuses.get(status, 0) + 1
                if result.waypoints:
                    paths.append(result)
                    cell_transfer_count += int(use_cell_transfer)
                    planner_reasons[planner_reason] = planner_reasons.get(planner_reason, 0) + 1
            if not paths:
                raise RuntimeError("当前设置没有生成可预览路径。")
            self.preview.show_paths(paths)
            total = sum(len(path.waypoints) for path in paths)
            self.status.setText(
                f"快速预览完成: angle={self._preview_angle()}° | regions={len(paths)} | points={total} | "
                f"cell抬刀={cell_transfer_count} | 判定={planner_reasons} | 避障={avoidance_statuses or '未启用'} | "
                "快速预览仅显示几何，正式实验执行避障IK/FK"
            )
        except Exception as exc:
            self.status.setText(f"快速预览失败: {exc}")
            QMessageBox.warning(self, "快速预览失败", str(exc))

    def apply_partition(self) -> None:
        if self._process is not None:
            QMessageBox.information(self, "正在运行", "当前任务还没有结束。")
            return
        try:
            input_path = Path(self.input_path.text())
            output_path = Path(self.partition_output_path)
            if not input_path.exists():
                raise ValueError(f"输入文件不存在: {input_path}")
            raw_regions = self.partition_regions.text().strip()
            regions = {1} if not raw_regions and self._current_region_count == 1 else set(parse_region_text(raw_regions))
            if not regions:
                raise ValueError("当前有多个 region，请填写需要手动划分的 region，例如 6。")
            validate_regions(sorted(regions), self._current_region_count)
        except Exception as exc:
            self.status.setText(f"分区参数错误: {exc}")
            QMessageBox.warning(self, "分区参数错误", str(exc))
            return

        partition_mode = self._choose_partition_mode()
        if partition_mode is None:
            self.status.setText("已取消区域划分。")
            return

        try:
            dialog = ManualPartitionDialog(input_path, output_path, regions, self, partition_mode=partition_mode)
        except Exception as exc:
            self.status.setText(f"区域划分加载失败: {exc}")
            QMessageBox.critical(self, "区域划分加载失败", str(exc))
            return
        if dialog.exec() != QDialog.DialogCode.Accepted:
            self.status.setText("已取消区域划分。")
            return

        self.input_path.setText(str(output_path))
        self._clear_avoidance_settings()
        self.refresh_region_count()
        self.preview.load_project(output_path)
        manifest_path = manual_manifest_path_for(output_path)
        self.status.setText(f"已完成手动区域划分: {output_path} | manifest={manifest_path}")
        QMessageBox.information(self, "完成", self.status.text())

    def _choose_partition_mode(self) -> str | None:
        box = QMessageBox(self)
        box.setWindowTitle("选择区域划分方式")
        box.setText("请选择这次手动 UV 区域划分方式。")
        box.setInformativeText("面边界式和贯穿式使用拉线；圈选区域式只保留框选/多边形圈出的加工区。")
        boundary_button = box.addButton("面边界式", QMessageBox.ButtonRole.AcceptRole)
        slab_button = box.addButton("贯穿式", QMessageBox.ButtonRole.ActionRole)
        pick_button = box.addButton("圈选区域式", QMessageBox.ButtonRole.ActionRole)
        box.addButton(QMessageBox.StandardButton.Cancel)
        box.exec()
        clicked = box.clickedButton()
        if clicked == boundary_button:
            return PARTITION_MODE_BOUNDARY
        if clicked == slab_button:
            return PARTITION_MODE_SLAB
        if clicked == pick_button:
            return PARTITION_MODE_PICK
        return None

    def start_run(self) -> None:
        self._start_run_with_planner("auto", "自动光栅/cell抬刀策略")

    def _start_run_with_planner(self, planner: str, label: str) -> None:
        if self._process is not None:
            QMessageBox.information(self, "正在运行", "当前任务还没有结束。")
            return
        try:
            input_path = Path(self.input_path.text())
            if not input_path.exists():
                raise ValueError(f"输入文件不存在: {input_path}")
            if self._avoidance_armed and not self._refresh_avoidance_settings(input_path, activate=True):
                raise ValueError(
                    f"已应用的避障配置与当前分区不匹配: {self._avoidance_settings_error}；"
                    "请重新打开“避障设置”并应用当前区域"
                )
            avoidance_selectors = ",".join(self.avoidance_selectors) if self._avoidance_armed else ""
            avoidance_settings_path = self.avoidance_settings_path if self._avoidance_armed else None
            command = runner_command(
                sys.executable,
                input_path,
                self.model_x.text(),
                self.model_y.text(),
                self.model_z.text(),
                self.turntable_angles.text(),
                self.window_limits.text(),
                self.boundary_margin.text(),
                planner,
                avoidance_selectors,
                self.robot_config_path,
                avoidance_settings_path,
            )
        except Exception as exc:
            self.status.setText(f"实验参数错误: {exc}")
            QMessageBox.warning(self, "实验参数错误", str(exc))
            return

        self._start_process(label, command, "run")

    def _start_process(self, label: str, command: list[str], kind: str) -> None:
        self.start_button.setEnabled(False)
        self.apply_partition_button.setEnabled(False)
        self.preview_path_button.setEnabled(False)
        self.robotstudio_button.setEnabled(False)
        self.robot_config_button.setEnabled(False)
        self.avoidance_settings_button.setEnabled(False)
        self.turntable_angles.setEnabled(False)
        self._runner_output = ""
        self._process_kind = kind
        self.status.setText(f"{label} 运行中...")

        process = QProcess(self)
        process.setProgram(command[0])
        process.setArguments(command[1:])
        process.setWorkingDirectory(str(ROOT))
        environment = QProcessEnvironment.systemEnvironment()
        environment.insert("PYTHONUTF8", "1")
        process.setProcessEnvironment(environment)
        process.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        process.readyReadStandardOutput.connect(self._collect_output)
        process.finished.connect(self._process_finished)
        self._process = process
        process.start()

    def _collect_output(self) -> None:
        if self._process is None:
            return
        text = bytes(self._process.readAllStandardOutput()).decode("utf-8", errors="replace")
        self._runner_output += text
        last_line = next((line for line in reversed(text.splitlines()) if line.strip()), "")
        if last_line:
            self.status.setText(last_line)

    def _process_finished(self, exit_code: int, _status) -> None:
        process_kind = self._process_kind
        self._process = None
        self.start_button.setEnabled(True)
        self.apply_partition_button.setEnabled(True)
        self.preview_path_button.setEnabled(True)
        self.robotstudio_button.setEnabled(True)
        self.robot_config_button.setEnabled(True)
        self.avoidance_settings_button.setEnabled(True)
        self.turntable_angles.setEnabled(True)
        if process_kind == "run":
            self._refresh_avoidance_settings(Path(self.input_path.text()), activate=False)
        if exit_code != 0:
            tail = "\n".join(self._runner_output.splitlines()[-8:])
            self.status.setText(f"运行失败，exit={exit_code}: {tail}")
            QMessageBox.critical(self, "运行失败", self.status.text())
            return

        if self._process_kind == "partition":
            output_path = Path(self.partition_output_path)
            self.input_path.setText(str(output_path))
            self._clear_avoidance_settings()
            self.refresh_region_count()
            self.preview.load_project(output_path)
            self.status.setText(f"已另存并应用: {output_path}")
            QMessageBox.information(self, "完成", self.status.text())
            return

        self._finish_run_success()

    def _finish_run_success(self) -> None:
        summary_path = self._summary_path_from_output()
        if summary_path and summary_path.exists():
            summary = json.loads(summary_path.read_text(encoding="utf-8"))
            reported_result_dir = Path(summary.get("output_dir", summary_path.parent))
            self._last_result_dir = reported_result_dir if reported_result_dir.is_absolute() else summary_path.parent
            self.status.setText(
                "完成: "
                f"output={summary.get('output_dir', summary_path.parent)} | "
                f"scan={summary.get('scan_axis', '-')} | "
                f"planner={summary.get('planner', 'legacy')} | "
                f"poses={summary.get('pose_count', '-')} | "
                f"regions={summary.get('selected_region_count', '-')} | "
                f"candidates={summary.get('candidate_count', '-')} | "
                f"optimal={summary.get('optimal_region_count', '-')}"
            )
            if not self._open_result_directory(self._last_result_dir):
                self.status.setText(f"{self.status.text()} | 结果文件夹未能自动打开")
            QMessageBox.information(self, "完成", self.status.text())
            return
        self.status.setText("完成: 未找到 summary.json，请查看脚本输出。")
        QMessageBox.information(self, "完成", self.status.text())

    @staticmethod
    def _open_result_directory(result_dir: Path) -> bool:
        if not result_dir.is_dir():
            return False
        return QDesktopServices.openUrl(QUrl.fromLocalFile(str(result_dir.resolve())))

    def export_to_robotstudio(self) -> None:
        if self._process is not None:
            QMessageBox.information(self, "正在运行", "请等待当前实验完成后再生成 RobotStudio 工作站。")
            return
        default_dir = self._last_result_dir or (Path(__file__).resolve().parents[1] / "results")
        selected = QFileDialog.getExistingDirectory(self, "选择 Optimal-Y 实验结果文件夹", str(default_dir))
        if not selected:
            return
        result_dir = Path(selected)
        try:
            manifest_path = build_package(result_dir)
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            queue_manifest(manifest_path, launch=True)
        except Exception as exc:
            self.status.setText(f"RobotStudio 导出准备失败: {exc}")
            QMessageBox.critical(self, "RobotStudio 导出准备失败", str(exc))
            return

        self._robotstudio_result_dir = result_dir
        self.robotstudio_button.setEnabled(False)
        self.robotstudio_timer.start()
        self.status.setText(
            f"已提交 RobotStudio: jobs={len(manifest.get('jobs', []))} | {manifest_path}"
        )

    def _poll_robotstudio_status(self) -> None:
        if self._robotstudio_result_dir is None:
            self.robotstudio_timer.stop()
            return
        status_path = self._robotstudio_result_dir / "robotstudio_status.json"
        if not status_path.exists():
            return
        try:
            payload = json.loads(status_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return
        state = payload.get("state", "unknown")
        self.status.setText(
            f"RobotStudio: {state} | {payload.get('completed', 0)}/{payload.get('total', 0)} | "
            f"region={payload.get('current_region', '-')} | {payload.get('message', '')}"
        )
        if state not in {"completed", "failed"}:
            return
        self.robotstudio_timer.stop()
        self.robotstudio_button.setEnabled(True)
        if state == "completed":
            QMessageBox.information(self, "RobotStudio 工作站已生成", self.status.text())
        else:
            QMessageBox.critical(self, "RobotStudio 工作站生成失败", self.status.text())

    def _summary_path_from_output(self) -> Path | None:
        for line in reversed(self._runner_output.splitlines()):
            if line.startswith("SUMMARY_JSON="):
                return Path(line.split("=", 1)[1].strip())
        return None


def main() -> int:
    app = QApplication(sys.argv)
    panel = ExperimentPanel()
    panel.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
