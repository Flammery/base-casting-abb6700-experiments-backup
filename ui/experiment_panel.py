from __future__ import annotations

import json
import sys
from pathlib import Path

from PySide6.QtCore import QProcess, Qt
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
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
if str(UI_DIR) not in sys.path:
    sys.path.insert(0, str(UI_DIR))

from experiment_config import (  # noqa: E402
    DEFAULT_INPUT,
    DEFAULT_PARTITIONED,
    DEFAULT_BOUNDARY_MARGIN,
    DEFAULT_X,
    DEFAULT_Y,
    DEFAULT_Z,
    ROOT,
    parse_region_text,
    read_region_count,
    runner_command,
    validate_regions,
)
from manual_partition_dialog import (  # noqa: E402
    PARTITION_MODE_BOUNDARY,
    PARTITION_MODE_PICK,
    PARTITION_MODE_SLAB,
    ManualPartitionDialog,
    manual_manifest_path_for,
)
from region_viewer import RegionPreview  # noqa: E402


class ExperimentPanel(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("ABB6700 polishing experiment")
        self.resize(1380, 620)
        self._process: QProcess | None = None
        self._runner_output = ""
        self._process_kind = ""
        self._current_region_count = 0

        self.input_path = QLineEdit(str(DEFAULT_INPUT))
        self.input_path.setMinimumWidth(360)
        self.input_button = QPushButton("选择输入")
        self.region_count = QLabel("regions: -")

        self.partition_regions = QLineEdit("")
        self.partition_regions.setPlaceholderText("留空=当前唯一region")
        self.partition_regions.setFixedWidth(120)
        self.partition_output = QLineEdit(str(DEFAULT_PARTITIONED))
        self.partition_output.setMinimumWidth(330)
        self.apply_partition_button = QPushButton("区域划分")

        self.window_limits = QLineEdit("1500,2500;-1050,1050")
        self.window_limits.setPlaceholderText("空=不限; 1000,2000;-1050,1050")
        self.window_limits.setMinimumWidth(230)
        self.boundary_margin = QLineEdit(DEFAULT_BOUNDARY_MARGIN)
        self.boundary_margin.setPlaceholderText("6")
        self.boundary_margin.setFixedWidth(58)
        self.start_button = QPushButton("开始")

        self.angle_preset = QComboBox()
        self.angle_preset.addItems(["地轨 0,180", "地轨 90,270", "转台 0..360 step10"])
        self.angle_preset.setMinimumWidth(150)

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
        first_row.addWidget(self.region_count)
        first_row.addWidget(QLabel("分区"))
        first_row.addWidget(self.partition_regions)
        first_row.addWidget(self.partition_output)
        first_row.addWidget(self.apply_partition_button)
        first_row.addWidget(self.start_button)

        second_row = QHBoxLayout()
        second_row.setContentsMargins(0, 0, 0, 0)
        second_row.addWidget(self.angle_preset)
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
        second_row.addStretch(1)

        layout.addLayout(first_row)
        layout.addLayout(second_row)
        layout.addWidget(self.preview, 1)
        layout.addWidget(self.status)

        self.input_button.clicked.connect(self.choose_input)
        self.apply_partition_button.clicked.connect(self.apply_partition)
        self.start_button.clicked.connect(self.start_run)
        self.input_path.editingFinished.connect(self.refresh_region_count)
        self.angle_preset.currentTextChanged.connect(self._sync_mode_defaults)
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

    def refresh_region_count(self) -> None:
        path = Path(self.input_path.text())
        if not path.exists():
            self._current_region_count = 0
            self.region_count.setText("regions: 文件不存在")
            self.status.setText(f"当前输入不存在: {path}")
            return
        try:
            count = read_region_count(path)
        except Exception as exc:
            self._current_region_count = 0
            self.region_count.setText("regions: 读取失败")
            self.status.setText(f"输入读取失败: {exc}")
            return
        self._current_region_count = count
        self.region_count.setText(f"regions: {count}")
        self.status.setText(f"当前输入: {path}")
        self.preview.load_project(path)

    def _sync_mode_defaults(self, mode: str) -> None:
        if mode == "转台 0..360 step10" and self.model_y.text().strip() in ("", DEFAULT_Y):
            self.model_y.setText("0")
        elif mode in ("地轨 0,180", "地轨 90,270") and self.model_y.text().strip() in ("", "0"):
            self.model_y.setText(DEFAULT_Y)

    def apply_partition(self) -> None:
        if self._process is not None:
            QMessageBox.information(self, "正在运行", "当前任务还没有结束。")
            return
        try:
            input_path = Path(self.input_path.text())
            output_path = Path(self.partition_output.text())
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
        if self._process is not None:
            QMessageBox.information(self, "正在运行", "当前任务还没有结束。")
            return
        try:
            input_path = Path(self.input_path.text())
            if not input_path.exists():
                raise ValueError(f"输入文件不存在: {input_path}")
            command = runner_command(
                sys.executable,
                input_path,
                self.model_x.text(),
                self.model_y.text(),
                self.model_z.text(),
                self.angle_preset.currentText(),
                self.window_limits.text(),
                self.boundary_margin.text(),
            )
        except Exception as exc:
            self.status.setText(f"实验参数错误: {exc}")
            QMessageBox.warning(self, "实验参数错误", str(exc))
            return

        self._start_process("Optimal-Y 实验", command, "run")

    def _start_process(self, label: str, command: list[str], kind: str) -> None:
        self.start_button.setEnabled(False)
        self.apply_partition_button.setEnabled(False)
        self._runner_output = ""
        self._process_kind = kind
        self.status.setText(f"{label} 运行中...")

        process = QProcess(self)
        process.setProgram(command[0])
        process.setArguments(command[1:])
        process.setWorkingDirectory(str(ROOT))
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
        self._process = None
        self.start_button.setEnabled(True)
        self.apply_partition_button.setEnabled(True)
        if exit_code != 0:
            tail = "\n".join(self._runner_output.splitlines()[-8:])
            self.status.setText(f"运行失败，exit={exit_code}: {tail}")
            QMessageBox.critical(self, "运行失败", self.status.text())
            return

        if self._process_kind == "partition":
            output_path = Path(self.partition_output.text())
            self.input_path.setText(str(output_path))
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
            self.status.setText(
                "完成: "
                f"output={summary.get('output_dir', summary_path.parent)} | "
                f"scan={summary.get('scan_axis', '-')} | "
                f"poses={summary.get('pose_count', '-')} | "
                f"regions={summary.get('selected_region_count', '-')} | "
                f"candidates={summary.get('candidate_count', '-')} | "
                f"optimal={summary.get('optimal_region_count', '-')}"
            )
            QMessageBox.information(self, "完成", self.status.text())
            return
        self.status.setText("完成: 未找到 summary.json，请查看脚本输出。")
        QMessageBox.information(self, "完成", self.status.text())

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
