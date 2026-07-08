from __future__ import annotations

from copy import deepcopy
import json
import sys
from pathlib import Path

from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QBrush, QColor, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import (
    QDialog,
    QGraphicsLineItem,
    QGraphicsPolygonItem,
    QGraphicsScene,
    QGraphicsView,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
)

UI_DIR = Path(__file__).resolve().parent
EXPERIMENT_DIR = UI_DIR.parents[0]
ROOT = EXPERIMENT_DIR.parents[1]
SRC = ROOT / "src"
SCRIPT_DIR = EXPERIMENT_DIR / "scripts"
for path in (SRC, SCRIPT_DIR):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from robot_studio_qt.cad.import_service import CadImportService  # noqa: E402
from robot_studio_qt.cad.mesh_io import create_mesh_reader  # noqa: E402
from robot_studio_qt.path_planning.mesh_raster import read_triangles  # noqa: E402
from robot_studio_qt.project import load_project_file, save_project_file  # noqa: E402

from manual_region_partitioning import BarrierLine, manual_clip_manifest_records  # noqa: E402
from region_partitioning import face_geometries_from_triangles  # noqa: E402


def manual_manifest_path_for(output_path: Path) -> Path:
    name = output_path.name
    stem = name[: -len(".rsp.json")] if name.endswith(".rsp.json") else output_path.stem
    return output_path.with_name(f"{stem}_manifest.json")


def scene_point_from_model_xy(point: tuple[float, float]) -> QPointF:
    return QPointF(point[0], -point[1])


def model_xy_from_scene_point(point: QPointF) -> tuple[float, float]:
    return float(point.x()), float(-point.y())


class BarrierView(QGraphicsView):
    barrier_drawn = Signal(tuple)

    def __init__(self, scene: QGraphicsScene) -> None:
        super().__init__(scene)
        self._draw_mode = False
        self._start: QPointF | None = None
        self._rubber_line: QGraphicsLineItem | None = None
        self.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        self.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)

    def set_draw_mode(self, enabled: bool) -> None:
        self._draw_mode = enabled
        self.setDragMode(QGraphicsView.DragMode.NoDrag if enabled else QGraphicsView.DragMode.ScrollHandDrag)
        self.setCursor(Qt.CursorShape.CrossCursor if enabled else Qt.CursorShape.ArrowCursor)

    def wheelEvent(self, event) -> None:  # noqa: N802
        factor = 1.15 if event.angleDelta().y() > 0 else 1 / 1.15
        self.scale(factor, factor)

    def mousePressEvent(self, event) -> None:  # noqa: N802
        if not self._draw_mode or event.button() != Qt.MouseButton.LeftButton:
            super().mousePressEvent(event)
            return
        self._start = self.mapToScene(event.position().toPoint())
        self._rubber_line = self.scene().addLine(self._start.x(), self._start.y(), self._start.x(), self._start.y(), QPen(QColor(255, 230, 0), 5))

    def mouseMoveEvent(self, event) -> None:  # noqa: N802
        if self._draw_mode and self._start is not None and self._rubber_line is not None:
            point = self.mapToScene(event.position().toPoint())
            self._rubber_line.setLine(self._start.x(), self._start.y(), point.x(), point.y())
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event) -> None:  # noqa: N802
        if not self._draw_mode or self._start is None or self._rubber_line is None:
            super().mouseReleaseEvent(event)
            return
        end = self.mapToScene(event.position().toPoint())
        start = self._start
        self._rubber_line.setLine(start.x(), start.y(), end.x(), end.y())
        self._rubber_line = None
        self._start = None
        if (start - end).manhattanLength() > 1.0:
            self.barrier_drawn.emit((model_xy_from_scene_point(start), model_xy_from_scene_point(end)))


class ManualPartitionDialog(QDialog):
    def __init__(self, input_path: Path, output_path: Path, selected_regions: set[int], parent=None) -> None:
        super().__init__(parent)
        self.setWindowTitle("手动区域划分")
        self.resize(1040, 760)
        self.input_path = input_path
        self.output_path = output_path
        self.selected_regions = set(selected_regions)
        self.barriers: list[BarrierLine] = []
        self._line_items: list[QGraphicsLineItem] = []

        self.scene = QGraphicsScene(self)
        self.scene.setBackgroundBrush(QBrush(QColor(8, 10, 12)))
        self.view = BarrierView(self.scene)
        self.view.barrier_drawn.connect(self._add_barrier)

        self.hint = QLabel("拉线模式关闭：可滚轮缩放、拖动画布。点击“拉线”后，在俯视图上按住鼠标拖出黄色分割线。")
        self.hint.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.draw_button = QPushButton("拉线")
        self.clear_button = QPushButton("清空线")
        self.apply_button = QPushButton("拉线并应用")
        self.cancel_button = QPushButton("取消")

        button_row = QHBoxLayout()
        button_row.addWidget(self.hint, 1)
        button_row.addWidget(self.draw_button)
        button_row.addWidget(self.clear_button)
        button_row.addWidget(self.apply_button)
        button_row.addWidget(self.cancel_button)

        layout = QVBoxLayout(self)
        layout.addWidget(self.view, 1)
        layout.addLayout(button_row)

        self.draw_button.setCheckable(True)
        self.draw_button.toggled.connect(self._toggle_draw_mode)
        self.clear_button.clicked.connect(self._clear_barriers)
        self.apply_button.clicked.connect(self._apply)
        self.cancel_button.clicked.connect(self.reject)
        self._load_scene()

    def _toggle_draw_mode(self, enabled: bool) -> None:
        self.view.set_draw_mode(enabled)
        self.hint.setText("拉线模式开启：按住鼠标左键拖出黄色线段；可画多条线。" if enabled else "拉线模式关闭：可滚轮缩放、拖动画布。")

    def _load_scene(self) -> None:
        project = load_project_file(self.input_path)
        if not project.selected_path_face_regions:
            raise RuntimeError("输入项目没有 selected_path_face_regions。")
        invalid = [region for region in self.selected_regions if region <= 0 or region > len(project.selected_path_face_regions)]
        if invalid:
            raise RuntimeError(f"region 超出范围: {invalid}; 当前共有 {len(project.selected_path_face_regions)} 个 region。")

        selected_face_ids = {
            face_id
            for region_index, region in enumerate(project.selected_path_face_regions, 1)
            if region_index in self.selected_regions
            for face_id in region
        }
        importer = CadImportService().import_model(project.workpiece.file_path)
        reader = create_mesh_reader(importer.display_path, importer.display_format)
        reader.SetFileName(str(importer.display_path))
        reader.Update()
        triangles = read_triangles(reader.GetOutput(), selected_face_ids)
        self._triangles = triangles

        fill = QBrush(QColor(225, 232, 238))
        edge = QPen(QColor(88, 96, 104), 0.8)
        bounds: QRectF | None = None
        for triangle in triangles:
            polygon = QPolygonF([scene_point_from_model_xy((point[0], point[1])) for point in triangle.points])
            item = QGraphicsPolygonItem(polygon)
            item.setBrush(fill)
            item.setPen(edge)
            self.scene.addItem(item)
            bounds = polygon.boundingRect() if bounds is None else bounds.united(polygon.boundingRect())

        if bounds is None:
            raise RuntimeError("选中的 region 没有可显示的三角面。")
        margin = max(bounds.width(), bounds.height()) * 0.06
        self.scene.setSceneRect(bounds.adjusted(-margin, -margin, margin, margin))
        self.view.fitInView(self.scene.sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)

    def _add_barrier(self, barrier: BarrierLine) -> None:
        self.barriers.append(barrier)
        start, end = barrier
        start_scene = scene_point_from_model_xy(start)
        end_scene = scene_point_from_model_xy(end)
        item = self.scene.addLine(start_scene.x(), start_scene.y(), end_scene.x(), end_scene.y(), QPen(QColor(255, 230, 0), 6))
        item.setZValue(10)
        self._line_items.append(item)
        self.hint.setText(f"已添加 {len(self.barriers)} 条分割线。继续拉线或点击“拉线并应用”。")

    def _clear_barriers(self) -> None:
        self.barriers.clear()
        for item in self._line_items:
            self.scene.removeItem(item)
        self._line_items.clear()
        self.hint.setText("已清空分割线。")

    def _apply(self) -> None:
        if not self.barriers:
            QMessageBox.warning(self, "没有分割线", "请先点击“拉线”，在俯视图上画至少一条分割线。")
            return
        try:
            project = load_project_file(self.input_path)
            faces = face_geometries_from_triangles(self._triangles)
            records = manual_clip_manifest_records(project.selected_path_face_regions, self.selected_regions, faces, self.barriers)
            output_project = deepcopy(project)
            save_project_file(self.output_path, output_project)
            manifest = {
                "schema": "base_casting_abb6700.manual_region_partition_manifest",
                "version": 2,
                "input_project": str(self.input_path),
                "output_project": str(self.output_path),
                "selected_regions": sorted(self.selected_regions),
                "barriers_model_xy": [[list(start), list(end)] for start, end in self.barriers],
                "input_region_count": len(project.selected_path_face_regions),
                "output_region_count": len(project.selected_path_face_regions),
                "clip_patch_count": sum(record["output_patch_count"] for record in records),
                "records": records,
            }
            manual_manifest_path_for(self.output_path).write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
        except Exception as exc:
            QMessageBox.critical(self, "区域划分失败", str(exc))
            return
        self.accept()
