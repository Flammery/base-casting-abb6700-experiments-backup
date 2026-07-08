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

from manual_region_partitioning import (  # noqa: E402
    PARTITION_MODE_BOUNDARY,
    PARTITION_MODE_SLAB,
    BarrierLine,
    clip_partitions_from_barriers,
    manual_clip_manifest_records,
)
from region_partitioning import face_geometries_from_triangles  # noqa: E402


PREVIEW_COLORS = [
    QColor(242, 156, 30, 110),
    QColor(44, 160, 207, 110),
    QColor(94, 190, 112, 110),
    QColor(218, 92, 112, 110),
    QColor(156, 118, 217, 110),
    QColor(224, 196, 74, 110),
]

PARTITION_MODE_TITLES = {
    PARTITION_MODE_BOUNDARY: "面边界式",
    PARTITION_MODE_SLAB: "贯穿式",
}


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
        self._rubber_line = self.scene().addLine(
            self._start.x(),
            self._start.y(),
            self._start.x(),
            self._start.y(),
            QPen(QColor(255, 230, 0), 5),
        )
        self._rubber_line.setZValue(30)

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
        rubber_line = self._rubber_line
        self._rubber_line = None
        self._start = None
        self.scene().removeItem(rubber_line)
        if (start - end).manhattanLength() > 1.0:
            self.barrier_drawn.emit((model_xy_from_scene_point(start), model_xy_from_scene_point(end)))


class ManualPartitionDialog(QDialog):
    def __init__(
        self,
        input_path: Path,
        output_path: Path,
        selected_regions: set[int],
        parent=None,
        partition_mode: str = PARTITION_MODE_BOUNDARY,
    ) -> None:
        super().__init__(parent)
        self.partition_mode = partition_mode
        self.setWindowTitle(f"手动区域划分 - {PARTITION_MODE_TITLES.get(partition_mode, partition_mode)}")
        self.resize(1040, 760)
        self.input_path = input_path
        self.output_path = output_path
        self.selected_regions = sorted(set(selected_regions))
        self._current_index = 0
        self._barriers_by_region: dict[int, list[BarrierLine]] = {region: [] for region in self.selected_regions}
        self._geometry_cache: dict[int, tuple[list, dict]] = {}
        self._line_items: list[QGraphicsLineItem] = []
        self._partition_items: list[QGraphicsPolygonItem] = []
        self._current_face_ids: set[int] = set()

        self.scene = QGraphicsScene(self)
        self.scene.setBackgroundBrush(QBrush(QColor(8, 10, 12)))
        self.view = BarrierView(self.scene)
        self.view.barrier_drawn.connect(self._add_barrier)

        self.hint = QLabel("")
        self.hint.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.draw_button = QPushButton("拉线")
        self.clear_button = QPushButton("清空线")
        self.prev_button = QPushButton("上一个")
        self.next_button = QPushButton("下一个")
        self.apply_button = QPushButton("应用")
        self.cancel_button = QPushButton("取消")

        button_row = QHBoxLayout()
        button_row.addWidget(self.hint, 1)
        button_row.addWidget(self.draw_button)
        button_row.addWidget(self.clear_button)
        button_row.addWidget(self.prev_button)
        button_row.addWidget(self.next_button)
        button_row.addWidget(self.apply_button)
        button_row.addWidget(self.cancel_button)

        layout = QVBoxLayout(self)
        layout.addWidget(self.view, 1)
        layout.addLayout(button_row)

        self.draw_button.setCheckable(True)
        self.draw_button.toggled.connect(self._toggle_draw_mode)
        self.clear_button.clicked.connect(self._clear_barriers)
        self.prev_button.clicked.connect(self._show_previous_region)
        self.next_button.clicked.connect(self._show_next_region)
        self.apply_button.clicked.connect(self._apply)
        self.cancel_button.clicked.connect(self.reject)

        self._load_project()
        self._load_current_scene()

    def _current_region(self) -> int:
        return self.selected_regions[self._current_index]

    def _current_barriers(self) -> list[BarrierLine]:
        return self._barriers_by_region[self._current_region()]

    def _toggle_draw_mode(self, enabled: bool) -> None:
        self.view.set_draw_mode(enabled)
        self._update_hint()

    def _load_project(self) -> None:
        self.project = load_project_file(self.input_path)
        if not self.project.selected_path_face_regions:
            raise RuntimeError("输入项目没有 selected_path_face_regions。")
        invalid = [region for region in self.selected_regions if region <= 0 or region > len(self.project.selected_path_face_regions)]
        if invalid:
            raise RuntimeError(f"region 超出范围: {invalid}; 当前共有 {len(self.project.selected_path_face_regions)} 个 region。")

        importer = CadImportService().import_model(self.project.workpiece.file_path)
        reader = create_mesh_reader(importer.display_path, importer.display_format)
        reader.SetFileName(str(importer.display_path))
        reader.Update()
        self._polydata = reader.GetOutput()

    def _region_geometry(self, region_index: int):
        if region_index not in self._geometry_cache:
            face_ids = {int(face_id) for face_id in self.project.selected_path_face_regions[region_index - 1]}
            triangles = read_triangles(self._polydata, face_ids)
            faces = face_geometries_from_triangles(triangles)
            self._geometry_cache[region_index] = (triangles, faces)
        return self._geometry_cache[region_index]

    def _load_current_scene(self) -> None:
        self.scene.clear()
        self._line_items.clear()
        self._partition_items.clear()
        self.draw_button.setChecked(False)

        region_index = self._current_region()
        triangles, faces = self._region_geometry(region_index)
        self._triangles = triangles
        self._faces = faces
        self._current_face_ids = {int(face_id) for face_id in self.project.selected_path_face_regions[region_index - 1]}

        fill = QBrush(QColor(225, 232, 238))
        edge = QPen(QColor(88, 96, 104), 0.8)
        bounds: QRectF | None = None
        for triangle in triangles:
            polygon = QPolygonF([scene_point_from_model_xy((point[0], point[1])) for point in triangle.points])
            item = QGraphicsPolygonItem(polygon)
            item.setBrush(fill)
            item.setPen(edge)
            item.setZValue(0)
            self.scene.addItem(item)
            bounds = polygon.boundingRect() if bounds is None else bounds.united(polygon.boundingRect())

        if bounds is None:
            raise RuntimeError(f"region {region_index} 没有可显示的三角面。")
        margin = max(bounds.width(), bounds.height()) * 0.06
        self.scene.setSceneRect(bounds.adjusted(-margin, -margin, margin, margin))
        self.view.fitInView(self.scene.sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)

        for barrier in self._current_barriers():
            self._draw_barrier_line(barrier)
        self._refresh_partition_preview()
        self._update_nav_buttons()
        self._update_hint()

    def _draw_barrier_line(self, barrier: BarrierLine) -> None:
        start, end = barrier
        start_scene = scene_point_from_model_xy(start)
        end_scene = scene_point_from_model_xy(end)
        item = self.scene.addLine(start_scene.x(), start_scene.y(), end_scene.x(), end_scene.y(), QPen(QColor(255, 230, 0), 6))
        item.setZValue(30)
        self._line_items.append(item)

    def _add_barrier(self, barrier: BarrierLine) -> None:
        self._current_barriers().append(barrier)
        self._draw_barrier_line(barrier)
        self._refresh_partition_preview()
        self._update_hint()

    def _preview_patch_count(self) -> int:
        barriers = self._current_barriers()
        if not barriers:
            return 1
        return len(
            clip_partitions_from_barriers(
                self._current_region(),
                self._current_face_ids,
                self._faces,
                barriers,
                mode=self.partition_mode,
            )
        )

    def _refresh_partition_preview(self) -> None:
        for item in self._partition_items:
            self.scene.removeItem(item)
        self._partition_items.clear()
        barriers = self._current_barriers()
        if not barriers:
            return

        partitions = sorted(
            clip_partitions_from_barriers(
                self._current_region(),
                self._current_face_ids,
                self._faces,
                barriers,
                mode=self.partition_mode,
            ),
            key=lambda partition: 0 if partition.exclude_polygons_model_xy else 1,
        )
        for color_index, partition in enumerate(partitions):
            polygon = QPolygonF([scene_point_from_model_xy(point) for point in partition.clip_polygon_model_xy])
            item = QGraphicsPolygonItem(polygon)
            item.setBrush(QBrush(PREVIEW_COLORS[color_index % len(PREVIEW_COLORS)]))
            item.setPen(QPen(QColor(235, 240, 245, 180), 1.2))
            item.setZValue(10)
            self.scene.addItem(item)
            self._partition_items.append(item)

    def _clear_barriers(self) -> None:
        self._barriers_by_region[self._current_region()] = []
        for item in self._line_items:
            self.scene.removeItem(item)
        self._line_items.clear()
        self._refresh_partition_preview()
        self._update_hint()

    def _show_previous_region(self) -> None:
        if self._current_index <= 0:
            return
        self._current_index -= 1
        self._load_current_scene()

    def _show_next_region(self) -> None:
        if self._current_index >= len(self.selected_regions) - 1:
            return
        self._current_index += 1
        self._load_current_scene()

    def _update_nav_buttons(self) -> None:
        multiple = len(self.selected_regions) > 1
        self.prev_button.setEnabled(multiple and self._current_index > 0)
        self.next_button.setEnabled(multiple and self._current_index < len(self.selected_regions) - 1)
        self.prev_button.setVisible(multiple)
        self.next_button.setVisible(multiple)

    def _update_hint(self) -> None:
        region_index = self._current_region()
        barriers = len(self._current_barriers())
        draw_state = "拉线模式开启" if self.draw_button.isChecked() else "拉线模式关闭"
        page = f"{self._current_index + 1}/{len(self.selected_regions)}"
        self.hint.setText(
            f"{draw_state} | 当前 region {region_index} ({page}) | "
            f"分割线 {barriers} 条 | 预览分区 {self._preview_patch_count()} 个"
        )

    def _apply(self) -> None:
        missing = [region for region in self.selected_regions if not self._barriers_by_region.get(region)]
        if missing:
            QMessageBox.warning(self, "还有 region 未拉线", f"请先完成这些 region 的拉线: {missing}")
            return
        try:
            output_project = deepcopy(self.project)
            save_project_file(self.output_path, output_project)

            records: list[dict] = []
            for region_index in self.selected_regions:
                _triangles, faces = self._region_geometry(region_index)
                records.extend(
                    manual_clip_manifest_records(
                        self.project.selected_path_face_regions,
                        {region_index},
                        faces,
                        self._barriers_by_region[region_index],
                        mode=self.partition_mode,
                    )
                )

            manifest = {
                "schema": "base_casting_abb6700.manual_region_partition_manifest",
                "version": 2,
                "input_project": str(self.input_path),
                "output_project": str(self.output_path),
                "selected_regions": self.selected_regions,
                "partition_mode": self.partition_mode,
                "barriers_by_region": {
                    str(region): [[list(start), list(end)] for start, end in barriers]
                    for region, barriers in self._barriers_by_region.items()
                },
                "input_region_count": len(self.project.selected_path_face_regions),
                "output_region_count": len(self.project.selected_path_face_regions),
                "clip_patch_count": sum(record["output_patch_count"] for record in records),
                "records": records,
            }
            manual_manifest_path_for(self.output_path).write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
        except Exception as exc:
            QMessageBox.critical(self, "区域划分失败", str(exc))
            return
        self.accept()
