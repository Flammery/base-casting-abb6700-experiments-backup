from __future__ import annotations

import json
import sys
from pathlib import Path

from PySide6.QtWidgets import QVBoxLayout, QWidget

from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
from vtkmodules.vtkCommonCore import vtkUnsignedCharArray
from vtkmodules.vtkInteractionStyle import vtkInteractorStyleTrackballCamera
from vtkmodules.vtkRenderingCore import vtkActor, vtkPolyDataMapper, vtkRenderer, vtkTextActor
import vtkmodules.vtkRenderingOpenGL2  # noqa: F401

EXPERIMENT_DIR = Path(__file__).resolve().parents[1]
ROOT = EXPERIMENT_DIR.parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from robot_studio_qt.cad.import_service import CadImportService  # noqa: E402
from robot_studio_qt.cad.mesh_io import create_mesh_reader  # noqa: E402
from robot_studio_qt.project import load_project_file  # noqa: E402


DEFAULT_COLOR = (150, 158, 168)
PALETTE = [
    (242, 156, 30),
    (44, 160, 207),
    (94, 190, 112),
    (218, 92, 112),
    (156, 118, 217),
    (224, 196, 74),
    (60, 184, 172),
    (232, 128, 64),
    (118, 174, 224),
    (196, 116, 164),
]


def manual_manifest_path_for(project_path: Path) -> Path:
    name = project_path.name
    stem = name[: -len(".rsp.json")] if name.endswith(".rsp.json") else project_path.stem
    return project_path.with_name(f"{stem}_manifest.json")


def point_in_polygon_xy(point: tuple[float, float], polygon: list[list[float]]) -> bool:
    x, y = point
    inside = False
    previous = polygon[-1]
    for current in polygon:
        yi, yj = current[1], previous[1]
        xi, xj = current[0], previous[0]
        if (yi > y) != (yj > y):
            cross_x = (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi
            if x < cross_x:
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


def cell_centroid_xy(polydata, cell_id: int) -> tuple[float, float]:
    cell = polydata.GetCell(cell_id)
    point_ids = cell.GetPointIds()
    count = point_ids.GetNumberOfIds()
    if count <= 0:
        return (0.0, 0.0)
    x_sum = 0.0
    y_sum = 0.0
    for index in range(count):
        point = polydata.GetPoint(point_ids.GetId(index))
        x_sum += float(point[0])
        y_sum += float(point[1])
    return (x_sum / count, y_sum / count)


class RegionPreview(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.setMinimumHeight(360)
        self._actor: vtkActor | None = None
        self._reader = None

        self.renderer = vtkRenderer()
        self.renderer.SetBackground(0.09, 0.10, 0.11)
        self.vtk_widget = QVTKRenderWindowInteractor(self)
        self.vtk_widget.GetRenderWindow().AddRenderer(self.renderer)
        self.interactor = self.vtk_widget.GetRenderWindow().GetInteractor()
        self.interactor.SetInteractorStyle(vtkInteractorStyleTrackballCamera())

        self.text_actor = vtkTextActor()
        self.text_actor.SetPosition(12, 12)
        self.text_actor.GetTextProperty().SetFontSize(14)
        self.text_actor.GetTextProperty().SetColor(0.88, 0.92, 0.96)
        self.renderer.AddActor2D(self.text_actor)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.vtk_widget)
        self.interactor.Initialize()
        self.set_message("未加载输入")

    def set_message(self, message: str) -> None:
        self.text_actor.SetInput(message)
        self.vtk_widget.GetRenderWindow().Render()

    def load_project(self, project_path: Path) -> None:
        if not project_path.exists():
            self.set_message(f"输入不存在: {project_path}")
            return
        try:
            project = load_project_file(project_path)
            importer = CadImportService().import_model(project.workpiece.file_path)
            reader = create_mesh_reader(importer.display_path, importer.display_format)
            reader.SetFileName(str(importer.display_path))
            reader.Update()
            polydata = reader.GetOutput()
            regions = [set(region) for region in project.selected_path_face_regions]
            clip_patches = self._manual_clip_patches(project_path, regions)
            if clip_patches:
                self._apply_clip_patch_colors(polydata, clip_patches)
                labels = ",".join(str(patch["label"]) for patch in clip_patches[:6])
                suffix = "..." if len(clip_patches) > 6 else ""
                message = f"{project_path.name} | regions: {len(regions)} | manual patches: {len(clip_patches)} ({labels}{suffix})"
            else:
                self._apply_region_colors(polydata, regions)
                message = f"{project_path.name} | regions: {len(regions)}"
            self._show_polydata(reader)
            self.set_message(message)
        except Exception as exc:
            self.set_message(f"预览加载失败: {exc}")

    def _show_polydata(self, reader) -> None:
        if self._actor is not None:
            self.renderer.RemoveActor(self._actor)
        mapper = vtkPolyDataMapper()
        mapper.SetInputConnection(reader.GetOutputPort())
        mapper.SetScalarModeToUseCellData()
        mapper.SetColorModeToDirectScalars()
        actor = vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetAmbient(0.28)
        actor.GetProperty().SetDiffuse(0.70)
        actor.GetProperty().SetSpecular(0.14)
        actor.GetProperty().SetSpecularPower(18)
        self._reader = reader
        self._actor = actor
        self.renderer.AddActor(actor)
        self.renderer.ResetCamera()
        self.vtk_widget.GetRenderWindow().Render()

    def _manual_clip_patches(self, project_path: Path, regions: list[set[int]]) -> list[dict]:
        manifest_path = manual_manifest_path_for(project_path)
        if not manifest_path.exists():
            return []
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        except Exception:
            return []
        if manifest.get("schema") != "base_casting_abb6700.manual_region_partition_manifest":
            return []

        patches: list[dict] = []
        patched_sources: set[int] = set()
        for record in manifest.get("records", []):
            source_region = int(record.get("original_region", 0))
            if source_region <= 0 or source_region > len(regions):
                continue
            patched_sources.add(source_region)
            for patch in record.get("patches", []):
                polygon = patch.get("clip_polygon")
                if not polygon:
                    continue
                patches.append(
                    {
                        "source_region": source_region,
                        "label": str(patch.get("label", f"{source_region}_1")),
                        "face_ids": set(regions[source_region - 1]),
                        "clip_polygon": polygon,
                        "exclude_polygons": patch.get("exclude_polygons") or [],
                    }
                )
        for index, region in enumerate(regions, 1):
            if index not in patched_sources:
                patches.append({"source_region": index, "label": str(index), "face_ids": region, "clip_polygon": None, "exclude_polygons": []})
        return patches

    def _apply_region_colors(self, polydata, regions: list[set[int]]) -> None:
        cell_count = polydata.GetNumberOfCells()
        colors = vtkUnsignedCharArray()
        colors.SetNumberOfComponents(3)
        colors.SetName("RegionColors")
        face_to_color: dict[int, tuple[int, int, int]] = {}
        for index, region in enumerate(regions):
            color = PALETTE[index % len(PALETTE)]
            for face_id in region:
                face_to_color[int(face_id)] = color
        for cell_id in range(cell_count):
            colors.InsertNextTypedTuple(face_to_color.get(cell_id, DEFAULT_COLOR))
        polydata.GetCellData().SetScalars(colors)
        polydata.Modified()

    def _apply_clip_patch_colors(self, polydata, patches: list[dict]) -> None:
        cell_count = polydata.GetNumberOfCells()
        colors = vtkUnsignedCharArray()
        colors.SetNumberOfComponents(3)
        colors.SetName("RegionColors")
        patch_colors = [PALETTE[index % len(PALETTE)] for index in range(len(patches))]

        # `.rsp` 里仍保留原始 region；这里按 manifest 的规则 UV clip polygon 给三角面临时上色，方便检查分区效果。
        for cell_id in range(cell_count):
            centroid = cell_centroid_xy(polydata, cell_id)
            color = DEFAULT_COLOR
            for patch_index, patch in enumerate(patches):
                if cell_id not in patch["face_ids"]:
                    continue
                if point_allowed_by_clip(centroid, patch.get("clip_polygon"), patch.get("exclude_polygons")):
                    color = patch_colors[patch_index]
                    break
            colors.InsertNextTypedTuple(color)
        polydata.GetCellData().SetScalars(colors)
        polydata.Modified()
