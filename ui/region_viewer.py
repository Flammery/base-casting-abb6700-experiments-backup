from __future__ import annotations

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
            self._apply_region_colors(polydata, regions)
            self._show_polydata(reader)
            self.set_message(f"{project_path.name} | regions: {len(regions)}")
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
