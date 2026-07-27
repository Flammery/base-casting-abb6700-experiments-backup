from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
from PySide6.QtCore import QPointF, Qt
from PySide6.QtGui import QBrush, QColor, QImage, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import QVBoxLayout, QWidget

from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
from vtkmodules.util.numpy_support import numpy_to_vtk
from vtkmodules.vtkCommonCore import VTK_UNSIGNED_CHAR, vtkFloatArray, vtkPoints, vtkUnsignedCharArray
from vtkmodules.vtkCommonDataModel import vtkCellArray, vtkImageData, vtkPolyData
from vtkmodules.vtkInteractionStyle import vtkInteractorStyleTrackballCamera
from vtkmodules.vtkRenderingCore import vtkActor, vtkPolyDataMapper, vtkRenderer, vtkTextActor, vtkTexture
import vtkmodules.vtkRenderingOpenGL2  # noqa: F401

EXPERIMENT_DIR = Path(__file__).resolve().parents[1]
ROOT = EXPERIMENT_DIR.parents[1]
SRC = ROOT / "src"
SCRIPT_DIR = EXPERIMENT_DIR / "scripts"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from robot_studio_qt.cad.import_service import CadImportService  # noqa: E402
from robot_studio_qt.cad.mesh_io import create_mesh_reader  # noqa: E402
from robot_studio_qt.project import load_project_file  # noqa: E402
from robot_studio_qt.path_planning.mesh_raster import read_triangles  # noqa: E402
from raster_domain import point_to_uv  # noqa: E402


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
RASTER_OVERLAY_OFFSET_FACTOR = -1.0
RASTER_OVERLAY_OFFSET_UNITS = -1.0


def manual_manifest_path_for(project_path: Path) -> Path:
    name = project_path.name
    stem = name[: -len(".rsp.json")] if name.endswith(".rsp.json") else project_path.stem
    return project_path.with_name(f"{stem}_manifest.json")


def build_raster_preview_plan(manifest: dict, regions: list[set[int]]) -> dict | None:
    """Replace partitioned source regions with patches and retain all others."""
    if (
        manifest.get("schema") != "base_casting_abb6700.manual_region_partition_manifest"
        or int(manifest.get("version", 0)) != 2
    ):
        return None

    records_by_source: dict[int, dict] = {}
    for record in manifest.get("records", []):
        source_region = int(record.get("original_region", 0))
        chart = record.get("raster_chart")
        patches = [patch for patch in record.get("patches", []) if patch.get("clip_polygon")]
        if chart and patches and 0 < source_region <= len(regions):
            records_by_source[source_region] = {"chart": chart, "patches": patches}
    if not records_by_source:
        return None

    groups: list[dict] = []
    passthrough: list[dict] = []
    labels: list[str] = []
    color_index = 0
    for source_region, face_ids in enumerate(regions, 1):
        record = records_by_source.get(source_region)
        if record is None:
            label = str(source_region)
            passthrough.append(
                {
                    "source_region": source_region,
                    "label": label,
                    "face_ids": face_ids,
                    "preview_color": PALETTE[color_index % len(PALETTE)],
                }
            )
            labels.append(label)
            color_index += 1
            continue

        colored_patches: list[dict] = []
        for patch_index, patch in enumerate(record["patches"], 1):
            colored_patch = dict(patch)
            label = str(patch.get("label", f"{source_region}_{patch_index}"))
            colored_patch["label"] = label
            colored_patch["source_region"] = source_region
            colored_patch["preview_color"] = PALETTE[color_index % len(PALETTE)]
            colored_patches.append(colored_patch)
            labels.append(label)
            color_index += 1
        groups.append(
            {
                "source_region": source_region,
                "face_ids": face_ids,
                "chart": record["chart"],
                "patches": colored_patches,
            }
        )
    return {"groups": groups, "passthrough": passthrough, "labels": labels}


def configure_raster_overlay_mapper(mapper: vtkPolyDataMapper) -> None:
    """Pull a textured coplanar overlay toward the camera in depth-buffer space."""
    mapper.SetResolveCoincidentTopologyToPolygonOffset()
    mapper.SetRelativeCoincidentTopologyPolygonOffsetParameters(
        RASTER_OVERLAY_OFFSET_FACTOR,
        RASTER_OVERLAY_OFFSET_UNITS,
    )


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
        self._path_actor: vtkActor | None = None
        self._volume_actor: vtkActor | None = None
        self._texture_actors: list[vtkActor] = []
        self._texture_data: list[object] = []
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
        self.clear_paths()
        self.clear_avoidance_volume()
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
            raster_plan = self._raster_preview_plan(project_path, regions)
            clip_patches = self._manual_clip_patches(project_path, regions)
            if raster_plan:
                self._apply_preview_entry_colors(polydata, raster_plan["passthrough"])
                labels = ",".join(raster_plan["labels"][:6])
                suffix = "..." if len(raster_plan["labels"]) > 6 else ""
                message = (
                    f"{project_path.name} | source regions: {len(regions)} | "
                    f"display regions: {len(raster_plan['labels'])} ({labels}{suffix})"
                )
            elif clip_patches:
                self._apply_clip_patch_colors(polydata, clip_patches)
                labels = ",".join(str(patch["label"]) for patch in clip_patches[:6])
                suffix = "..." if len(clip_patches) > 6 else ""
                message = f"{project_path.name} | regions: {len(regions)} | manual patches: {len(clip_patches)} ({labels}{suffix})"
            else:
                self._apply_region_colors(polydata, regions)
                message = f"{project_path.name} | regions: {len(regions)}"
            self._show_polydata(reader)
            if raster_plan:
                self._show_raster_textures(polydata, raster_plan["groups"])
            self.set_message(message)
        except Exception as exc:
            self.set_message(f"预览加载失败: {exc}")

    def _show_polydata(self, reader) -> None:
        if self._actor is not None:
            self.renderer.RemoveActor(self._actor)
        for texture_actor in self._texture_actors:
            self.renderer.RemoveActor(texture_actor)
        self._texture_actors.clear()
        self._texture_data.clear()
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

    def clear_paths(self) -> None:
        if self._path_actor is not None:
            self.renderer.RemoveActor(self._path_actor)
            self._path_actor = None
            self.vtk_widget.GetRenderWindow().Render()

    def clear_avoidance_volume(self) -> None:
        if self._volume_actor is not None:
            self.renderer.RemoveActor(self._volume_actor)
            self._volume_actor = None
        for actor in self._texture_actors:
            self.renderer.RemoveActor(actor)
        self._texture_actors.clear()
        self._texture_data.clear()
        self.vtk_widget.GetRenderWindow().Render()

    def show_neutral_model(self, message: str = "避障范围已隐藏") -> None:
        """Hide avoidance overlays and restore one neutral workpiece color."""

        self.clear_avoidance_volume()
        if self._reader is None:
            return
        polydata = self._reader.GetOutput()
        colors = vtkUnsignedCharArray()
        colors.SetNumberOfComponents(3)
        colors.SetName("NeutralModelColors")
        for _cell_id in range(int(polydata.GetNumberOfCells())):
            colors.InsertNextTypedTuple(DEFAULT_COLOR)
        polydata.GetCellData().SetScalars(colors)
        polydata.Modified()
        self.vtk_widget.GetRenderWindow().Render()
        self.set_message(message)

    def show_paths(self, paths: list[object]) -> None:
        """Overlay model-coordinate raster runs without joining holes or lines."""
        self.clear_paths()
        points = vtkPoints()
        lines = vtkCellArray()
        run_count = 0
        point_count = 0
        for path in paths:
            current: list[object] = []
            current_key = None
            for waypoint in path.waypoints:
                key = (waypoint.region_id, waypoint.line_id)
                if current_key is not None and key != current_key:
                    if len(current) >= 2:
                        lines.InsertNextCell(len(current))
                        for item in current:
                            lines.InsertCellPoint(points.InsertNextPoint(*item.position_model))
                        run_count += 1
                    current = []
                current.append(waypoint)
                current_key = key
                point_count += 1
            if len(current) >= 2:
                lines.InsertNextCell(len(current))
                for item in current:
                    lines.InsertCellPoint(points.InsertNextPoint(*item.position_model))
                run_count += 1
        polydata = vtkPolyData()
        polydata.SetPoints(points)
        polydata.SetLines(lines)
        mapper = vtkPolyDataMapper()
        mapper.SetInputData(polydata)
        actor = vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(0.18, 0.66, 1.0)
        actor.GetProperty().SetLineWidth(3.0)
        self._path_actor = actor
        self.renderer.AddActor(actor)
        self.vtk_widget.GetRenderWindow().Render()
        self.set_message(f"path preview: {point_count} points / {run_count} continuous runs")

    def show_support_surface_cells(self, support_cell_ids: set[int], seed_cell_ids: set[int]) -> None:
        """Color recovered support, path seeds, and remaining wall obstacles."""

        if self._reader is None:
            return
        polydata = self._reader.GetOutput()
        cell_count = int(polydata.GetNumberOfCells())
        colors = vtkUnsignedCharArray()
        colors.SetNumberOfComponents(3)
        colors.SetName("SupportObstacleColors")
        support_color = (48, 188, 126)
        seed_color = (250, 190, 42)
        obstacle_color = (174, 88, 88)
        for cell_id in range(cell_count):
            if cell_id in seed_cell_ids:
                color = seed_color
            elif cell_id in support_cell_ids:
                color = support_color
            else:
                color = obstacle_color
            colors.InsertNextTypedTuple(color)
        polydata.GetCellData().SetScalars(colors)
        polydata.Modified()
        for actor in self._texture_actors:
            actor.SetVisibility(False)
        self.vtk_widget.GetRenderWindow().Render()
        self.set_message(
            f"support preview: seeds={len(seed_cell_ids)} / support={len(support_cell_ids)} / "
            f"obstacles={max(0, cell_count - len(support_cell_ids))}"
        )

    def show_avoidance_volume(
        self,
        *,
        support_cell_ids: set[int],
        obstacle_cell_ids: set[int],
        selected_face_ids: set[int],
        volume_vertices: list[tuple[float, float, float]] | tuple[tuple[float, float, float], ...],
        clip_polygon: list[list[float]] | None = None,
        exclude_polygons: list[list[list[float]]] | None = None,
        raster_chart: dict | None = None,
        label: str = "",
    ) -> None:
        """Show selected machining area, support, local walls, and UVN volume."""

        if self._reader is None:
            return
        self.clear_avoidance_volume()
        polydata = self._reader.GetOutput()
        cell_count = int(polydata.GetNumberOfCells())
        colors = vtkUnsignedCharArray()
        colors.SetNumberOfComponents(3)
        colors.SetName("AvoidanceVolumeColors")
        support_color = (48, 188, 126)
        selected_color = (250, 190, 42)
        obstacle_color = (174, 70, 70)
        outside_color = (128, 134, 142)
        use_texture_selection = bool(raster_chart and clip_polygon)
        for cell_id in range(cell_count):
            if not use_texture_selection and cell_id in selected_face_ids:
                color = selected_color
            elif cell_id in support_cell_ids:
                color = support_color
            elif cell_id in obstacle_cell_ids:
                color = obstacle_color
            else:
                color = outside_color
            colors.InsertNextTypedTuple(color)
        polydata.GetCellData().SetScalars(colors)
        polydata.Modified()

        if len(volume_vertices) == 8:
            points = vtkPoints()
            for point in volume_vertices:
                points.InsertNextPoint(*point)
            faces = vtkCellArray()
            for face in (
                (0, 1, 3, 2),
                (4, 6, 7, 5),
                (0, 4, 5, 1),
                (2, 3, 7, 6),
                (0, 2, 6, 4),
                (1, 5, 7, 3),
            ):
                faces.InsertNextCell(len(face))
                for point_id in face:
                    faces.InsertCellPoint(point_id)
            volume_mesh = vtkPolyData()
            volume_mesh.SetPoints(points)
            volume_mesh.SetPolys(faces)
            mapper = vtkPolyDataMapper()
            mapper.SetInputData(volume_mesh)
            mapper.ScalarVisibilityOff()
            actor = vtkActor()
            actor.SetMapper(mapper)
            actor.ForceTranslucentOn()
            actor.GetProperty().SetColor(0.66, 0.69, 0.73)
            actor.GetProperty().SetOpacity(0.14)
            actor.GetProperty().EdgeVisibilityOn()
            actor.GetProperty().SetEdgeColor(0.82, 0.85, 0.90)
            actor.GetProperty().SetLineWidth(1.5)
            self.renderer.AddActor(actor)
            self._volume_actor = actor
            self._texture_data.extend([volume_mesh, mapper])

        if use_texture_selection:
            self._show_raster_textures(
                polydata,
                [
                    {
                        "face_ids": selected_face_ids,
                        "chart": raster_chart,
                        "patches": [
                            {
                                "clip_polygon": clip_polygon,
                                "exclude_polygons": exclude_polygons or [],
                                "preview_color": selected_color,
                            }
                        ],
                    }
                ],
            )
        self.vtk_widget.GetRenderWindow().Render()
        self.set_message(
            f"avoidance {label}: selected={len(selected_face_ids)} / "
            f"support={len(support_cell_ids)} / walls={len(obstacle_cell_ids)}"
        )

    def _raster_preview_plan(self, project_path: Path, regions: list[set[int]]) -> dict | None:
        manifest_path = manual_manifest_path_for(project_path)
        if not manifest_path.exists():
            return None
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        except Exception:
            return None
        return build_raster_preview_plan(manifest, regions)

    def _show_raster_textures(self, polydata, groups: list[dict]) -> None:
        """Paint PySide partition masks and texture them onto the unchanged STL."""
        for group in groups:
            patches, chart = group["patches"], group["chart"]
            all_points = [point for patch in patches for point in patch["clip_polygon"]]
            if not all_points:
                continue
            u_min, u_max = min(point[0] for point in all_points), max(point[0] for point in all_points)
            v_min, v_max = min(point[1] for point in all_points), max(point[1] for point in all_points)
            u_pad = max((u_max - u_min) * 0.02, 1e-6)
            v_pad = max((v_max - v_min) * 0.02, 1e-6)
            u_min, u_max = u_min - u_pad, u_max + u_pad
            v_min, v_max = v_min - v_pad, v_max + v_pad
            u_span, v_span = max(u_max - u_min, 1e-9), max(v_max - v_min, 1e-9)
            size = 1024
            image = QImage(size, size, QImage.Format.Format_RGBA8888)
            image.fill(Qt.GlobalColor.transparent)
            painter = QPainter(image)
            painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)

            def image_polygon(polygon):
                return QPolygonF(
                    [QPointF((point[0] - u_min) / u_span * (size - 1), (v_max - point[1]) / v_span * (size - 1)) for point in polygon]
                )

            ordered = sorted(enumerate(patches), key=lambda item: 0 if item[1].get("exclude_polygons") else 1)
            for _patch_index, patch in ordered:
                color = patch["preview_color"]
                painter.setCompositionMode(QPainter.CompositionMode.CompositionMode_SourceOver)
                painter.setPen(QPen(Qt.PenStyle.NoPen))
                painter.setBrush(QBrush(QColor(*color, 210)))
                painter.drawPolygon(image_polygon(patch["clip_polygon"]))
                painter.setCompositionMode(QPainter.CompositionMode.CompositionMode_Clear)
                for excluded in patch.get("exclude_polygons") or []:
                    painter.drawPolygon(image_polygon(excluded))
            painter.end()

            overlay, tcoords = self._textured_region_mesh(polydata, group["face_ids"], chart, u_min, u_span, v_max, v_span)
            overlay.GetPointData().SetTCoords(tcoords)
            vtk_image = vtkImageData()
            vtk_image.SetDimensions(size, size, 1)
            rgba = np.frombuffer(image.bits(), dtype=np.uint8, count=image.sizeInBytes()).reshape((-1, 4)).copy()
            scalars = numpy_to_vtk(rgba, deep=True, array_type=VTK_UNSIGNED_CHAR)
            scalars.SetNumberOfComponents(4)
            vtk_image.GetPointData().SetScalars(scalars)
            texture = vtkTexture()
            texture.SetInputData(vtk_image)
            texture.InterpolateOn()
            texture.RepeatOff()
            mapper = vtkPolyDataMapper()
            mapper.SetInputData(overlay)
            mapper.ScalarVisibilityOff()
            configure_raster_overlay_mapper(mapper)
            actor = vtkActor()
            actor.SetMapper(mapper)
            actor.SetTexture(texture)
            actor.ForceTranslucentOn()
            actor.GetProperty().SetAmbient(0.45)
            actor.GetProperty().SetDiffuse(0.65)
            self.renderer.AddActor(actor)
            self._texture_actors.append(actor)
            self._texture_data.extend([image, overlay, tcoords, vtk_image, scalars, texture, mapper])
        self.vtk_widget.GetRenderWindow().Render()

    @staticmethod
    def _textured_region_mesh(polydata, face_ids, chart, u_min, u_span, v_max, v_span):
        points = vtkPoints()
        polys = vtkCellArray()
        tcoords = vtkFloatArray()
        tcoords.SetNumberOfComponents(2)
        for triangle in read_triangles(polydata, face_ids):
            ids = []
            for point in triangle.points:
                ids.append(points.InsertNextPoint(*point))
                u, v = point_to_uv(point, chart)
                tcoords.InsertNextTuple2((u - u_min) / u_span, (v_max - v) / v_span)
            polys.InsertNextCell(3)
            for point_id in ids:
                polys.InsertCellPoint(point_id)
        output = vtkPolyData()
        output.SetPoints(points)
        output.SetPolys(polys)
        return output, tcoords

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
        # Raster-domain partitions are represented by their preview paths, not
        # by coloring whole STL cells. The mesh remains only the lift/normal source.
        if int(manifest.get("version", 0)) == 2 and any(record.get("raster_chart") for record in manifest.get("records", [])):
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
        entries = [
            {"face_ids": region, "preview_color": PALETTE[index % len(PALETTE)]}
            for index, region in enumerate(regions)
        ]
        self._apply_preview_entry_colors(polydata, entries)

    def _apply_preview_entry_colors(self, polydata, entries: list[dict]) -> None:
        cell_count = polydata.GetNumberOfCells()
        colors = vtkUnsignedCharArray()
        colors.SetNumberOfComponents(3)
        colors.SetName("RegionColors")
        face_to_color: dict[int, tuple[int, int, int]] = {}
        for entry in entries:
            color = entry["preview_color"]
            for face_id in entry["face_ids"]:
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
