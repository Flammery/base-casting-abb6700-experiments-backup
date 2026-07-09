from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any

EXPERIMENT_DIR = Path(__file__).resolve().parents[1]
ROOT = EXPERIMENT_DIR.parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from robot_studio_qt.core.geometry import cross, normalize

Point2 = tuple[float, float]
Vector3 = tuple[float, float, float]


@dataclass(frozen=True)
class CadFaceInfo:
    cad_face_id: int
    signature: dict[str, Any]


@dataclass(frozen=True)
class CadSurfaceSample:
    label: str
    cad_face_id: int
    line_id: int
    point_id: int
    uv: Point2
    position_model: Vector3
    normal_model: Vector3
    tangent_model: Vector3


def load_step_shape(step_path: str | Path):
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.STEPControl import STEPControl_Reader

    source = Path(step_path)
    reader = STEPControl_Reader()
    status = reader.ReadFile(str(source))
    if status != IFSelect_RetDone:
        raise RuntimeError(f"OpenCascade cannot read STEP file: {source}")
    reader.TransferRoots()
    shape = reader.OneShape()
    if shape.IsNull():
        raise RuntimeError(f"STEP file has no B-Rep shape: {source}")
    return shape


def iter_faces(shape):
    from OCP.TopAbs import TopAbs_FACE
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopoDS import TopoDS

    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    while explorer.More():
        yield TopoDS.Face_s(explorer.Current())
        explorer.Next()


def build_face_index(step_path: str | Path) -> dict[str, Any]:
    shape = load_step_shape(step_path)
    faces = []
    for cad_face_id, face in enumerate(iter_faces(shape), 1):
        faces.append({"cad_face_id": cad_face_id, "signature": face_signature(face)})
    return {
        "schema": "base_casting_abb6700.step_cad_manifest",
        "version": 1,
        "source_step": str(Path(step_path)),
        "faces": faces,
        "selected_cad_face_regions": [],
        "manual_partitions": [],
        "patches": [],
    }


def face_signature(face) -> dict[str, Any]:
    from OCP.BRepBndLib import BRepBndLib
    from OCP.BRepGProp import BRepGProp
    from OCP.Bnd import Bnd_Box
    from OCP.GProp import GProp_GProps

    props = GProp_GProps()
    BRepGProp.SurfaceProperties_s(face, props)
    center = props.CentreOfMass()

    bbox = Bnd_Box()
    BRepBndLib.Add_s(face, bbox)
    x_min, y_min, z_min, x_max, y_max, z_max = bbox.Get()

    edge_lengths = sorted(round(length, 6) for length in edge_length_summary(face))
    wire_count = len(face_wires(face))
    normal = surface_normal_at_middle(face)
    payload = {
        "area": round(float(props.Mass()), 6),
        "center": [round(center.X(), 6), round(center.Y(), 6), round(center.Z(), 6)],
        "bbox": [round(value, 6) for value in (x_min, y_min, z_min, x_max, y_max, z_max)],
        "normal_hint": [round(value, 6) for value in normal],
        "wire_count": wire_count,
        "hole_count_hint": max(0, wire_count - 1),
        "edge_lengths": edge_lengths,
    }
    payload["hash"] = hashlib.sha1(json.dumps(payload, sort_keys=True).encode("utf-8")).hexdigest()[:16]
    return payload


def face_wires(face):
    from OCP.TopAbs import TopAbs_WIRE
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopoDS import TopoDS

    wires = []
    explorer = TopExp_Explorer(face, TopAbs_WIRE)
    while explorer.More():
        wires.append(TopoDS.Wire_s(explorer.Current()))
        explorer.Next()
    return wires


def edge_length_summary(face) -> list[float]:
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps
    from OCP.TopAbs import TopAbs_EDGE
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopoDS import TopoDS

    lengths = []
    explorer = TopExp_Explorer(face, TopAbs_EDGE)
    while explorer.More():
        edge = TopoDS.Edge_s(explorer.Current())
        props = GProp_GProps()
        BRepGProp.LinearProperties_s(edge, props)
        lengths.append(float(props.Mass()))
        explorer.Next()
    return lengths


def face_by_id(shape, cad_face_id: int):
    for index, face in enumerate(iter_faces(shape), 1):
        if index == cad_face_id:
            return face
    raise KeyError(f"cad_face_id not found: {cad_face_id}")


def surface_normal_at_middle(face) -> Vector3:
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.TopAbs import TopAbs_REVERSED
    from OCP.gp import gp_Pnt, gp_Vec

    surface = BRepAdaptor_Surface(face)
    u = midpoint_parameter(surface.FirstUParameter(), surface.LastUParameter())
    v = midpoint_parameter(surface.FirstVParameter(), surface.LastVParameter())
    point = gp_Pnt()
    du = gp_Vec()
    dv = gp_Vec()
    surface.D1(u, v, point, du, dv)
    normal = normalize(cross((du.X(), du.Y(), du.Z()), (dv.X(), dv.Y(), dv.Z())))
    if face.Orientation() == TopAbs_REVERSED:
        normal = (-normal[0], -normal[1], -normal[2])
    return normal


def midpoint_parameter(first: float, last: float) -> float:
    if math.isinf(first) or math.isinf(last):
        raise ValueError("Unbounded CAD surface is not supported in v1.")
    return (first + last) * 0.5


def point_in_polygon_xy(point: Point2, polygon: list[Point2]) -> bool:
    if len(polygon) < 3:
        return False
    inside = False
    x, y = point
    previous = polygon[-1]
    for current in polygon:
        xi, yi = current
        xj, yj = previous
        if (yi > y) != (yj > y):
            cross_x = (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi
            if x <= cross_x:
                inside = not inside
        previous = current
    return inside


def picked_patch_records(
    selected_cad_face_regions: list[list[int]],
    picked_polygons_by_region: dict[int, list[list[Point2]]],
    face_index: dict[int, dict[str, Any]],
) -> list[dict[str, Any]]:
    records = []
    for region_index, cad_face_ids in enumerate(selected_cad_face_regions, 1):
        polygons = picked_polygons_by_region.get(region_index, [])
        patches = []
        for patch_index, polygon in enumerate(polygons, 1):
            if len(polygon) < 3:
                continue
            patches.append(
                {
                    "label": f"{region_index}_{patch_index}",
                    "source_region": region_index,
                    "cad_face_ids": list(cad_face_ids),
                    "clip_space": "model_xy",
                    "clip_polygon": [list(point) for point in polygon],
                    "face_signatures": [face_index[face_id]["signature"] for face_id in cad_face_ids if face_id in face_index],
                }
            )
        records.append(
            {
                "original_region": region_index,
                "reason": "manual_cad_pick_clip",
                "partition_mode": "pick",
                "output_patch_count": len(patches),
                "patches": patches,
            }
        )
    return records


def attach_pick_partitions(
    manifest: dict[str, Any],
    selected_cad_face_regions: list[list[int]],
    picked_polygons_by_region: dict[int, list[list[Point2]]],
) -> dict[str, Any]:
    face_index = {int(face["cad_face_id"]): face for face in manifest.get("faces", [])}
    records = picked_patch_records(selected_cad_face_regions, picked_polygons_by_region, face_index)
    updated = dict(manifest)
    updated["selected_cad_face_regions"] = [list(region) for region in selected_cad_face_regions]
    updated["manual_partitions"] = records
    updated["patches"] = [patch for record in records for patch in record["patches"]]
    return updated


def sample_manifest_patches(
    step_path: str | Path,
    manifest: dict[str, Any],
    spacing: float,
    point_step: float,
    boundary_margin: float = 0.0,
) -> list[CadSurfaceSample]:
    shape = load_step_shape(step_path)
    samples: list[CadSurfaceSample] = []
    for patch in manifest.get("patches", []):
        clip_polygon = [(float(x), float(y)) for x, y in patch.get("clip_polygon", [])]
        if len(clip_polygon) < 3:
            continue
        for cad_face_id in patch.get("cad_face_ids", []):
            face = face_by_id(shape, int(cad_face_id))
            samples.extend(sample_face_patch(face, int(cad_face_id), str(patch["label"]), clip_polygon, spacing, point_step, boundary_margin))
    return samples


def sample_face_patch(
    face,
    cad_face_id: int,
    label: str,
    clip_polygon_xy: list[Point2],
    spacing: float,
    point_step: float,
    boundary_margin: float = 0.0,
) -> list[CadSurfaceSample]:
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.BRepClass import BRepClass_FaceClassifier
    from OCP.TopAbs import TopAbs_IN, TopAbs_ON, TopAbs_REVERSED
    from OCP.gp import gp_Pnt, gp_Vec

    if spacing <= 0.0 or point_step <= 0.0:
        raise ValueError("spacing and point_step must be positive")

    surface = BRepAdaptor_Surface(face)
    u_min = float(surface.FirstUParameter()) + boundary_margin
    u_max = float(surface.LastUParameter()) - boundary_margin
    v_min = float(surface.FirstVParameter()) + boundary_margin
    v_max = float(surface.LastVParameter()) - boundary_margin
    if u_max < u_min or v_max < v_min:
        return []

    samples: list[CadSurfaceSample] = []
    v = v_min
    line_id = 0
    while v <= v_max + 1e-9:
        u = u_min
        point_id = 0
        while u <= u_max + 1e-9:
            point = gp_Pnt()
            du = gp_Vec()
            dv = gp_Vec()
            surface.D1(u, v, point, du, dv)
            position = (point.X(), point.Y(), point.Z())
            if point_in_polygon_xy((position[0], position[1]), clip_polygon_xy):
                classifier = BRepClass_FaceClassifier(face, point, 1e-7)
                if classifier.State() in (TopAbs_IN, TopAbs_ON):
                    normal = normalize(cross((du.X(), du.Y(), du.Z()), (dv.X(), dv.Y(), dv.Z())))
                    if face.Orientation() == TopAbs_REVERSED:
                        normal = (-normal[0], -normal[1], -normal[2])
                    tangent = normalize((du.X(), du.Y(), du.Z()))
                    samples.append(CadSurfaceSample(label, cad_face_id, line_id, point_id, (u, v), position, normal, tangent))
                    point_id += 1
            u += point_step
        v += spacing
        line_id += 1
    return samples


def tessellated_polydata_with_face_ids(step_path: str | Path, linear_deflection: float = 0.5):
    from OCP.BRep import BRep_Tool
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.TopLoc import TopLoc_Location
    from vtkmodules.vtkCommonCore import vtkIdList, vtkIntArray, vtkPoints
    from vtkmodules.vtkCommonDataModel import vtkCellArray, vtkPolyData

    shape = load_step_shape(step_path)
    mesh = BRepMesh_IncrementalMesh(shape, linear_deflection, False, 0.5, True)
    mesh.Perform()
    if not mesh.IsDone():
        raise RuntimeError("OpenCascade tessellation failed")

    points = vtkPoints()
    polys = vtkCellArray()
    face_ids = vtkIntArray()
    face_ids.SetName("cad_face_id")

    for cad_face_id, face in enumerate(iter_faces(shape), 1):
        location = TopLoc_Location()
        triangulation = BRep_Tool.Triangulation_s(face, location)
        if triangulation is None:
            continue
        transform = location.Transformation()
        local_to_global: dict[int, int] = {}
        for node_index in range(1, triangulation.NbNodes() + 1):
            point = triangulation.Node(node_index).Transformed(transform)
            local_to_global[node_index] = points.InsertNextPoint(point.X(), point.Y(), point.Z())
        for tri_index in range(1, triangulation.NbTriangles() + 1):
            ids = triangulation.Triangle(tri_index).Get()
            cell_ids = vtkIdList()
            for node_id in ids:
                cell_ids.InsertNextId(local_to_global[int(node_id)])
            polys.InsertNextCell(cell_ids)
            face_ids.InsertNextValue(cad_face_id)

    polydata = vtkPolyData()
    polydata.SetPoints(points)
    polydata.SetPolys(polys)
    polydata.GetCellData().AddArray(face_ids)
    polydata.GetCellData().SetActiveScalars("cad_face_id")
    return polydata


def samples_to_jsonable(samples: list[CadSurfaceSample]) -> list[dict[str, Any]]:
    return [
        {
            "label": sample.label,
            "cad_face_id": sample.cad_face_id,
            "line_id": sample.line_id,
            "point_id": sample.point_id,
            "uv": list(sample.uv),
            "position_model": list(sample.position_model),
            "normal_model": list(sample.normal_model),
            "tangent_model": list(sample.tangent_model),
        }
        for sample in samples
    ]


def read_manifest(path: str | Path) -> dict[str, Any]:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def write_manifest(path: str | Path, manifest: dict[str, Any]) -> None:
    Path(path).write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
