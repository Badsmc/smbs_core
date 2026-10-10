"""
pythonocc_backend.py - OpenCASCADE Technology Backend using pythonocc-core.
"""

from typing import Any, Tuple, Dict, Set, List, Optional
import math
import logging

from smbs_core.geometry_backend.base import GeometryBackend

logger = logging.getLogger(__name__)

try:
    from OCC.Core.STEPControl import STEPControl_Reader
    from OCC.Core.IFSelect import IFSelect_RetDone
    from OCC.Core.BRepTools import breptools
    from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform, BRepBuilderAPI_Copy
    from OCC.Core.gp import gp_Trsf, gp_Ax1, gp_Pnt, gp_Dir, gp_Vec
    from OCC.Core.Bnd import Bnd_Box
    from OCC.Core.BRepBndLib import brepbndlib
    from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Common
    from OCC.Core.GProp import GProp_GProps
    from OCC.Core.BRepGProp import brepgprop
    from OCC.Core.TopExp import TopExp_Explorer
    from OCC.Core.TopAbs import TopAbs_FACE, TopAbs_SOLID, TopAbs_COMPOUND
    from OCC.Core.BRepAdaptor import BRepAdaptor_Surface
    from OCC.Core.GeomAbs import GeomAbs_Plane, GeomAbs_Cylinder
    from OCC.Core.BRep import BRep_Tool, BRep_Builder
    from OCC.Core.TopoDS import TopoDS_Shape, topods, TopoDS_Compound
    from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox
    HAS_OCC = True
except ImportError:
    HAS_OCC = False


class PythonOCCBackend(GeometryBackend):
    def __init__(self):
        if not HAS_OCC:
            raise ImportError("pythonocc-core is required for PythonOCCBackend.")

    def load_shape(self, filepath: str) -> Any:
        reader = STEPControl_Reader()
        status = reader.ReadFile(filepath)
        if status == IFSelect_RetDone:
            reader.TransferRoots()
            return reader.Shape()
        raise ValueError(f"Failed to load STEP file: {filepath}")

    def export_brep(self, shape: Any, filepath: str) -> bool:
        if shape is None:
            return False
        return breptools.Write(shape, filepath)

    def copy_shape(self, shape: Any) -> Any:
        if shape is None:
            return None
        return BRepBuilderAPI_Copy(shape).Shape()

    def get_bounding_box(self, shape: Any) -> Tuple[float, float, float, float, float, float]:
        if shape is None:
            return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        bbox = Bnd_Box()
        brepbndlib.Add(shape, bbox, False)
        return bbox.Get()

    def make_box(self, x: float, y: float, z: float) -> Any:
        return BRepPrimAPI_MakeBox(x, y, z).Shape()

    def transform_shape(
        self,
        shape: Any,
        translation: Tuple[float, float, float],
        axis_origin: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        axis_dir: Tuple[float, float, float] = (0.0, 0.0, 1.0),
        angle_deg: float = 0.0
    ) -> Any:
        if shape is None:
            return None
        trsf = gp_Trsf()
        if angle_deg != 0.0:
            ax1 = gp_Ax1(gp_Pnt(*axis_origin), gp_Dir(*axis_dir))
            trsf.SetRotation(ax1, math.radians(angle_deg))
        if translation != (0.0, 0.0, 0.0):
            trsf_trans = gp_Trsf()
            trsf_trans.SetTranslation(gp_Vec(*translation))
            trsf.Multiply(trsf_trans)

        return BRepBuilderAPI_Transform(shape, trsf, False).Shape()

    def rotate_subtree(
        self,
        shape: Any,
        axis_origin: Tuple[float, float, float],
        axis_direction: Tuple[float, float, float],
        signed_angle_deg: float,
        moving_panels: Optional[Set[str]] = None,
        panel_graph: Optional[Any] = None
    ) -> Any:
        if shape is None:
            return None

        # Transform full shape using OCC matrix rotation
        return self.transform_shape(
            shape,
            translation=(0, 0, 0),
            axis_origin=axis_origin,
            axis_dir=axis_direction,
            angle_deg=signed_angle_deg
        )

    def intersect_volume(self, shape1: Any, shape2: Any) -> bool:
        if shape1 is None or shape2 is None:
            return False

        bb1 = self.get_bounding_box(shape1)
        bb2 = self.get_bounding_box(shape2)
        if not (bb1[0] <= bb2[3] and bb1[3] >= bb2[0] and
                bb1[1] <= bb2[4] and bb1[4] >= bb2[1] and
                bb1[2] <= bb2[5] and bb1[5] >= bb2[2]):
            return False

        try:
            common = BRepAlgoAPI_Common(shape1, shape2)
            common.Build()
            if not common.IsDone():
                return False

            result = common.Shape()
            props = GProp_GProps()
            brepgprop.VolumeProperties(result, props)
            return props.Mass() > 0.01
        except Exception:
            return False

    def get_faces(self, shape: Any) -> List[Any]:
        if shape is None:
            return []
        faces = []
        explorer = TopExp_Explorer(shape, TopAbs_FACE)
        while explorer.More():
            faces.append(explorer.Current())
            explorer.Next()
        return faces

    def extract_faces(self, shape: Any, face_ids: Set[str]) -> Any:
        if shape is None or not face_ids:
            return None

        builder = BRep_Builder()
        comp = TopoDS_Compound()
        builder.MakeCompound(comp)

        explorer = TopExp_Explorer(shape, TopAbs_FACE)
        idx = 0
        added = False
        while explorer.More():
            if str(idx) in face_ids:
                builder.Add(comp, explorer.Current())
                added = True
            idx += 1
            explorer.Next()

        return comp if added else None

    def is_valid(self, shape: Any) -> bool:
        return shape is not None and not shape.IsNull()

    def get_surface_type(self, face: Any) -> str:
        if face is None:
            return "Other"
        surf = BRepAdaptor_Surface(topods.Face(face))
        stype = surf.GetType()
        if stype == GeomAbs_Plane:
            return "Plane"
        elif stype == GeomAbs_Cylinder:
            return "Cylinder"
        return "Other"

    def get_face_normal(self, face: Any) -> Tuple[float, float, float]:
        if face is None:
            return (0.0, 0.0, 1.0)
        surf = BRepAdaptor_Surface(topods.Face(face))
        if surf.GetType() == GeomAbs_Plane:
            plane = surf.Plane()
            norm = plane.Axis().Direction()
            return (norm.X(), norm.Y(), norm.Z())
        return (0.0, 0.0, 1.0)

    def get_cylinder_data(self, face: Any) -> Dict[str, Any]:
        if face is None:
            return {}
        surf = BRepAdaptor_Surface(topods.Face(face))
        if surf.GetType() == GeomAbs_Cylinder:
            cyl = surf.Cylinder()
            ax = cyl.Axis()
            pos = ax.Location()
            dir_ = ax.Direction()
            return {
                "radius": cyl.Radius(),
                "origin": (pos.X(), pos.Y(), pos.Z()),
                "axis": (dir_.X(), dir_.Y(), dir_.Z()),
                "length": 100.0
            }
        return {}

    def get_face_adjacencies(self, shape: Any) -> Dict[str, List[str]]:
        adj_map: Dict[str, List[str]] = {}
        if shape is None:
            return adj_map

        faces = self.get_faces(shape)
        if not faces:
            return adj_map

        from OCC.Core.TopExp import TopExp_Explorer
        from OCC.Core.TopAbs import TopAbs_EDGE
        from OCC.Core.TopTools import TopTools_IndexedDataMapOfShapeListOfShape
        from OCC.Core.TopExp import topexp

        edge_map = TopTools_IndexedDataMapOfShapeListOfShape()
        topexp.MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edge_map)

        # Build face index mapping
        def face_hash(f):
            try:
                return hash(f.TShape())
            except Exception:
                return id(f)

        face_index_map = {}
        for i, face in enumerate(faces):
            face_index_map[face_hash(face)] = str(i)

        for i, face in enumerate(faces):
            f_id = str(i)
            adjacent_indices = set()
            explorer = TopExp_Explorer(face, TopAbs_EDGE)
            while explorer.More():
                edge = explorer.Current()
                if edge_map.Contains(edge):
                    face_list = edge_map.FindFromKey(edge)
                    from OCC.Core.TopTools import TopTools_ListIteratorOfListOfShape
                    it = TopTools_ListIteratorOfListOfShape(face_list)
                    while it.More():
                        adj_face = it.Value()
                        h = face_hash(adj_face)
                        if h in face_index_map and face_index_map[h] != f_id:
                            adjacent_indices.add(face_index_map[h])
                        it.Next()
                explorer.Next()
            adj_map[f_id] = list(adjacent_indices)

        return adj_map
