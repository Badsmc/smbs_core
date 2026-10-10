import math
from typing import Any, Tuple, Set, List, Dict
from smbs_core.geometry_backend.base import GeometryBackend

try:
    from OCC.Core.STEPControl import STEPControl_Reader, STEPControl_Writer, STEPControl_AsIs
    from OCC.Core.IFSelect import IFSelect_RetDone
    from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform, BRepBuilderAPI_MakeFace
    from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Common
    from OCC.Core.gp import gp_Trsf, gp_Vec, gp_Ax1, gp_Pnt, gp_Dir, gp_Quaternion
    from OCC.Core.Bnd import Bnd_Box
    from OCC.Core.BRepBndLib import brepbndlib
    from OCC.Core.TopoDS import TopoDS_Shape
    from OCC.Core.TopExp import TopExp_Explorer
    from OCC.Core.TopAbs import TopAbs_FACE
    from OCC.Core.BRep import BRep_Builder
    from OCC.Core.TopoDS import topods
    from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeFace
    HAS_OCC = True
except ImportError:
    HAS_OCC = False


class PythonOCCBackend(GeometryBackend):
    """
    Implementation of GeometryBackend using pythonocc-core.
    Provides headless CAD operations.
    """
    
    def __init__(self):
        if not HAS_OCC:
            raise ImportError("pythonocc-core is not installed. Cannot use PythonOCCBackend.")
            
    def load_shape(self, filepath: str) -> Any:
        reader = STEPControl_Reader()
        status = reader.ReadFile(filepath)
        if status == IFSelect_RetDone:
            reader.TransferRoots()
            return reader.OneShape()
        return None

    def export_brep(self, shape: Any, filepath: str) -> bool:
        writer = STEPControl_Writer()
        writer.Transfer(shape, STEPControl_AsIs)
        status = writer.Write(filepath)
        return status == IFSelect_RetDone

    def copy_shape(self, shape: Any) -> Any:
        # In OCC, topological shapes are handles. BRepBuilderAPI_Transform with identity 
        # can copy, or we can just return the shape since they are immutable in many contexts.
        trsf = gp_Trsf()
        transform = BRepBuilderAPI_Transform(shape, trsf, True) # True for copy
        return transform.Shape()

    def get_bounding_box(self, shape: Any) -> Tuple[float, float, float, float, float, float]:
        if shape is None:
            return None
        bbox = Bnd_Box()
        brepbndlib.Add(shape, bbox)
        xmin, ymin, zmin, xmax, ymax, zmax = bbox.Get()
        return (xmin, ymin, zmin, xmax, ymax, zmax)

    def make_box(self, x: float, y: float, z: float) -> Any:
        box_maker = BRepPrimAPI_MakeBox(x, y, z)
        return box_maker.Shape()

    def transform_shape(self, shape: Any, translation: Tuple[float, float, float], 
                        axis_origin: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                        axis_dir: Tuple[float, float, float] = (0.0, 0.0, 1.0),
                        angle_deg: float = 0.0) -> Any:
        if shape is None:
            return None
            
        trsf = gp_Trsf()
        
        if angle_deg != 0.0:
            pnt = gp_Pnt(*axis_origin)
            dir = gp_Dir(*axis_dir)
            ax1 = gp_Ax1(pnt, dir)
            trsf.SetRotation(ax1, math.radians(angle_deg))
            
        if translation != (0.0, 0.0, 0.0):
            vec = gp_Vec(*translation)
            trsf_trans = gp_Trsf()
            trsf_trans.SetTranslation(vec)
            trsf.Multiply(trsf_trans)
            
        transform = BRepBuilderAPI_Transform(shape, trsf, False)
        return transform.Shape()

    def rotate_subtree(self, shape: Any, axis_origin: Tuple[float, float, float], 
                       axis_direction: Tuple[float, float, float], 
                       signed_angle_deg: float, moving_faces: Set[str]) -> Any:
        if shape is None:
            return None
            
        if not moving_faces:
            return self.transform_shape(shape, translation=(0,0,0), axis_origin=axis_origin, axis_dir=axis_direction, angle_deg=signed_angle_deg)
            
        # In OCC, rotating specific faces requires splitting the shape or identifying them.
        # Since this requires reconstructing the Compound, we iterate over faces.
        builder = BRep_Builder()
        compound = TopoDS_Shape()
        builder.MakeCompound(topods.Compound(compound)) # Correct way in pythonocc is slightly different
        
        # We will use a simpler approach for Phase 2 OCC prototype:
        # Rebuild a compound of faces, transforming only the moving ones.
        explorer = TopExp_Explorer(shape, TopAbs_FACE)
        face_idx = 0
        
        from OCC.Core.TopoDS import TopoDS_Compound
        comp = TopoDS_Compound()
        builder.MakeCompound(comp)
        
        while explorer.More():
            face = explorer.Current()
            if str(face_idx) in moving_faces:
                rot_face = self.transform_shape(face, translation=(0,0,0), axis_origin=axis_origin, axis_dir=axis_direction, angle_deg=signed_angle_deg)
                builder.Add(comp, rot_face)
            else:
                builder.Add(comp, face)
            explorer.Next()
            face_idx += 1
            
        return comp

    def intersect_volume(self, shape1: Any, shape2: Any) -> bool:
        if shape1 is None or shape2 is None:
            return False
            
        # Pre-filter using Bounding Boxes
        bb1 = self.get_bounding_box(shape1)
        bb2 = self.get_bounding_box(shape2)
        if not (bb1[0] <= bb2[3] and bb1[3] >= bb2[0] and
                bb1[1] <= bb2[4] and bb1[4] >= bb2[1] and
                bb1[2] <= bb2[5] and bb1[5] >= bb2[2]):
            return False
            
        common = BRepAlgoAPI_Common(shape1, shape2)
        common.Build()
        if not common.IsDone():
            return False
            
        result = common.Shape()
        
        # Check if the result has volume or area
        from OCC.Core.GProp import GProp_GProps
        from OCC.Core.BRepGProp import brepgprop
        props = GProp_GProps()
        brepgprop.VolumeProperties(result, props)
        return props.Mass() > 0.01

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
        if shape is None:
            return None
        from OCC.Core.TopoDS import TopoDS_Compound
        builder = BRep_Builder()
        comp = TopoDS_Compound()
        builder.MakeCompound(comp)
        
        explorer = TopExp_Explorer(shape, TopAbs_FACE)
        face_idx = 0
        while explorer.More():
            if str(face_idx) in face_ids:
                builder.Add(comp, explorer.Current())
            explorer.Next()
            face_idx += 1
        return comp

    def is_valid(self, shape: Any) -> bool:
        from OCC.Core.BRepCheck import BRepCheck_Analyzer
        if shape is None:
            return False
        analyzer = BRepCheck_Analyzer(shape)
        return analyzer.IsValid()

    def get_surface_type(self, face: Any) -> str:
        if face is None:
            return "Other"
        from OCC.Core.BRepAdaptor import BRepAdaptor_Surface
        from OCC.Core.GeomAbs import GeomAbs_Plane, GeomAbs_Cylinder
        try:
            surf = BRepAdaptor_Surface(face)
            typ = surf.GetType()
            if typ == GeomAbs_Plane:
                return "Plane"
            if typ == GeomAbs_Cylinder:
                return "Cylinder"
            return "Other"
        except Exception:
            return "Other"

    def get_face_normal(self, face: Any) -> Tuple[float, float, float]:
        if face is None:
            return (0.0, 0.0, 1.0)
        from OCC.Core.BRepAdaptor import BRepAdaptor_Surface
        from OCC.Core.BRepTools import breptools_UVBounds
        from OCC.Core.gp import gp_Dir
        try:
            u_min, u_max, v_min, v_max = breptools_UVBounds(face)
            u = (u_min + u_max) / 2.0
            v = (v_min + v_max) / 2.0
            surf = BRepAdaptor_Surface(face)
            p = gp_Pnt()
            d1u = gp_Vec()
            d1v = gp_Vec()
            surf.D1(u, v, p, d1u, d1v)
            normal = d1u.Crossed(d1v)
            if normal.Magnitude() > 1e-9:
                normal.Normalize()
                
            # Account for face orientation
            from OCC.Core.TopAbs import TopAbs_REVERSED
            if face.Orientation() == TopAbs_REVERSED:
                normal.Reverse()
                
            return (normal.X(), normal.Y(), normal.Z())
        except Exception:
            return (0.0, 0.0, 1.0)

    def get_cylinder_data(self, face: Any) -> Dict[str, Any]:
        if face is None:
            return {}
        from OCC.Core.BRepAdaptor import BRepAdaptor_Surface
        from OCC.Core.GeomAbs import GeomAbs_Cylinder
        try:
            surf = BRepAdaptor_Surface(face)
            if surf.GetType() == GeomAbs_Cylinder:
                cyl = surf.Cylinder()
                axis = cyl.Axis().Direction()
                loc = cyl.Location()
                
                # Approximate length using bounding box along axis
                bb = self.get_bounding_box(face)
                length = 100.0
                if bb:
                    dx, dy, dz = bb[3]-bb[0], bb[4]-bb[1], bb[5]-bb[2]
                    length = max(dx, dy, dz)
                
                return {
                    "radius": cyl.Radius(),
                    "axis": (axis.X(), axis.Y(), axis.Z()),
                    "origin": (loc.X(), loc.Y(), loc.Z()),
                    "length": length
                }
        except Exception:
            pass
        return {}

    def get_face_adjacencies(self, shape: Any) -> Dict[str, List[str]]:
        adj_map = {}
        if shape is None:
            return adj_map
            
        from OCC.Core.TopExp import TopExp_Explorer
        from OCC.Core.TopAbs import TopAbs_EDGE
        from OCC.Core.TopExp import topexp_MapShapesAndAncestors
        from OCC.Core.TopTools import TopTools_IndexedDataMapOfShapeListOfShape
        from OCC.Core.TopoDS import topods
        
        # Map Faces to ID
        face_list = self.get_faces(shape)
        face_to_id = {hash(f): str(i) for i, f in enumerate(face_list)}
        
        edge_face_map = TopTools_IndexedDataMapOfShapeListOfShape()
        topexp_MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edge_face_map)
        
        for i in range(1, edge_face_map.Extent() + 1):
            face_list = edge_face_map.FindFromIndex(i)
            face_ids = []
            for j in range(1, face_list.Extent() + 1):
                f = topods.Face(face_list.Value(j))
                f_hash = hash(f)
                if f_hash in face_to_id:
                    face_ids.append(face_to_id[f_hash])
                    
            if len(face_ids) == 2:
                f1, f2 = face_ids
                if f1 not in adj_map: adj_map[f1] = []
                if f2 not in adj_map: adj_map[f2] = []
                if f2 not in adj_map[f1]: adj_map[f1].append(f2)
                if f1 not in adj_map[f2]: adj_map[f2].append(f1)
                
        return adj_map
