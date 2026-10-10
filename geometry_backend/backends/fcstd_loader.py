"""
fcstd_loader.py - Loads Tooling objects from .FCStd files + sidecar .json datums.
"""
import os
import json
from typing import Dict, Any, Optional

from smbs_core.models.tooling import Tooling, ToolingKind, DatumRole, Datum
from smbs_core.geometry_backend.backends.fcstd_reader import read_fcstd

try:
    from OCC.Core.TopoDS import TopoDS_Shape
    from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform
    from OCC.Core.gp import gp_Trsf, gp_Vec, gp_Ax1, gp_Pnt, gp_Dir
except ImportError:
    pass

class FcstdToolingLoader:
    """Loads Tooling from .FCStd and sidecar .tooling.json"""
    
    def load(self, path: str, kind: ToolingKind) -> Tooling:
        if not os.path.exists(path):
            raise FileNotFoundError(f"FCStd file not found: {path}")
            
        # 1. Read geometry
        results = read_fcstd(path)
        if not results:
            raise ValueError(f"No BRep geometry found in {path}")
            
        # For simplicity, we just take the first valid shape. 
        # A more robust implementation might combine all shapes into a Compound.
        obj_name, shape, trsf = results[-1]  # Typically the last one is the final result (e.g. Pad)
        
        # We need to apply the placement to the shape to get it in world space (of the tool)
        transform = BRepBuilderAPI_Transform(shape, trsf, False)
        world_shape = transform.Shape()
        
        tool_id = os.path.basename(path).replace(".FCStd", "")
        tool = Tooling(
            id=tool_id,
            kind=kind,
            source_file=path,
            shape=world_shape,
            local_frame=trsf,
            datums={},
            metadata={},
            geometry_status="REAL"
        )
        
        # 2. Check for sidecar JSON
        sidecar_path = path.replace(".FCStd", ".tooling.json")
        if os.path.exists(sidecar_path):
            self._load_sidecar(sidecar_path, tool)
        else:
            # Fallback heuristic
            tool.geometry_status = "SIMPLIFIED"
            tool.metadata["warning"] = "Sidecar tooling.json missing. Datums inferred."
            self._infer_datums(tool)
            
        return tool
        
    def _load_sidecar(self, path: str, tool: Tooling):
        with open(path, 'r', encoding='utf-8') as f:
            data = json.load(f)
            
        if "id" in data:
            tool.id = data["id"]
        if "metadata" in data:
            tool.metadata = data["metadata"]
            
        if "datums" in data:
            for role_str, datum_data in data["datums"].items():
                try:
                    role = DatumRole(role_str)
                    d_kind = datum_data.get("kind", "point")
                    tol = datum_data.get("tolerance_mm", 0.05)
                    
                    if d_kind == "point":
                        pos = tuple(datum_data.get("position", [0,0,0]))
                        geom = (pos,)
                    elif d_kind == "axis":
                        orig = tuple(datum_data.get("origin", [0,0,0]))
                        dir = tuple(datum_data.get("direction", [1,0,0]))
                        geom = (orig, dir)
                    elif d_kind == "plane":
                        orig = tuple(datum_data.get("origin", [0,0,0]))
                        norm = tuple(datum_data.get("normal", [0,0,1]))
                        geom = (orig, norm)
                    else:
                        geom = tuple()
                        
                    tool.datums[role] = Datum(
                        role=role,
                        kind=d_kind,
                        geometry=geom,
                        tolerance_mm=tol
                    )
                except ValueError:
                    pass # Ignore unknown datum roles
                    
    def _infer_datums(self, tool: Tooling):
        # Extremely naive bounding box based datums for demonstration.
        # Real logic would extract top/bottom faces.
        from OCC.Core.Bnd import Bnd_Box
        from OCC.Core.BRepBndLib import brepbndlib
        
        bbox = Bnd_Box()
        brepbndlib.Add(tool.shape, bbox)
        xmin, ymin, zmin, xmax, ymax, zmax = bbox.Get()
        
        x_c = (xmin + xmax) / 2.0
        y_c = (ymin + ymax) / 2.0
        
        if tool.kind == ToolingKind.PUNCH:
            tool.datums[DatumRole.MOUNTING_PLANE] = Datum(
                role=DatumRole.MOUNTING_PLANE, kind="plane", 
                geometry=((x_c, y_c, zmax), (0, 0, 1))
            )
            tool.datums[DatumRole.TIP_POINT] = Datum(
                role=DatumRole.TIP_POINT, kind="point", 
                geometry=((x_c, y_c, zmin),)
            )
            tool.datums[DatumRole.WORKING_AXIS] = Datum(
                role=DatumRole.WORKING_AXIS, kind="axis", 
                geometry=((x_c, y_c, zmin), (1, 0, 0)) # Assume X axis is bend length
            )
            
        elif tool.kind == ToolingKind.DIE:
            tool.datums[DatumRole.MOUNTING_PLANE] = Datum(
                role=DatumRole.MOUNTING_PLANE, kind="plane", 
                geometry=((x_c, y_c, zmin), (0, 0, -1))
            )
            tool.datums[DatumRole.V_CENTER] = Datum(
                role=DatumRole.V_CENTER, kind="axis", 
                geometry=((x_c, y_c, zmax), (1, 0, 0))
            )
