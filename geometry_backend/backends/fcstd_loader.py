"""
fcstd_loader.py - Loads Tooling objects from .FCStd files and .tooling.json sidecars.
"""

import os
import json
from typing import Optional

from smbs_core.models.tooling import Tooling, ToolingKind, DatumRole, Datum, GeometryStatus
from smbs_core.geometry_backend.backends.fcstd_reader import read_fcstd

try:
    from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform
    from OCC.Core.gp import gp_Trsf
    from OCC.Core.Bnd import Bnd_Box
    from OCC.Core.BRepBndLib import brepbndlib
    HAS_OCC = True
except ImportError:
    HAS_OCC = False


class FcstdToolingLoader:
    """Loads Tooling from .FCStd and sidecar .tooling.json"""

    def load(self, path: str, kind: ToolingKind) -> Tooling:
        tool_id = os.path.basename(path).replace(".FCStd", "")
        
        # 1. Read geometry
        results, geo_status = read_fcstd(path)
        
        if geo_status == GeometryStatus.MISSING or not results:
            # Return Tooling with MISSING geometry
            return Tooling(
                id=tool_id,
                kind=kind,
                source_file=path,
                shape=None,
                local_frame=gp_Trsf() if HAS_OCC else None,
                geometry_status=GeometryStatus.MISSING,
                datums={},
                metadata={},
                diagnostics=[f"No valid B-Rep shape extracted from {path}"]
            )

        # Prefer object named Pad or Body or last valid shape
        pad_objs = [r for r in results if r[0] in ("Pad", "Body")]
        obj_name, shape, trsf = pad_objs[-1] if pad_objs else results[-1]

        world_shape = shape
        if HAS_OCC and trsf:
            world_shape = BRepBuilderAPI_Transform(shape, trsf, False).Shape()

        tool = Tooling(
            id=tool_id,
            kind=kind,
            source_file=path,
            shape=world_shape,
            local_frame=trsf,
            geometry_status=GeometryStatus.REAL,
            datums={},
            metadata={},
            diagnostics=[]
        )

        # 2. Check for sidecar JSON
        sidecar_path = path.replace(".FCStd", ".tooling.json")
        if os.path.exists(sidecar_path):
            self._load_sidecar(sidecar_path, tool)
        else:
            tool.geometry_status = GeometryStatus.SIMPLIFIED
            tool.diagnostics.append("Sidecar tooling.json missing. Datums inferred heuristically.")
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
                    tool.datums[role] = Datum.from_json(role, datum_data, source="sidecar")
                except (ValueError, KeyError) as e:
                    tool.diagnostics.append(f"Failed to parse datum {role_str}: {e}")

    def _infer_datums(self, tool: Tooling):
        if not HAS_OCC or tool.shape is None:
            return

        bbox = Bnd_Box()
        brepbndlib.Add(tool.shape, bbox, False)
        xmin, ymin, zmin, xmax, ymax, zmax = bbox.Get()

        x_c = (xmin + xmax) / 2.0
        y_c = (ymin + ymax) / 2.0

        if tool.kind == ToolingKind.PUNCH:
            tool.datums[DatumRole.MOUNTING_PLANE] = Datum(
                role=DatumRole.MOUNTING_PLANE, kind="plane",
                geometry=((x_c, y_c, zmax), (0, 0, 1)),
                source="inferred", confirmed=False
            )
            tool.datums[DatumRole.TIP_POINT] = Datum(
                role=DatumRole.TIP_POINT, kind="point",
                geometry=((x_c, y_c, zmin),),
                source="inferred", confirmed=False
            )
            tool.datums[DatumRole.WORKING_AXIS] = Datum(
                role=DatumRole.WORKING_AXIS, kind="axis",
                geometry=((x_c, y_c, zmin), (1, 0, 0)),
                source="inferred", confirmed=False
            )

        elif tool.kind == ToolingKind.DIE:
            tool.datums[DatumRole.MOUNTING_PLANE] = Datum(
                role=DatumRole.MOUNTING_PLANE, kind="plane",
                geometry=((x_c, y_c, zmin), (0, 0, -1)),
                source="inferred", confirmed=False
            )
            tool.datums[DatumRole.V_CENTER] = Datum(
                role=DatumRole.V_CENTER, kind="axis",
                geometry=((x_c, y_c, zmax), (1, 0, 0)),
                source="inferred", confirmed=False
            )
            tool.datums[DatumRole.WORKING_AXIS] = Datum(
                role=DatumRole.WORKING_AXIS, kind="axis",
                geometry=((x_c, y_c, zmax), (1, 0, 0)),
                source="inferred", confirmed=False
            )
