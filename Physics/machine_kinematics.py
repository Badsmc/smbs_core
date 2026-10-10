"""
machine_kinematics.py - Machine Kinematics and Tooling Assembly Bridge
"""
from typing import Dict, Any, Optional, Tuple
import math

from smbs_core.geometry_backend.base import GeometryBackend
from smbs_core.models.tooling import Tooling, ToolingKind, DatumRole, Datum, MachineConfiguration
from smbs_core.Physics.tooling_assembly import ToolingAssembly

try:
    from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform
    from OCC.Core.gp import gp_Trsf, gp_Vec
    HAS_OCC = True
except ImportError:
    HAS_OCC = False

class DummyProfile:
    def __init__(self, **kwargs):
        for k, v in kwargs.items():
            setattr(self, k, v)

class PressBrakeMachine:
    """
    Bridge class that maintains backward compatibility with the orchestrator,
    while utilizing the new ToolingAssembly and exact collision logic.
    """
    def __init__(
        self,
        name: str = "Standard_PressBrake_100T",
        max_tonnage: float = 100.0,
        bed_length_mm: float = 2000.0,
        backgauge_x_max_mm: float = 800.0,
        backgauge_r_max_mm: float = 250.0,
        punch: Optional[Any] = None,
        die: Optional[Any] = None,
        filepath: Optional[str] = None
    ):
        self.punch = punch or DummyProfile(height_mm=120.0, width_mm=30.0, length_mm=1000.0, tip_radius_mm=1.0, tip_angle_deg=88.0, name="PUNCH_GOOSENECK_120")
        self.die = die or DummyProfile(v_width_mm=12.0, height_mm=80.0, width_mm=50.0, length_mm=1000.0, v_angle_deg=88.0, shoulder_radius_mm=1.5, name="DIE_V12")
        self.name = name
        self.max_tonnage = max_tonnage
        self.bed_length_mm = bed_length_mm
        self.backgauge_x_max_mm = backgauge_x_max_mm
        self.backgauge_r_max_mm = backgauge_r_max_mm
        self.filepath = filepath
        
        self.config = MachineConfiguration(
            id=name,
            bed_top_plane=Datum(DatumRole.MOUNTING_PLANE, "plane", ((0,0,0), (0,0,1))),
            ram_mount_plane=Datum(DatumRole.MOUNTING_PLANE, "plane", ((0,0,300), (0,0,-1))),
            ram_stroke_mm=200.0,
            ram_direction=(0,0,-1),
            backgauge_axes={"X": {"max": backgauge_x_max_mm}, "R": {"max": backgauge_r_max_mm}},
            max_tonnage_kn=max_tonnage * 10,
            max_bend_length_mm=bed_length_mm
        )
        
        self.assembly = None  # Will be populated when backend is available

    @classmethod
    def from_config(cls, config: Dict[str, Any]) -> "PressBrakeMachine":
        return cls(
            name=config.get("name", "CustomPress"),
            bed_length_mm=config.get("bed_length_mm", 2000.0)
        )
        
    def setup_assembly(self, backend: GeometryBackend) -> ToolingAssembly:
        """
        Creates synthetic tools if needed and assembles them.
        Called by orchestrator or physical validator to initialize exact collision geometry.
        """
        self.assembly = ToolingAssembly(backend, self.config)
        
        from smbs_core.geometry_backend.backends.fcstd_loader import FcstdToolingLoader
        import os
        
        loader = FcstdToolingLoader()
        punch_path = "/Users/badmc/Desktop/SMBS/toollibrary/punch/puancon.FCStd"
        die_path = "/Users/badmc/Desktop/SMBS/toollibrary/die/matrica.FCStd"
        
        if HAS_OCC and os.path.exists(punch_path):
            punch_tool = loader.load(punch_path, ToolingKind.PUNCH)
        else:
            shape_p = BRepPrimAPI_MakeBox(30, 1000, 120).Shape() if HAS_OCC else None
            punch_tool = Tooling(
                id="SyntheticPunch", kind=ToolingKind.PUNCH, source_file="",
                shape=shape_p, local_frame=None, geometry_status="SIMPLIFIED"
            )
            punch_tool.datums[DatumRole.MOUNTING_PLANE] = Datum(DatumRole.MOUNTING_PLANE, "plane", ((0,0,120), (0,0,1)))
            punch_tool.datums[DatumRole.TIP_POINT] = Datum(DatumRole.TIP_POINT, "point", ((0,0,0),))
            punch_tool.datums[DatumRole.WORKING_AXIS] = Datum(DatumRole.WORKING_AXIS, "axis", ((0,0,0), (1,0,0)))

        if HAS_OCC and os.path.exists(die_path):
            die_tool = loader.load(die_path, ToolingKind.DIE)
        else:
            shape_d = BRepPrimAPI_MakeBox(50, 1000, 80).Shape() if HAS_OCC else None
            die_tool = Tooling(
                id="SyntheticDie", kind=ToolingKind.DIE, source_file="",
                shape=shape_d, local_frame=None, geometry_status="SIMPLIFIED"
            )
            die_tool.datums[DatumRole.MOUNTING_PLANE] = Datum(DatumRole.MOUNTING_PLANE, "plane", ((0,0,0), (0,0,-1)))
            die_tool.datums[DatumRole.V_CENTER] = Datum(DatumRole.V_CENTER, "point", ((0,0,80),))
            die_tool.datums[DatumRole.WORKING_AXIS] = Datum(DatumRole.WORKING_AXIS, "axis", ((0,0,80), (1,0,0)))

        self.assembly.place(punch_tool, die_tool)
        return self.assembly
