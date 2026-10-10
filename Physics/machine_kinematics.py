"""
machine_kinematics.py - Machine Kinematics and Real/Synthetic Tooling Setup Bridge
"""

from typing import Dict, Any, Optional
import os
import logging

from smbs_core.geometry_backend.base import GeometryBackend
from smbs_core.models.tooling import Tooling, ToolingKind, DatumRole, Datum, MachineConfiguration, GeometryStatus
from smbs_core.Physics.tooling_assembly import ToolingAssembly
from smbs_core.geometry_backend.backends.fcstd_loader import FcstdToolingLoader

logger = logging.getLogger(__name__)

try:
    from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox
    HAS_OCC = True
except ImportError:
    HAS_OCC = False


class PressBrakeMachine:
    def __init__(
        self,
        name: str = "press_100t",
        max_tonnage: float = 100.0,
        bed_length_mm: float = 2000.0,
        backgauge_x_max_mm: float = 800.0,
        backgauge_r_max_mm: float = 250.0,
        punch_path: Optional[str] = None,
        die_path: Optional[str] = None,
        filepath: Optional[str] = None
    ):
        self.name = name
        self.max_tonnage = max_tonnage
        self.bed_length_mm = bed_length_mm
        self.backgauge_x_max_mm = backgauge_x_max_mm
        self.backgauge_r_max_mm = backgauge_r_max_mm
        self.filepath = filepath

        self.punch_path = punch_path or "/Users/badmc/Desktop/SMBS/toollibrary/punch/puancon.FCStd"
        self.die_path = die_path or "/Users/badmc/Desktop/SMBS/toollibrary/die/matrica.FCStd"

        self.config = MachineConfiguration(
            id=name,
            bed_top_plane=Datum(DatumRole.MOUNTING_PLANE, "plane", ((0, 0, 0), (0, 0, 1)), source="sidecar", confirmed=True),
            ram_mount_plane=Datum(DatumRole.MOUNTING_PLANE, "plane", ((0, 0, 200), (0, 0, -1)), source="sidecar", confirmed=True),
            ram_stroke_mm=200.0,
            ram_direction=(0.0, 0.0, -1.0),
            backgauge_axes={"X": {"max": backgauge_x_max_mm}, "R": {"max": backgauge_r_max_mm}},
            max_tonnage_kn=max_tonnage * 10.0,
            max_bend_length_mm=bed_length_mm,
            source_file=filepath
        )

        self.assembly: Optional[ToolingAssembly] = None
        self.punch: Optional[Tooling] = None
        self.die: Optional[Tooling] = None

    @classmethod
    def default_setup(cls) -> "PressBrakeMachine":
        return cls()

    @classmethod
    def from_config(cls, config: Dict[str, Any]) -> "PressBrakeMachine":
        return cls(
            name=config.get("name", "press_100t"),
            bed_length_mm=config.get("bed_length_mm", 2000.0),
            punch_path=config.get("punch_path"),
            die_path=config.get("die_path"),
            filepath=config.get("source_file")
        )

    def setup_assembly(self, backend: GeometryBackend) -> ToolingAssembly:
        self.assembly = ToolingAssembly(backend, self.config)
        loader = FcstdToolingLoader()

        if os.path.exists(self.punch_path):
            self.punch = loader.load(self.punch_path, ToolingKind.PUNCH)
        else:
            shape_p = BRepPrimAPI_MakeBox(30, 1000, 120).Shape() if HAS_OCC else None
            self.punch = Tooling(
                id="SyntheticPunch", kind=ToolingKind.PUNCH, source_file="",
                shape=shape_p, local_frame=None, geometry_status=GeometryStatus.SIMPLIFIED,
                diagnostics=["Punch FCStd file not found. Fallback to synthetic."]
            )
            self.punch.datums[DatumRole.MOUNTING_PLANE] = Datum(DatumRole.MOUNTING_PLANE, "plane", ((0, 0, 120), (0, 0, 1)), source="inferred", confirmed=False)
            self.punch.datums[DatumRole.TIP_POINT] = Datum(DatumRole.TIP_POINT, "point", ((0, 0, 0),), source="inferred", confirmed=False)
            self.punch.datums[DatumRole.WORKING_AXIS] = Datum(DatumRole.WORKING_AXIS, "axis", ((0, 0, 0), (1, 0, 0)), source="inferred", confirmed=False)

        if os.path.exists(self.die_path):
            self.die = loader.load(self.die_path, ToolingKind.DIE)
        else:
            shape_d = BRepPrimAPI_MakeBox(50, 1000, 80).Shape() if HAS_OCC else None
            self.die = Tooling(
                id="SyntheticDie", kind=ToolingKind.DIE, source_file="",
                shape=shape_d, local_frame=None, geometry_status=GeometryStatus.SIMPLIFIED,
                diagnostics=["Die FCStd file not found. Fallback to synthetic."]
            )
            self.die.datums[DatumRole.MOUNTING_PLANE] = Datum(DatumRole.MOUNTING_PLANE, "plane", ((0, 0, 0), (0, 0, -1)), source="inferred", confirmed=False)
            self.die.datums[DatumRole.V_CENTER] = Datum(DatumRole.V_CENTER, "axis", ((0, 0, 80), (1, 0, 0)), source="inferred", confirmed=False)
            self.die.datums[DatumRole.WORKING_AXIS] = Datum(DatumRole.WORKING_AXIS, "axis", ((0, 0, 80), (1, 0, 0)), source="inferred", confirmed=False)

        self.assembly.place(self.punch, self.die)
        return self.assembly
