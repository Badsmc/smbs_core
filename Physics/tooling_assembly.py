"""
tooling_assembly.py - Assembles tooling in machine coordinate space.
"""
from dataclasses import dataclass
from typing import Dict, Any, Optional, List
import math

from smbs_core.models.tooling import Tooling, MachineConfiguration, DatumRole, ToolingKind
from smbs_core.geometry_backend.base import GeometryBackend

try:
    from OCC.Core.gp import gp_Trsf, gp_Vec, gp_Ax3, gp_Pnt, gp_Dir
    from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform
except ImportError:
    pass

@dataclass
class AssemblyResult:
    status: str  # "ALIGNED", "MISALIGNED", "CANNOT_VERIFY"
    diagnostics: List[str]

class ToolingAssembly:
    def __init__(self, backend: GeometryBackend, machine: MachineConfiguration):
        self.backend = backend
        self.machine = machine
        self.punch: Optional[Tooling] = None
        self.die: Optional[Tooling] = None
        self.machine_frame: Optional[Tooling] = None
        self.assembly_trsf: Dict[str, Any] = {} # Dict[ToolingKind, gp_Trsf]
        
    def place(self, punch: Tooling, die: Tooling, machine_frame: Optional[Tooling] = None) -> AssemblyResult:
        """
        Positions punch, die, and machine frame in the machine coordinate system.
        """
        self.punch = punch
        self.die = die
        self.machine_frame = machine_frame
        
        diagnostics = []
        
        # We need datums to assemble
        if not self._check_datums(punch, [DatumRole.MOUNTING_PLANE, DatumRole.WORKING_AXIS, DatumRole.TIP_POINT]):
            diagnostics.append(f"Punch {punch.id} is missing required datums.")
            
        if not self._check_datums(die, [DatumRole.MOUNTING_PLANE, DatumRole.WORKING_AXIS, DatumRole.V_CENTER]):
            diagnostics.append(f"Die {die.id} is missing required datums.")
            
        if diagnostics:
            return AssemblyResult("CANNOT_VERIFY", diagnostics)
            
        # 1. Position Die
        # Align Die MOUNTING_PLANE to Machine bed_top_plane
        die_trsf = self._align_planes(
            source_datum=die.datums[DatumRole.MOUNTING_PLANE],
            target_datum=self.machine.bed_top_plane
        )
        # Apply additional constraint: align V_CENTER to World X-axis (assuming bed is aligned to World)
        # For a full implementation, we build a gp_Ax3 for the source and target and compute the transformation.
        
        # Using a simplified translation for the demonstration of the architecture
        die_z_offset = self.machine.bed_top_plane.geometry[0][2] - die.datums[DatumRole.MOUNTING_PLANE].geometry[0][2]
        die_trsf_simple = gp_Trsf()
        die_trsf_simple.SetTranslationPart(gp_Vec(0, 0, die_z_offset))
        self.assembly_trsf[ToolingKind.DIE] = die_trsf_simple
        
        # 2. Position Punch
        punch_z_offset = self.machine.ram_mount_plane.geometry[0][2] - punch.datums[DatumRole.MOUNTING_PLANE].geometry[0][2]
        punch_trsf_simple = gp_Trsf()
        punch_trsf_simple.SetTranslationPart(gp_Vec(0, 0, punch_z_offset))
        self.assembly_trsf[ToolingKind.PUNCH] = punch_trsf_simple
        
        # 3. Position Machine Frame
        if machine_frame:
            frame_trsf = gp_Trsf() # Identity, assuming machine frame is World
            self.assembly_trsf[ToolingKind.MACHINE_FRAME] = frame_trsf
            
        # 4. Validate Alignment
        # Calculate world position of Punch TIP_POINT
        punch_tip_local = punch.datums[DatumRole.TIP_POINT].geometry[0]
        punch_tip_pnt = gp_Pnt(*punch_tip_local).Transformed(punch_trsf_simple)
        
        # Calculate world position of Die V_CENTER
        die_vcenter_local = die.datums[DatumRole.V_CENTER].geometry[0]
        die_vcenter_pnt = gp_Pnt(*die_vcenter_local).Transformed(die_trsf_simple)
        
        # Check collinearity in X-Y plane
        dx = punch_tip_pnt.X() - die_vcenter_pnt.X()
        dy = punch_tip_pnt.Y() - die_vcenter_pnt.Y()
        
        tolerance = 0.5
        if abs(dx) > tolerance or abs(dy) > tolerance:
            diagnostics.append(f"Punch TIP and Die V_CENTER are misaligned by ({dx:.3f}, {dy:.3f})")
            return AssemblyResult("MISALIGNED", diagnostics)
            
        return AssemblyResult("ALIGNED", ["Tooling assembled successfully."])

    def _check_datums(self, tool: Tooling, required: List[DatumRole]) -> bool:
        for r in required:
            if r not in tool.datums:
                return False
        return True

    def _align_planes(self, source_datum, target_datum) -> Any:
        # Placeholder for full gp_Ax3 alignment
        return gp_Trsf()
        
    def get_world_shape(self, kind: ToolingKind) -> Any:
        if kind == ToolingKind.PUNCH and self.punch:
            trsf = self.assembly_trsf.get(ToolingKind.PUNCH, gp_Trsf())
            return BRepBuilderAPI_Transform(self.punch.shape, trsf, False).Shape()
        elif kind == ToolingKind.DIE and self.die:
            trsf = self.assembly_trsf.get(ToolingKind.DIE, gp_Trsf())
            return BRepBuilderAPI_Transform(self.die.shape, trsf, False).Shape()
        elif kind == ToolingKind.MACHINE_FRAME and self.machine_frame:
            trsf = self.assembly_trsf.get(ToolingKind.MACHINE_FRAME, gp_Trsf())
            return BRepBuilderAPI_Transform(self.machine_frame.shape, trsf, False).Shape()
        return None
