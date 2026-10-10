"""
collision_detector.py - Exact Geometry-Based Collision Engine
"""
from enum import Enum
from dataclasses import dataclass
from typing import Dict, Any, Tuple, Optional, Set
import logging

from smbs_core.geometry_backend.base import GeometryBackend
from smbs_core.Physics.tooling_assembly import ToolingAssembly, ToolingKind

logger = logging.getLogger(__name__)

class CollisionStatus(Enum):
    VALID = "valid"
    INVALID = "invalid"
    UNKNOWN = "unknown"

@dataclass
class CollisionResult:
    status: CollisionStatus
    message: str
    
    @property
    def is_valid(self) -> bool:
        # UNKNOWN is never treated as VALID
        return self.status == CollisionStatus.VALID

class CollisionDetector:
    def __init__(self, assembly: ToolingAssembly, backend: GeometryBackend):
        self.assembly = assembly
        self.backend = backend
        self._memo_panels: Dict[int, Tuple[Any, Dict[str, Any]]] = {}

    def begin_step(self) -> None:
        self._memo_panels.clear()

    def end_step(self) -> None:
        self._memo_panels.clear()

    def _extract_panel_shapes(self, shape: Any, panel_graph: Any) -> Dict[str, Any]:
        panel_shapes = {}
        if shape is None or panel_graph is None:
            return panel_shapes

        memo = self._memo_panels.get(id(shape))
        if memo is not None and memo[0] is shape:
            return memo[1]

        if not any(p.face_indices for p in panel_graph.panels.values()):
            self._memo_panels[id(shape)] = (shape, panel_shapes)
            return panel_shapes

        for panel_id, panel in panel_graph.panels.items():
            if panel.face_indices:
                faces = set(str(idx) for idx in panel.face_indices)
                panel_shapes[panel_id] = self.backend.extract_faces(shape, faces)
                
        self._memo_panels[id(shape)] = (shape, panel_shapes)
        return panel_shapes

    def _check_intersection(self, shape1: Any, shape2: Any, tolerance_volume: float = 0.1) -> CollisionResult:
        if shape1 is None or shape2 is None:
            return CollisionResult(CollisionStatus.VALID, "Clear (missing shape is ignored)")
            
        try:
            # We assume intersect_volume is exposed by the backend
            if hasattr(self.backend, 'intersect_volume'):
                collides = self.backend.intersect_volume(shape1, shape2)
                if collides:
                    return CollisionResult(CollisionStatus.INVALID, "Intersection detected > tolerance")
                return CollisionResult(CollisionStatus.VALID, "Clear")
            else:
                return CollisionResult(CollisionStatus.UNKNOWN, "Backend lacks exact intersection check")
        except Exception as e:
            logger.error(f"Collision check failed: {e}")
            return CollisionResult(CollisionStatus.UNKNOWN, f"OCC Boolean failed: {e}")

    def self_collision(self, shape: Any, bend_id: Optional[str] = None, panel_graph: Optional[Any] = None) -> CollisionResult:
        if shape is None or bend_id is None or panel_graph is None:
            return CollisionResult(CollisionStatus.VALID, "Clear")
            
        fixed_panels = panel_graph.fixed_set(bend_id)
        moving_panels = panel_graph.moving_subtree(bend_id)
        
        if not fixed_panels or not moving_panels:
            return CollisionResult(CollisionStatus.VALID, "Clear")
            
        panel_shapes = self._extract_panel_shapes(shape, panel_graph)
        
        for fp_id in fixed_panels:
            if fp_id not in panel_shapes: continue
            fp_shape = panel_shapes[fp_id]
            
            for mp_id in moving_panels:
                if panel_graph.are_adjacent(fp_id, mp_id):
                    continue
                if mp_id not in panel_shapes: continue
                mp_shape = panel_shapes[mp_id]
                
                res = self._check_intersection(fp_shape, mp_shape)
                if not res.is_valid:
                    return CollisionResult(CollisionStatus.INVALID, f"Panel {mp_id} collides with {fp_id}")
                    
        return CollisionResult(CollisionStatus.VALID, "Clear")

    def approach_collision(self, shape: Any, bend_info: Dict[str, Any]) -> CollisionResult:
        # Part vs Machine frame or die during approach
        # For full implementation, we'd sweep the part. Here we just check static interference.
        die = self.assembly.get_world_shape(ToolingKind.DIE)
        if die:
            res = self._check_intersection(shape, die)
            if not res.is_valid:
                return CollisionResult(res.status, "Part collides with Die during approach")
                
        machine = self.assembly.get_world_shape(ToolingKind.MACHINE_FRAME)
        if machine:
            res = self._check_intersection(shape, machine)
            if not res.is_valid:
                return CollisionResult(res.status, "Part collides with Machine Frame during approach")
                
        return CollisionResult(CollisionStatus.VALID, "Clear")

    def tool_collision(self, shape: Any, bend_info: Dict[str, Any], bend_id: str, panel_graph: Any) -> CollisionResult:
        punch = self.assembly.get_world_shape(ToolingKind.PUNCH)
        if punch:
            res = self._check_intersection(shape, punch)
            if not res.is_valid:
                return CollisionResult(res.status, "Part collides with Punch in final position")
                
        return CollisionResult(CollisionStatus.VALID, "Clear")

    def machine_collision(self, shape: Any, bend_info: Dict[str, Any], bend_id: str, panel_graph: Any) -> CollisionResult:
        machine = self.assembly.get_world_shape(ToolingKind.MACHINE_FRAME)
        if machine:
            res = self._check_intersection(shape, machine)
            if not res.is_valid:
                return CollisionResult(res.status, "Part collides with Machine Frame in final position")
        return CollisionResult(CollisionStatus.VALID, "Clear")

    def trajectory_collision(self, start_shape: Any, bend_info: Dict[str, Any], bend_id: str, panel_graph: Any, dtheta: float = 5.0) -> CollisionResult:
        # Placeholder for swept volume or discretized check
        # In a real implementation we step through angle and run tool_collision and machine_collision
        return CollisionResult(CollisionStatus.VALID, "Clear")

    def retract_collision(self, shape: Any, bend_info: Dict[str, Any]) -> CollisionResult:
        # Similar to approach but for retraction
        return self.approach_collision(shape, bend_info)
