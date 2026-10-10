"""
validator.py - Single Authoritative Physical Oracle for Step Validation (Headless Core)
"""
import logging
from typing import Dict, Any, Optional, Tuple
from smbs_core.Core.fold_state import FoldState
from smbs_core.Core.bend_graph import BendRecord
from smbs_core.Core.panel_graph import PanelGraph
from smbs_core.Core.bend_transform import BendTransform
from smbs_core.Physics.collision_detector import CollisionDetector
from smbs_core.Physics.machine_kinematics import PressBrakeMachine
from smbs_core.geometry_backend.base import GeometryBackend

logger = logging.getLogger(__name__)

class ValidationResult:
    def __init__(
        self,
        valid: bool,
        reason: str,
        stage: str,
        next_state: Optional[FoldState] = None,
        diagnostics: Optional[Dict[str, Any]] = None
    ):
        self.valid = valid
        self.reason = reason
        self.stage = stage
        self.next_state = next_state
        self.diagnostics = diagnostics or {}

    def __repr__(self) -> str:
        status = "VALID" if self.valid else f"INVALID[{self.stage}:{self.reason}]"
        return f"<ValidationResult {status}>"

class PhysicalValidator:
    def __init__(self, machine: PressBrakeMachine, backend: GeometryBackend):
        self.machine = machine
        self.backend = backend
        self.assembly = self.machine.setup_assembly(self.backend)
        self.detector = CollisionDetector(self.assembly, self.backend)
        self.invalid_transitions: Dict[Tuple[str, str], ValidationResult] = {}

    def validate_step(
        self,
        current_state: FoldState,
        bend_record: BendRecord,
        panel_graph: Optional[PanelGraph] = None,
        tooling_override: Optional[Dict[str, Any]] = None
    ) -> ValidationResult:
        bend_id = bend_record.id
        current_fp = current_state.fingerprint()
        
        cache_key = (current_fp, bend_id)
        if cache_key in self.invalid_transitions:
            return self.invalid_transitions[cache_key]

        if hasattr(self.detector, 'begin_step'):
            self.detector.begin_step()

        try:
            if bend_id not in current_state.remaining_bends:
                return ValidationResult(
                    valid=False,
                    reason=f"Bend {bend_id} is not in remaining_bends set.",
                    stage="consistency"
                )
                
            current_axis_origin, current_axis_dir = current_state.bend_axes.get(
                bend_id, 
                (bend_record.axis_origin, bend_record.axis_direction)
            )
                
            bend_info = bend_record.to_dict()
            bend_info["origin"] = current_axis_origin
            bend_info["axis"] = current_axis_dir
            
            app_res = self.detector.approach_collision(current_state.shape, bend_info)
            if not app_res.is_valid:
                logger.info(f"Reject [{bend_id}] stage=APPROACH: {app_res.message}")
                res = ValidationResult(valid=False, reason=app_res.message, stage="APPROACH")
                self.invalid_transitions[cache_key] = res
                return res

            moving_panels = panel_graph.moving_subtree(bend_id) if panel_graph else set()
            
            unfolded_shape = BendTransform.rotate_subtree(
                self.backend,
                current_state.shape,
                current_axis_origin,
                current_axis_dir,
                -bend_record.signed_angle,
                moving_panels,
                panel_graph
            )

            self_res = self.detector.self_collision(unfolded_shape, bend_id, panel_graph)
            if not self_res.is_valid:
                logger.info(f"Reject [{bend_id}] stage=SELF: {self_res.message}")
                res = ValidationResult(valid=False, reason=self_res.message, stage="SELF")
                self.invalid_transitions[cache_key] = res
                return res

            tool_res = self.detector.tool_collision(unfolded_shape, bend_info, bend_id, panel_graph)
            if not tool_res.is_valid:
                logger.info(f"Reject [{bend_id}] stage=TOOL: {tool_res.message}")
                res = ValidationResult(valid=False, reason=tool_res.message, stage="TOOL")
                self.invalid_transitions[cache_key] = res
                return res

            mach_res = self.detector.machine_collision(unfolded_shape, bend_info, bend_id, panel_graph)
            if not mach_res.is_valid:
                logger.info(f"Reject [{bend_id}] stage=MACHINE: {mach_res.message}")
                res = ValidationResult(valid=False, reason=mach_res.message, stage="MACHINE")
                self.invalid_transitions[cache_key] = res
                return res

            traj_res = self.detector.trajectory_collision(current_state.shape, bend_info, bend_id=bend_id, panel_graph=panel_graph, dtheta=5.0)
            if not traj_res.is_valid:
                logger.info(f"Reject [{bend_id}] stage=BEND_TRAJECTORY: {traj_res.message}")
                res = ValidationResult(valid=False, reason=traj_res.message, stage="BEND_TRAJECTORY")
                self.invalid_transitions[cache_key] = res
                return res

            ret_res = self.detector.retract_collision(unfolded_shape, bend_info)
            if not ret_res.is_valid:
                logger.info(f"Reject [{bend_id}] stage=RETRACT: {ret_res.message}")
                res = ValidationResult(valid=False, reason=ret_res.message, stage="RETRACT")
                self.invalid_transitions[cache_key] = res
                return res

            next_fold_state = current_state.apply_unfold(
                bend_id=bend_id,
                unfolded_shape=unfolded_shape,
                moving_panels=moving_panels
            )

            return ValidationResult(
                valid=True,
                reason="Step validated physically.",
                stage="valid",
                next_state=next_fold_state,
                diagnostics={"bend_id": bend_id, "signed_angle": bend_record.signed_angle}
            )
        finally:
            if hasattr(self.detector, 'end_step'):
                self.detector.end_step()
