"""
orchestrator.py - Central Coordination Loop for Backward Bend Sequence Planning (Headless Core)
"""
import time
import os
import logging
from typing import Dict, Any, Optional, List

from .state_cache import StateCache
from .backward_graph import BackwardGraph, BendState
from smbs_core.Geometry.feature_extractor import FeatureExtractor
from smbs_core.Physics.collision_detector import CollisionDetector
from smbs_core.Physics.machine_kinematics import PressBrakeMachine
from smbs_core.Physics.backgauge_calculator import BackgaugeCalculator
from smbs_core.Solvers.astar_backward import AStarBackwardPlanner
from smbs_core.Solvers.greedy_unfold import GreedyUnfoldPlanner
from smbs_core.Solvers.tool_optimizer import ToolOptimizer
from smbs_core.IO.bendseq_writer import BendSeqWriter
from smbs_core.IO.tool_database import ToolDatabase
from smbs_core.geometry_backend.base import GeometryBackend
from smbs_core.Physics.validator import PhysicalValidator
from smbs_core.Solvers.dfs_backward import DFSBackwardPlanner
from smbs_core.Core.bend_graph import BendGraph, BendRecord
from smbs_core.Core.panel_graph import PanelGraph
from smbs_core.Core.fold_state import FoldState

logger = logging.getLogger(__name__)

class BendSeqOrchestrator:
    def __init__(self, backend: GeometryBackend, machine_config: Optional[Dict[str, Any]] = None):
        self.backend = backend
        self.state_cache = StateCache()
        self.machine = PressBrakeMachine.from_config(machine_config) if machine_config else PressBrakeMachine()
        self.collision_detector = CollisionDetector(self.machine, self.backend)
        self.extractor = FeatureExtractor(self.backend)
        self.backgauge_calculator = BackgaugeCalculator(self.machine)
        self.tool_database = ToolDatabase()
        self.tool_optimizer = ToolOptimizer(self.tool_database)

    def run_planning(
        self,
        target_shape: Any,
        algorithm: str = "astar",
        output_file: str = "part_bendseq.json",
        max_iterations: int = 5000
    ) -> Dict[str, Any]:
        start_time = time.time()

        if output_file == "part_bendseq.json":
            output_file = os.path.expanduser("~/part_bendseq.json")

        features = self.extractor.extract_features(target_shape)
        if not features or "bends" not in features or not features["bends"]:
            return {
                "success": False,
                "error": "No valid sheet metal bend features detected in model shape."
            }

        bends_map = features["bends"]
        k_factor = features.get("k_factor", 0.40)
        thickness = features.get("thickness", 2.0)
        
        logger.info(f"extracted {len(bends_map)} bends, thickness={thickness}")

        bg = BendGraph()
        pg = PanelGraph()
        for idx, (b_id, b_info) in enumerate(bends_map.items()):
            parent_p = b_info.get("parent_panel_id", f"PANEL_{idx:02d}")
            child_p = b_info.get("child_panel_id", f"PANEL_{(idx+1):02d}")
            rec = BendRecord(
                bend_id=b_id,
                feature_id=f"FEAT_{b_id}",
                parent_panel_id=parent_p,
                child_panel_id=child_p,
                axis_origin=tuple(b_info.get("origin", (0.0, 0.0, 0.0))),
                axis_direction=tuple(b_info.get("axis", (1.0, 0.0, 0.0))),
                signed_angle=b_info.get("angle", 90.0),
                radius=b_info.get("radius", 2.0),
                thickness=thickness,
                length=b_info.get("length", 100.0)
            )
            bg.add_bend(rec)
            pg.add_bend_connection(b_id, rec.parent_panel_id, rec.child_panel_id)

        root_fold_state = FoldState.initial(target_shape, bg, pg)
        validator = PhysicalValidator(self.machine, self.backend)

        alg = algorithm.lower()
        if alg == "dfs":
            planner = DFSBackwardPlanner(bg, validator, pg)
        elif alg == "greedy":
            planner = GreedyUnfoldPlanner(bg, validator, pg)
        else:
            planner = AStarBackwardPlanner(bg, validator, pg, state_cache=self.state_cache)

        if alg == "astar":
            planning_result = planner.solve(root_fold_state, max_iterations=max_iterations)
        else:
            planning_result = planner.solve(root_fold_state)

        if not planning_result.get("success"):
            rejections = planning_result.get("rejections", [])
            reasons = [f"{r['bend_id']}:{r['stage']}({r['reason']})" for r in rejections] if rejections else []
            err_msg = f"{algorithm.upper()} planning failed: {', '.join(reasons)}" if reasons else planning_result.get("error", "Solver failed to find a valid unfold path.")
            return {
                "success": False,
                "error": err_msg,
                "planning_time": time.time() - start_time,
                "cache_stats": self.state_cache.stats
            }

        forward_sequence = planning_result.get("sequence", [])

        detailed_sequence = []
        for step in forward_sequence:
            bend_id = step["bend_id"]
            bend_info = bends_map[bend_id]

            bg_data = self.backgauge_calculator.compute_step_backgauge(
                step_number=step["step_number"],
                bend_info=bend_info,
                part_thickness=thickness
            )

            tooling = self.tool_database.select_tooling_for_bend(bend_info, thickness)

            step_entry = {
                "step": step["step_number"],
                "bend_id": bend_id,
                "status": "ok",
                "angle_deg": step.get("angle", bend_info.get("angle", 90.0)),
                "radius_mm": step.get("radius", bend_info.get("radius", 1.0)),
                "length_mm": step.get("length", bend_info.get("length", 100.0)),
                "axis": step.get("axis", bend_info.get("axis", (0.0, 0.0, 1.0))),
                "origin": step.get("origin", bend_info.get("origin", (0.0, 0.0, 0.0))),
                "moving_panels": step.get("moving_panels", []),
                "backgauge": bg_data,
                "tooling": tooling
            }
            detailed_sequence.append(step_entry)

        optimized_sequence = self.tool_optimizer.optimize_sequence_tooling(detailed_sequence)

        writer = BendSeqWriter()
        written_path = writer.write_bendseq(
            filename=output_file,
            metadata={
                "part_thickness_mm": thickness,
                "k_factor": k_factor,
                "material": features.get("material", "Steel_S235"),
                "total_bends": len(bends_map),
                "algorithm_used": algorithm,
                "planning_time_sec": round(time.time() - start_time, 3)
            },
            sequence=optimized_sequence
        )

        total_time = time.time() - start_time

        return {
            "success": True,
            "sequence": optimized_sequence,
            "bendseq_file": written_path,
            "planning_time": total_time,
            "nodes_explored": planning_result.get("nodes_explored", 0),
            "cache_stats": self.state_cache.stats
        }
