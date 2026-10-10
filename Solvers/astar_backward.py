"""
astar_backward.py - A* Backward Search Planner with State Cache Integration
"""

import time
import heapq
import logging
from typing import Dict, Any, List, Optional, Set, Tuple

from smbs_core.Core.fold_state import FoldState
from smbs_core.Core.bend_graph import BendGraph
from smbs_core.Core.panel_graph import PanelGraph
from smbs_core.Core.state_cache import StateCache
from smbs_core.Physics.validator import PhysicalValidator

logger = logging.getLogger(__name__)


class PriorityQueueNode:
    def __init__(self, f_score: float, state: FoldState, path: List[Dict[str, Any]]):
        self.f_score = f_score
        self.state = state
        self.path = path

    def __lt__(self, other: "PriorityQueueNode") -> bool:
        return self.f_score < other.f_score


class AStarBackwardPlanner:
    def __init__(
        self,
        bend_graph: BendGraph,
        validator: PhysicalValidator,
        panel_graph: Optional[PanelGraph] = None,
        state_cache: Optional[StateCache] = None
    ):
        self.bend_graph = bend_graph
        self.validator = validator
        self.panel_graph = panel_graph
        self.cache = state_cache or StateCache()
        self.nodes_explored = 0

    def solve(self, initial_state: FoldState, max_iterations: int = 5000) -> Dict[str, Any]:
        start_time = time.time()
        self.nodes_explored = 0
        rejections = []

        open_set: List[PriorityQueueNode] = []
        start_fp = initial_state.fingerprint()
        
        # Consult state cache
        cached_entry = self.cache.get(start_fp)
        if not cached_entry:
            self.cache.put(start_fp, {"visited": True})

        heapq.heappush(open_set, PriorityQueueNode(0.0, initial_state, []))

        g_scores: Dict[str, float] = {start_fp: 0.0}

        while open_set and self.nodes_explored < max_iterations:
            current_node = heapq.heappop(open_set)
            current_state = current_node.state
            current_path = current_node.path
            current_fp = current_state.fingerprint()

            self.nodes_explored += 1

            if current_state.is_goal():
                return {
                    "success": True,
                    "sequence": current_path,
                    "nodes_explored": self.nodes_explored,
                    "planning_time": time.time() - start_time,
                    "rejections": rejections
                }

            current_g = g_scores.get(current_fp, float('inf'))

            for bend_id in list(current_state.remaining_bends):
                bend_rec = self.bend_graph.bends[bend_id]

                val_res = self.validator.validate_step(
                    current_state=current_state,
                    bend_record=bend_rec,
                    panel_graph=self.panel_graph
                )

                if not val_res.valid:
                    rejections.append({
                        "bend_id": bend_id,
                        "stage": val_res.stage,
                        "reason": val_res.reason,
                        "collision_status": val_res.collision_status.value,
                        "tooling_status": val_res.tooling_status.value,
                        "alignment_status": val_res.alignment_status.value
                    })
                    continue

                next_state = val_res.next_state
                if next_state is None:
                    continue

                next_fp = next_state.fingerprint()
                tentative_g = current_g + 1.0

                if tentative_g < g_scores.get(next_fp, float('inf')):
                    g_scores[next_fp] = tentative_g
                    
                    # Check cache for successor state
                    cache_check = self.cache.get(next_fp)
                    if not cache_check:
                        self.cache.put(next_fp, {"visited": True})

                    h_score = len(next_state.remaining_bends)  # Admissible heuristic
                    f_score = tentative_g + h_score

                    step_info = {
                        "step_number": len(current_path) + 1,
                        "bend_id": bend_id,
                        "angle": bend_rec.signed_angle,
                        "radius": bend_rec.radius,
                        "length": bend_rec.length,
                        "axis": bend_rec.axis_direction,
                        "origin": bend_rec.axis_origin,
                        "moving_panels": list(self.panel_graph.moving_subtree(bend_id)) if self.panel_graph else [],
                        "collision_status": val_res.collision_status.value,
                        "tooling_status": val_res.tooling_status.value,
                        "alignment_status": val_res.alignment_status.value
                    }

                    heapq.heappush(open_set, PriorityQueueNode(f_score, next_state, current_path + [step_info]))

        return {
            "success": False,
            "error": "Max iterations reached or no valid sequence path found.",
            "nodes_explored": self.nodes_explored,
            "planning_time": time.time() - start_time,
            "rejections": rejections
        }
