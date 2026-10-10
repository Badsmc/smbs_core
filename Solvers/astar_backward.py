"""
astar_backward.py - Backward A* Search Planner with Physical Oracle (SMBS Phase 8)

SPEC v1.0 Requirement (Section 19 & 48):
A* search planner operating directly on canonical FoldState and Physical Oracle gate.
Priority queue ranks state nodes by f(n) = g(n) + h(n).
Every candidate transition is validated by PhysicalValidator.
"""

import heapq
from typing import Dict, Any, List, Optional, Set, Tuple
from smbs_core.Core.fold_state import FoldState
from smbs_core.Core.bend_graph import BendGraph, BendRecord
from smbs_core.Core.panel_graph import PanelGraph
from smbs_core.Core.state_cache import StateCache
from smbs_core.Physics.validator import PhysicalValidator, ValidationResult
from .base_planner import BasePlanner


class AStarBackwardPlanner(BasePlanner):
    """
    Backward A* Search Planner with Physical Oracle Gate.
    """

    def __init__(
        self,
        bend_graph: BendGraph,
        validator: PhysicalValidator,
        panel_graph: Optional[PanelGraph] = None,
        state_cache: Optional[StateCache] = None
    ):
        super().__init__(validator)
        self.bend_graph = bend_graph
        self.panel_graph = panel_graph
        self.cache = state_cache or StateCache()

    def compute_heuristic(self, state: FoldState) -> float:
        """
        Admissible heuristic:
        Each remaining bend requires at least 1 step (cost = 1.0).
        We don't artificially penalize inner bends to guarantee optimal search.
        """
        num_remaining = len(state.remaining_bends)
        return float(num_remaining)

    def solve(
        self,
        initial_state: FoldState,
        max_iterations: int = 5000,
        timeout: Optional[float] = None,
        tooling_override: Optional[Dict[str, Any]] = None
    ) -> Dict[str, Any]:
        """
        Execute backward A* search.
        """
        import time
        start_time = time.time()
        initial_fp = initial_state.fingerprint()
        initial_h = self.compute_heuristic(initial_state)
        counter = 0
        open_set: List[Tuple[float, int, FoldState]] = [(initial_h, counter, initial_state)]
        g_scores: Dict[str, float] = {initial_fp: 0.0}
        came_from: Dict[str, Tuple[str, Dict[str, Any]]] = {}

        nodes_explored = 0

        while open_set and nodes_explored < max_iterations:
            if timeout is not None and (time.time() - start_time) > timeout:
                break
                
            _, _, current = heapq.heappop(open_set)
            nodes_explored += 1
            current_fp = current.fingerprint()

            if current.is_goal():
                # Reconstruct path
                sequence = []
                curr_fp = current_fp
                step_num = 1
                while curr_fp in came_from:
                    parent_fp, action = came_from[curr_fp]
                    action_copy = dict(action)
                    action_copy["step_number"] = step_num
                    sequence.append(action_copy)
                    curr_fp = parent_fp
                    step_num += 1
                
                return {
                    "success": True,
                    "sequence": sequence,
                    "nodes_explored": nodes_explored,
                    "cache_stats": self.cache.stats
                }
                
            # Determine previous bend angle to penalize flips
            last_angle = 0.0
            if current_fp in came_from:
                _, last_action = came_from[current_fp]
                last_bend = self.bend_graph.get_bend(last_action["bend_id"])
                if last_bend:
                    last_angle = last_bend.signed_angle

            def score_candidate(b_id: str) -> float:
                """
                Tie-breaker heuristic for priority queue ordering (not A* cost).
                Guides search to evaluate "no-flip" and "outer" bends first.
                """
                score = 0.0
                brec = self.bend_graph.get_bend(b_id)
                if brec:
                    # Penalize flips in ordering
                    if (last_angle > 0 and brec.signed_angle < 0) or (last_angle < 0 and brec.signed_angle > 0):
                        score += 10.0
                
                if self.panel_graph:
                    moving_count = len(self.panel_graph.moving_subtree(b_id))
                    total_count = len(self.panel_graph.panels)
                    score += min(moving_count, total_count - moving_count) * 0.1
                return score

            candidates = sorted(list(current.remaining_bends), key=score_candidate)

            for bend_id in candidates:
                bend_rec = self.bend_graph.get_bend(bend_id)
                if bend_rec is None:
                    continue

                # Calculate true transition cost
                step_cost = 1.0
                # Add penalty for flip
                if (last_angle > 0 and bend_rec.signed_angle < 0) or (last_angle < 0 and bend_rec.signed_angle > 0):
                    step_cost += 5.0  # Large penalty for flipping the sheet

                tentative_g = g_scores[current_fp] + step_cost
                
                # Predictive check: skip OCC if target state is already reached via a better/equal path
                next_fp = current.predict_successor_fingerprint(bend_id, panel_graph=self.panel_graph)
                if next_fp in g_scores and tentative_g >= g_scores[next_fp]:
                    continue

                val_result: ValidationResult = self.validator.validate_step(
                    current_state=current,
                    bend_record=bend_rec,
                    panel_graph=self.panel_graph,
                    tooling_override=tooling_override
                )

                if not val_result.valid:
                    continue  # INVALID transition -> reject branch

                successor = val_result.next_state
                if successor is None:
                    continue

                fingerprint = successor.fingerprint()

                if fingerprint not in g_scores or tentative_g < g_scores[fingerprint]:
                    g_scores[fingerprint] = tentative_g
                    h_cost = self.compute_heuristic(successor)
                    f_cost = tentative_g + h_cost
                    self.cache.put(fingerprint, successor)

                    moving_panels = list(self.panel_graph.moving_subtree(bend_id)) if self.panel_graph else []
                    action = {
                        "bend_id": bend_id,
                        "unfolded_angle": 0.0,
                        "moving_panels": moving_panels,
                        "diagnostics": {"signed_angle": bend_rec.signed_angle}
                    }
                    came_from[fingerprint] = (current_fp, action)

                    counter += 1
                    heapq.heappush(open_set, (f_cost, counter, successor))

            # Memory Optimization: Free the shape after expanding all successors
            # Since FoldState tree keeps parents alive, this prevents OOM.
            if hasattr(current, "shape") and current.shape is not None:
                if hasattr(current.shape, "nullify"):
                    try:
                        current.shape.nullify()
                    except Exception:
                        pass
                current.shape = None

        return {
            "success": False,
            "error": f"A* search exhausted after exploring {nodes_explored} nodes without reaching flat state.",
            "nodes_explored": nodes_explored,
            "cache_stats": self.cache.stats
        }
