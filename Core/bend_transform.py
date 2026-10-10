"""
bend_transform.py - Math and Kinematics for Bend Transformations (Headless Core)
"""
import math
from typing import Tuple, List, Any, Optional, Set
from smbs_core.geometry_backend.base import GeometryBackend

class BendTransform:
    """
    Mathematical operations for exact 3D rigid-body transformations of bend points and axes.
    """

    @staticmethod
    def rodrigues_rotation_matrix(
        axis_direction: Tuple[float, float, float],
        signed_angle_deg: float
    ) -> List[List[float]]:
        u = BendTransform._normalize(axis_direction)
        rad = math.radians(signed_angle_deg)
        cos_t = math.cos(rad)
        sin_t = math.sin(rad)
        one_minus_cos = 1.0 - cos_t
        ux, uy, uz = u

        uuT = [
            [ux*ux, ux*uy, ux*uz],
            [uy*ux, uy*uy, uy*uz],
            [uz*ux, uz*uy, uz*uz]
        ]
        u_cross = [
            [0.0, -uz, uy],
            [uz, 0.0, -ux],
            [-uy, ux, 0.0]
        ]
        R = [
            [cos_t + one_minus_cos*uuT[0][0] + sin_t*u_cross[0][0],
             one_minus_cos*uuT[0][1] + sin_t*u_cross[0][1],
             one_minus_cos*uuT[0][2] + sin_t*u_cross[0][2]],
            [one_minus_cos*uuT[1][0] + sin_t*u_cross[1][0],
             cos_t + one_minus_cos*uuT[1][1] + sin_t*u_cross[1][1],
             one_minus_cos*uuT[1][2] + sin_t*u_cross[1][2]],
            [one_minus_cos*uuT[2][0] + sin_t*u_cross[2][0],
             one_minus_cos*uuT[2][1] + sin_t*u_cross[2][1],
             cos_t + one_minus_cos*uuT[2][2] + sin_t*u_cross[2][2]]
        ]
        return R

    @staticmethod
    def transform_point(
        point: Tuple[float, float, float],
        axis_origin: Tuple[float, float, float],
        axis_direction: Tuple[float, float, float],
        signed_angle_deg: float
    ) -> Tuple[float, float, float]:
        R = BendTransform.rodrigues_rotation_matrix(axis_direction, signed_angle_deg)
        cx, cy, cz = axis_origin
        px, py, pz = point
        dx, dy, dz = px - cx, py - cy, pz - cz
        rx = R[0][0]*dx + R[0][1]*dy + R[0][2]*dz
        ry = R[1][0]*dx + R[1][1]*dy + R[1][2]*dz
        rz = R[2][0]*dx + R[2][1]*dy + R[2][2]*dz
        return (round(cx + rx, 6), round(cy + ry, 6), round(cz + rz, 6))

    @staticmethod
    def transform_vector(
        vector: Tuple[float, float, float],
        axis_direction: Tuple[float, float, float],
        signed_angle_deg: float
    ) -> Tuple[float, float, float]:
        R = BendTransform.rodrigues_rotation_matrix(axis_direction, signed_angle_deg)
        vx, vy, vz = vector
        rx = R[0][0]*vx + R[0][1]*vy + R[0][2]*vz
        ry = R[1][0]*vx + R[1][1]*vy + R[1][2]*vz
        rz = R[2][0]*vx + R[2][1]*vy + R[2][2]*vz
        return (round(rx, 6), round(ry, 6), round(rz, 6))

    @staticmethod
    def rotate_subtree(
        backend: GeometryBackend,
        shape: Any,
        axis_origin: Tuple[float, float, float],
        axis_direction: Tuple[float, float, float],
        signed_angle_deg: float,
        moving_panels: Optional[Set[str]] = None,
        panel_graph: Optional[Any] = None
    ) -> Any:
        if shape is None:
            return shape
            
        moving_face_ids = set()
        if moving_panels and panel_graph:
            for p_id in moving_panels:
                if p_id in panel_graph.panels:
                    for idx in panel_graph.panels[p_id].face_indices:
                        moving_face_ids.add(str(idx))
                        
        return backend.rotate_subtree(shape, axis_origin, axis_direction, signed_angle_deg, moving_face_ids)

    @staticmethod
    def _normalize(v: Tuple[float, float, float]) -> Tuple[float, float, float]:
        mag = math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)
        if mag < 1e-9:
            return (0.0, 0.0, 1.0)
        return (v[0] / mag, v[1] / mag, v[2] / mag)
