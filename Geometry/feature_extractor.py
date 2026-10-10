"""
feature_extractor.py - Feature Extraction for Sheet Metal 3D Models (Headless Core)
"""
import math
from typing import Dict, Any, List, Optional, Tuple
from smbs_core.Core.bend_graph import BendRecord, BendGraph, IncompleteBendError
from smbs_core.Core.panel_graph import Panel, PanelGraph
from smbs_core.geometry_backend.base import GeometryBackend

class FeatureExtractionError(Exception):
    pass

class FeatureExtractor:
    def __init__(self, backend: GeometryBackend, default_k_factor: float = 0.40):
        self.backend = backend
        self.default_k_factor = default_k_factor

    def calculate_signed_angle(
        self,
        normal1: Tuple[float, float, float],
        normal2: Tuple[float, float, float],
        bend_axis: Tuple[float, float, float]
    ) -> float:
        n1 = self._normalize(normal1)
        n2 = self._normalize(normal2)
        axis = self._normalize(bend_axis)

        dot = n1[0]*n2[0] + n1[1]*n2[1] + n1[2]*n2[2]
        dot_clamped = max(-1.0, min(1.0, dot))
        alpha_rad = math.acos(dot_clamped)
        alpha_deg = math.degrees(alpha_rad)

        cross = (
            n1[1]*n2[2] - n1[2]*n2[1],
            n1[2]*n2[0] - n1[0]*n2[2],
            n1[0]*n2[1] - n1[1]*n2[0]
        )

        triple = axis[0]*cross[0] + axis[1]*cross[1] + axis[2]*cross[2]
        sign = 1.0 if triple >= 0.0 else -1.0

        signed_angle = sign * alpha_deg
        if abs(signed_angle) < 1e-4:
            raise FeatureExtractionError("Bend angle is zero, invalid geometry.")

        return round(signed_angle, 2)

    @staticmethod
    def _normalize(v: Tuple[float, float, float]) -> Tuple[float, float, float]:
        mag = math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)
        if mag < 1e-9:
            return (0.0, 0.0, 1.0)
        return (v[0] / mag, v[1] / mag, v[2] / mag)

    def extract_features(self, shape: Any) -> Dict[str, Any]:
        if shape is None:
            raise FeatureExtractionError("Cannot extract features from None shape.")

        thickness = self._detect_thickness(shape)
        bends = self._detect_bends(shape, thickness)

        if not bends:
            raise FeatureExtractionError("No bends detected in shape.")

        return {
            "thickness": thickness,
            "k_factor": self.default_k_factor,
            "material": "Steel_S235",
            "bends": bends
        }

    def _detect_thickness(self, shape: Any) -> float:
        bb = self.backend.get_bounding_box(shape)
        if bb:
            dims = sorted([bb[3]-bb[0], bb[4]-bb[1], bb[5]-bb[2]])
            if 0.5 <= dims[0] <= 15.0:
                return round(dims[0], 2)
        raise FeatureExtractionError("Could not reliably determine sheet thickness from geometry.")

    def _detect_bends(self, shape: Any, thickness: float) -> Dict[str, Dict[str, Any]]:
        # Fallback for mock shapes
        if hasattr(shape, "get") and shape.get("type") == "mock_shape":
            return {}

        faces = self.backend.get_faces(shape)
        if not faces:
            return {}

        adj_map = self.backend.get_face_adjacencies(shape)
        
        cylinders = []
        planes = []
        for i, face in enumerate(faces):
            f_type = self.backend.get_surface_type(face)
            if f_type == "Cylinder":
                cylinders.append(str(i))
            elif f_type == "Plane":
                planes.append(str(i))

        bends = {}
        bend_idx = 1
        
        # Deduplicate inner/outer cylinders first
        unique_cyls = []
        for cyl_id in cylinders:
            face = faces[int(cyl_id)]
            cyl_data = self.backend.get_cylinder_data(face)
            if not cyl_data: continue
            
            radius = cyl_data.get("radius", 0.0)
            if radius > thickness * 5.0: continue
            
            axis = cyl_data.get("axis", (0, 0, 1))
            origin = cyl_data.get("origin", (0, 0, 0))
            is_dup = False
            for u in unique_cyls:
                u_axis = u['data'].get("axis", (0, 0, 1))
                u_origin = u['origin']
                
                # Check if axes are parallel
                dot_axis = sum(axis[i]*u_axis[i] for i in range(3))
                if abs(abs(dot_axis) - 1.0) < 1e-4:
                    # Calculate perpendicular distance between axes
                    vec = [origin[i] - u_origin[i] for i in range(3)]
                    # cross product vec x axis
                    cross = [
                        vec[1]*axis[2] - vec[2]*axis[1],
                        vec[2]*axis[0] - vec[0]*axis[2],
                        vec[0]*axis[1] - vec[1]*axis[0]
                    ]
                    dist = math.sqrt(sum(c**2 for c in cross))
                    if dist < 1e-3:
                        is_dup = True
                        if radius < u['radius']:
                            u['id'] = cyl_id
                            u['radius'] = radius
                            u['data'] = cyl_data
                        break
            
            if not is_dup:
                unique_cyls.append({
                    'id': cyl_id,
                    'radius': radius,
                    'origin': origin,
                    'data': cyl_data
                })

        for u in unique_cyls:
            cyl_id = u['id']
            cyl_data = u['data']
            radius = u['radius']
            
            adjacent_faces = adj_map.get(cyl_id, [])
            adj_planes = [fid for fid in adjacent_faces if fid in planes]
            
            axis = cyl_data.get("axis", (0, 0, 1))
            
            # Filter adjacent planes: main panels have normals perpendicular to bend axis
            valid_planes = []
            for p_id in adj_planes:
                n = self.backend.get_face_normal(faces[int(p_id)])
                dot = n[0]*axis[0] + n[1]*axis[1] + n[2]*axis[2]
                if abs(dot) < 0.1:
                    valid_planes.append(p_id)
            
            if len(valid_planes) == 2:
                p1_id, p2_id = valid_planes
                p1_face = faces[int(p1_id)]
                p2_face = faces[int(p2_id)]
                
                n1 = self.backend.get_face_normal(p1_face)
                n2 = self.backend.get_face_normal(p2_face)
                
                angle = self.calculate_signed_angle(n1, n2, axis)
                
                # Correct angle sign based on inner radius logic if needed, but calculate_signed_angle is fine.
                bends[f"BEND_{bend_idx:02d}"] = {
                    "origin": cyl_data.get("origin", (0, 0, 0)),
                    "axis": axis,
                    "angle": abs(angle),  # Usually bends are stored as positive unfolded angle or specific sign
                    "radius": radius,
                    "length": cyl_data.get("length", 100.0),
                    "parent_panel_id": f"PANEL_{p1_id}",
                    "child_panel_id": f"PANEL_{p2_id}"
                }
                bend_idx += 1

        return bends


