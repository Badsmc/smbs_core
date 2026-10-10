from typing import Any, Tuple, Dict, Set, List, Optional
from smbs_core.geometry_backend.base import GeometryBackend

class MockBackend(GeometryBackend):
    """
    Mock geometry backend for testing the core planning algorithms without a real CAD kernel.
    """
    
    def load_shape(self, filepath: str) -> Any:
        return {"type": "mock_shape", "file": filepath}

    def copy_shape(self, shape: Any) -> Any:
        return shape.copy() if isinstance(shape, dict) else shape

    def get_bounding_box(self, shape: Any) -> Tuple[float, float, float, float, float, float]:
        return (0.0, 0.0, 0.0, 10.0, 10.0, 10.0)

    def rotate_subtree(
        self,
        shape: Any,
        axis_origin: Tuple[float, float, float],
        axis_direction: Tuple[float, float, float],
        signed_angle_deg: float,
        moving_panels: Optional[Set[str]] = None,
        panel_graph: Optional[Any] = None
    ) -> Any:
        new_shape = self.copy_shape(shape)
        if isinstance(new_shape, dict):
            new_shape["last_rotation"] = signed_angle_deg
        return new_shape

    def intersect_volume(self, shape1: Any, shape2: Any) -> bool:
        return False

    def get_faces(self, shape: Any) -> Dict[str, Any]:
        return {"FACE_1": "mock_face_1", "FACE_2": "mock_face_2"}
        
    def is_valid(self, shape: Any) -> bool:
        return True

    def export_brep(self, shape: Any, filepath: str) -> bool:
        return True

    def make_box(self, x: float, y: float, z: float) -> Any:
        return {"type": "mock_box", "dim": (x, y, z)}

    def transform_shape(self, shape: Any, translation: Tuple[float, float, float], 
                        axis_origin: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                        axis_dir: Tuple[float, float, float] = (0.0, 0.0, 1.0),
                        angle_deg: float = 0.0) -> Any:
        new_shape = self.copy_shape(shape)
        if isinstance(new_shape, dict):
            new_shape["translation"] = translation
            new_shape["rotation"] = angle_deg
        return new_shape

    def extract_faces(self, shape: Any, face_ids: Set[str]) -> Any:
        new_shape = self.copy_shape(shape)
        if isinstance(new_shape, dict):
            new_shape["extracted_faces"] = list(face_ids)
        return new_shape

    def get_cylinder_data(self, face: Any) -> Dict[str, Any]:
        return {}
