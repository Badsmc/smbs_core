from abc import ABC, abstractmethod
from typing import Any, Tuple, Dict, Set, List

class GeometryBackend(ABC):
    """
    Abstract Base Class defining required geometry operations for SMBS.
    Decoupled from FreeCAD to allow running in headless environments via pythonocc-core.
    """
    
    @abstractmethod
    def load_shape(self, filepath: str) -> Any:
        """Load a neutral 3D model (e.g. STEP)."""
        pass

    @abstractmethod
    def copy_shape(self, shape: Any) -> Any:
        """Create an independent deep copy of the shape."""
        pass

    @abstractmethod
    def get_bounding_box(self, shape: Any) -> Tuple[float, float, float, float, float, float]:
        """Returns (xmin, ymin, zmin, xmax, ymax, zmax)"""
        pass

    @abstractmethod
    def rotate_subtree(self, shape: Any, axis_origin: Tuple[float, float, float], 
                       axis_dir: Tuple[float, float, float], angle_deg: float, 
                       moving_face_ids: Set[str]) -> Any:
        """Rotates a specified subset of faces around a given axis."""
        pass

    @abstractmethod
    def intersect_volume(self, shape1: Any, shape2: Any) -> bool:
        """Returns True if shape1 and shape2 have a volumetric intersection (> 0 volume)."""
        pass

    @abstractmethod
    def get_faces(self, shape: Any) -> Dict[str, Any]:
        """Returns a map of stable face_id to backend-specific face object."""
        pass
        
    @abstractmethod
    def is_valid(self, shape: Any) -> bool:
        """Checks if the shape is topologically valid."""
        pass

    @abstractmethod
    def export_brep(self, shape: Any, filepath: str) -> bool:
        """Export the shape to BREP or STEP format."""
        pass

    @abstractmethod
    def make_box(self, x: float, y: float, z: float) -> Any:
        """Create a primitive box for testing or fallback tooling."""
        pass

    @abstractmethod
    def transform_shape(self, shape: Any, translation: Tuple[float, float, float], 
                        axis_origin: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                        axis_dir: Tuple[float, float, float] = (0.0, 0.0, 1.0),
                        angle_deg: float = 0.0) -> Any:
        """Translate and rotate an entire shape."""
        pass

    @abstractmethod
    def extract_faces(self, shape: Any, face_ids: Set[str]) -> Any:
        """Extract a subset of faces into a new shape (e.g. Compound)."""
        pass

    @abstractmethod
    def get_surface_type(self, face: Any) -> str:
        """Return 'Plane', 'Cylinder', or 'Other'."""
        pass

    @abstractmethod
    def get_face_normal(self, face: Any) -> Tuple[float, float, float]:
        """Return outward normal of a planar face."""
        pass

    @abstractmethod
    def get_cylinder_data(self, face: Any) -> Dict[str, Any]:
        """Return dict with 'radius', 'axis', 'origin', 'length' for a cylindrical face."""
        pass

    @abstractmethod
    def get_face_adjacencies(self, shape: Any) -> Dict[str, List[str]]:
        """Return adjacency map: face_id -> list of adjacent face_ids."""
        pass
