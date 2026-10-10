from abc import ABC, abstractmethod
from typing import Any, Tuple, Dict, Set, List, Optional

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
    def rotate_subtree(
        self,
        shape: Any,
        axis_origin: Tuple[float, float, float],
        axis_direction: Tuple[float, float, float],
        signed_angle_deg: float,
        moving_panels: Optional[Set[str]] = None,
        panel_graph: Optional[Any] = None
    ) -> Any:
        """Rotates a specified subset of faces/panels around a given axis."""
        pass

    @abstractmethod
    def intersect_volume(self, shape1: Any, shape2: Any) -> bool:
        """Returns True if shape1 and shape2 have a volumetric intersection (> 0 volume)."""
        pass

    @abstractmethod
    def get_faces(self, shape: Any) -> Dict[str, Any]:
        """Returns a map of stable face_id to backend-specific face object."""
        pass
