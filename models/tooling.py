"""
tooling.py - Data models for Machine, Tooling and Datums (headless core).

Strict contracts:
  * ``geometry_status`` is mandatory. A tool whose B-Rep could not be loaded
    from a ``.brp`` inside its ``.FCStd`` is ``MISSING`` -- never ``REAL``.
  * A datum with ``source == "inferred"`` is NOT usable for final
    verification unless ``confirmed`` is True (user confirmation).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Literal, Optional, Tuple

Vec3 = Tuple[float, float, float]


class ToolingKind(Enum):
    PUNCH = "punch"
    DIE = "die"
    MACHINE_BED = "machine_bed"
    MACHINE_FRAME = "machine_frame"
    RAM = "ram"
    BACKGAUGE = "backgauge"


class DatumRole(Enum):
    MOUNTING_PLANE = "mounting_plane"     # plane mating with the machine
    WORKING_AXIS = "working_axis"         # tool working line (tip line / V line)
    TIP_POINT = "tip_point"               # punch tip
    V_CENTER = "v_center"                 # die V-groove centre (top plane)
    V_OPENING = "v_opening"               # die V opening
    STOP_FACE = "stop_face"               # backgauge stop face
    REFERENCE_PLANE = "reference_plane"   # orientation reference


class GeometryStatus(Enum):
    REAL = "real"              # loaded from a .brp inside the .FCStd
    SIMPLIFIED = "simplified"  # synthetic profile / inferred datums
    MISSING = "missing"        # no geometry


class AlignmentStatus(Enum):
    ALIGNED = "ALIGNED"
    MISALIGNED = "MISALIGNED"
    CANNOT_VERIFY = "CANNOT_VERIFY"


DatumKind = Literal["point", "axis", "plane", "face"]
DatumSource = Literal["sidecar", "inferred"]


def _norm(v: Vec3) -> Vec3:
    m = math.sqrt(sum(c * c for c in v))
    if m < 1e-12:
        raise ValueError(f"Zero-length direction in datum: {v}")
    return (v[0] / m, v[1] / m, v[2] / m)


@dataclass
class Datum:
    """
    geometry layout by kind:
      point : (position,)
      axis  : (origin, direction)
      plane : (origin, normal[, x_direction])   normal = OUTWARD normal of the face
      face  : (origin, normal)
    """
    role: DatumRole
    kind: DatumKind
    geometry: Tuple[Any, ...]
    tolerance_mm: float = 0.05
    source: DatumSource = "sidecar"
    confirmed: bool = True
    value: Optional[float] = None   # scalar datums (e.g. V opening width)

    @property
    def usable_for_verification(self) -> bool:
        return self.source == "sidecar" or self.confirmed

    @property
    def origin(self) -> Vec3:
        return tuple(float(c) for c in self.geometry[0])  # type: ignore

    @property
    def direction(self) -> Vec3:
        if self.kind == "point":
            raise ValueError("Point datum has no direction")
        return _norm(tuple(float(c) for c in self.geometry[1]))  # type: ignore

    @property
    def x_direction(self) -> Optional[Vec3]:
        if self.kind == "plane" and len(self.geometry) > 2 and self.geometry[2] is not None:
            return _norm(tuple(float(c) for c in self.geometry[2]))  # type: ignore
        return None

    @staticmethod
    def from_json(role: DatumRole, data: Dict[str, Any], source: DatumSource = "sidecar") -> "Datum":
        kind = data.get("kind", "point")
        tol = float(data.get("tolerance_mm", 0.05))
        if kind == "point":
            geom: Tuple[Any, ...] = (tuple(data["position"]),)
        elif kind == "axis":
            geom = (tuple(data["origin"]), tuple(data["direction"]))
        elif kind in ("plane", "face"):
            xd = data.get("x_direction")
            geom = (tuple(data["origin"]), tuple(data["normal"]), tuple(xd) if xd else None)
        else:
            raise ValueError(f"Unknown datum kind '{kind}' for {role.value}")
        return Datum(role=role, kind=kind, geometry=geom, tolerance_mm=tol, source=source,
                     confirmed=bool(data.get("confirmed", source == "sidecar")),
                     value=data.get("value"))


@dataclass
class Tooling:
    id: str
    kind: ToolingKind
    source_file: str
    shape: Any                      # TopoDS_Shape | None (tool-local coordinates)
    local_frame: Any                # gp_Ax3 / gp_Trsf of the FreeCAD placement
    geometry_status: GeometryStatus
    datums: Dict[DatumRole, Datum] = field(default_factory=dict)
    metadata: Dict[str, Any] = field(default_factory=dict)
    diagnostics: List[str] = field(default_factory=list)

    def __post_init__(self) -> None:
        if not isinstance(self.geometry_status, GeometryStatus):
            self.geometry_status = GeometryStatus(str(self.geometry_status).lower())
        if self.shape is None and self.geometry_status != GeometryStatus.MISSING:
            self.diagnostics.append(f"{self.id}: shape is None -> geometry_status forced to MISSING")
            self.geometry_status = GeometryStatus.MISSING

    def has_verified_datums(self, roles: List[DatumRole]) -> bool:
        return all(r in self.datums and self.datums[r].usable_for_verification for r in roles)


@dataclass
class MachineConfiguration:
    id: str
    bed_top_plane: Datum
    ram_mount_plane: Datum
    ram_stroke_mm: float
    ram_direction: Vec3
    backgauge_axes: Dict[str, Dict[str, Any]]
    max_tonnage_kn: float
    max_bend_length_mm: float
    source_file: Optional[str] = None
    lateral_tolerance_mm: float = 0.05
    angular_tolerance_deg: float = 0.1
    frame: Optional[Tooling] = None
    backgauge_finger: Optional[Tooling] = None
    metadata: Dict[str, Any] = field(default_factory=dict)
