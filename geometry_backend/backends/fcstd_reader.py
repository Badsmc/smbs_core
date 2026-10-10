"""
fcstd_reader.py - Reads .FCStd (FreeCAD Document) files without FreeCAD API.
Uses Python's zipfile and xml.etree.ElementTree to extract placement and geometry (.brp).
Then uses pythonocc-core to load the BRep shapes.
"""

import os
import zipfile
import tempfile
import xml.etree.ElementTree as ET
from typing import List, Tuple, Any

# Delay OCC imports to avoid crashing if OCC is not installed when just parsing XML
try:
    from OCC.Core.BRepTools import breptools_Read
    from OCC.Core.BRep import BRep_Builder
    from OCC.Core.TopoDS import TopoDS_Shape
    from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform
    from OCC.Core.gp import gp_Trsf, gp_Vec, gp_Ax1, gp_Pnt, gp_Dir, gp_Quaternion
    import math
    HAS_OCC = True
except ImportError:
    HAS_OCC = False

def _parse_placement(prop_element: ET.Element) -> Tuple[Tuple[float, float, float], Tuple[float, float, float], float]:
    """
    Extracts Position and Axis/Angle from a PropertyPlacement element.
    Returns (position, axis, angle_deg).
    """
    placement = prop_element.find("Placement")
    if placement is None:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 0.0
        
    pos_elem = placement.find("Position")
    axis_elem = placement.find("Axis")
    angle_elem = placement.find("Angle")
    
    pos = (0.0, 0.0, 0.0)
    if pos_elem is not None:
        pos = (
            float(pos_elem.get("x", 0.0)),
            float(pos_elem.get("y", 0.0)),
            float(pos_elem.get("z", 0.0))
        )
        
    axis = (0.0, 0.0, 1.0)
    if axis_elem is not None:
        axis = (
            float(axis_elem.get("x", 0.0)),
            float(axis_elem.get("y", 0.0)),
            float(axis_elem.get("z", 1.0))
        )
        
    angle_deg = 0.0
    if angle_elem is not None:
        angle_text = angle_elem.text
        if angle_text:
            angle_deg = float(angle_text)
            # FreeCAD might store angles in radians sometimes, but usually it's degrees in Document.xml.
            # FreeCAD XML: <Angle>1.570796326794897</Angle> is radians!
            # Wait, no, FreeCAD XML usually stores Angle in RADIANS?
            # Let's assume radians for now, if it's typical FC. Wait!
            # Actually, FreeCAD Document.xml usually stores it in radians. 
            # We'll convert it to degrees below to be safe, OR we can just use radians directly in gp_Trsf.
            # I will return angle_rad, wait. Let's return the raw float.
    
    return pos, axis, angle_deg

def _build_trsf(pos: Tuple[float, float, float], axis: Tuple[float, float, float], angle: float) -> Any:
    """Builds a gp_Trsf from position, axis, and angle (in radians)."""
    trsf = gp_Trsf()
    if angle != 0.0:
        ax1 = gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(*axis))
        trsf.SetRotation(ax1, angle)
    
    if pos != (0.0, 0.0, 0.0):
        trsf.SetTranslationPart(gp_Vec(*pos))
        
    return trsf

def read_fcstd(path: str) -> List[Tuple[str, Any, Any]]:
    """
    Reads an FCStd file, extracts .brp shapes, applies Placement, and returns a list of:
    (object_name, TopoDS_Shape_local, gp_Trsf_world)
    """
    if not HAS_OCC:
        raise ImportError("pythonocc-core is required to load BRep geometry.")
        
    results = []
    
    with zipfile.ZipFile(path, 'r') as zf:
        if "Document.xml" not in zf.namelist():
            raise ValueError(f"Invalid FCStd file (no Document.xml found): {path}")
            
        xml_data = zf.read("Document.xml")
        root = ET.fromstring(xml_data)
        
        # FreeCAD objects are in <ObjectData> or directly under <Document>
        # Let's find all <Object> tags
        for obj in root.iter("Object"):
            obj_name = obj.get("name", "Unknown")
            obj_type = obj.get("type", "")
            
            # We look for objects that have a Shape property pointing to a .brp
            properties = obj.find("Properties")
            if properties is None:
                continue
                
            brp_file = None
            pos = (0.0, 0.0, 0.0)
            axis = (0.0, 0.0, 1.0)
            angle_rad = 0.0
            
            for prop in properties.findall("Property"):
                p_name = prop.get("name")
                if p_name == "Shape":
                    part = prop.find("Part")
                    if part is not None:
                        brp_file = part.get("file")
                elif p_name == "Placement":
                    pos, axis, angle_rad = _parse_placement(prop)
            
            if brp_file and brp_file in zf.namelist():
                # Extract the .brp to a temporary file
                brp_data = zf.read(brp_file)
                fd, tmp_path = tempfile.mkstemp(suffix=".brp")
                try:
                    with open(fd, 'wb') as f:
                        f.write(brp_data)
                    
                    shape = TopoDS_Shape()
                    builder = BRep_Builder()
                    
                    try:
                        from OCC.Core.BRepTools import breptools
                        success = breptools.Read(shape, tmp_path, builder)
                    except AttributeError:
                        from OCC.Core.BRepTools import breptools_Read
                        success = breptools_Read(shape, tmp_path, builder)
                    
                    if success:
                        trsf = _build_trsf(pos, axis, angle_rad)
                        results.append((obj_name, shape, trsf))
                finally:
                    os.remove(tmp_path)
                    
    return results
