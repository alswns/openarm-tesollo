#!/usr/bin/env python3
"""Generate prefixable Xacro macros from the installed Tesollo models."""
import copy
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def generate(source: Path, destination: Path, side: str) -> None:
    root = ET.parse(source).getroot()
    links = {element.attrib["name"] for element in root.findall("link")}
    joints = {element.attrib["name"] for element in root.findall("joint")}
    active_joints = {element.attrib["name"] for element in root.findall("joint")
                     if element.get("type") != "fixed"}
    expected = {f"joint_{finger}_{joint}" for finger in range(1, 6) for joint in range(1, 5)}
    if "link_base" not in links or active_joints != expected:
        raise ValueError(f"Unexpected DG-5F-S model: {source}")
    output = ET.Element("robot", {"xmlns:xacro": "http://www.ros.org/wiki/xacro"})
    output.append(ET.Comment(" Derived from Tesollo DG-5F-S URDF, Copyright 2026 Tesollo Inc., BSD-3-Clause. "))
    macro = ET.SubElement(output, "xacro:macro", {"name": f"dg5f_s_{side}", "params": "prefix"})
    for element in root:
        element = copy.deepcopy(element)
        if element.tag == "link" and element.get("name") in links:
            element.set("name", "${prefix}" + element.get("name"))
        elif element.tag == "joint" and element.get("name") in joints:
            element.set("name", "${prefix}" + element.get("name"))
        for node in element.iter():
            if node.tag in ("parent", "child") and node.get("link") in links:
                node.set("link", "${prefix}" + node.get("link"))
            if node.tag == "mimic" and node.get("joint") in joints:
                node.set("joint", "${prefix}" + node.get("joint"))
        macro.append(element)
    ET.indent(output)
    ET.ElementTree(output).write(destination, encoding="utf-8", xml_declaration=True)


def main() -> None:
    destination_dir = Path(sys.argv[1])
    source_dir = Path(sys.argv[2])
    for side in ("left", "right"):
        generate(source_dir / f"dg5fs_{side}.urdf", destination_dir / f"dg5f_s_{side}.xacro", side)


if __name__ == "__main__":
    main()
