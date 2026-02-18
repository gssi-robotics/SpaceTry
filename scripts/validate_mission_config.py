#!/usr/bin/env python3
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

try:
    import yaml  # type: ignore
except Exception as e:
    print("ERROR: PyYAML not available. Install python3-yaml in the container.", file=sys.stderr)
    raise

def die(msg: str) -> None:
    print(f"ERROR: {msg}", file=sys.stderr)
    sys.exit(1)

def load_yaml(path: Path):
    if not path.exists():
        die(f"Missing YAML file: {path}")
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)

def extract_included_names(world_sdf: Path) -> set[str]:
    if not world_sdf.exists():
        die(f"Missing world SDF: {world_sdf}")
    tree = ET.parse(world_sdf)
    root = tree.getroot()

    names: set[str] = set()
    for inc in root.iter("include"):
        name_el = inc.find("name")
        if name_el is not None and name_el.text:
            names.add(name_el.text.strip())
    return names

def validate_waypoints(waypoints_yaml: dict) -> None:
    if not isinstance(waypoints_yaml, dict):
        die("waypoints.yaml must be a mapping")
    if "frame" not in waypoints_yaml:
        die("waypoints.yaml missing top-level key: frame")
    if "waypoints" not in waypoints_yaml:
        die("waypoints.yaml missing top-level key: waypoints")

    wps = waypoints_yaml["waypoints"]
    if not isinstance(wps, dict) or not wps:
        die("waypoints.yaml waypoints must be a non-empty mapping")

    for name, wp in wps.items():
        if not isinstance(wp, dict):
            die(f"waypoint '{name}' must be a mapping")
        for k in ["x", "y", "yaw"]:
            if k not in wp:
                die(f"waypoint '{name}' missing key '{k}'")
            if not isinstance(wp[k], (int, float)):
                die(f"waypoint '{name}' key '{k}' must be a number")
        if "tolerance" in wp and not isinstance(wp["tolerance"], (int, float)):
            die(f"waypoint '{name}' tolerance must be a number")

def validate_objects(objects_yaml: dict, world_names: set[str]) -> None:
    if not isinstance(objects_yaml, dict) or "objects" not in objects_yaml:
        die("objects.yaml must be a mapping with top-level key: objects")

    objs = objects_yaml["objects"]
    if not isinstance(objs, list) or not objs:
        die("objects.yaml objects must be a non-empty list")

    seen: set[str] = set()
    missing: list[str] = []

    for obj in objs:
        if not isinstance(obj, dict):
            die("each object entry must be a mapping")
        if "id" not in obj or "type" not in obj:
            die("each object must have 'id' and 'type'")
        oid = obj["id"]
        if not isinstance(oid, str) or not oid:
            die("object id must be a non-empty string")

        if oid in seen:
            die(f"duplicate object id: {oid}")
        seen.add(oid)

        if oid not in world_names:
            missing.append(oid)

    if missing:
        die("objects.yaml references IDs not found in world SDF: " + ", ".join(missing))

def main() -> None:
    # Require these env vars so this script always validates the *installed* artifacts
    world_sdf = os.environ.get("SPACETRY_WORLD_SDF")
    mission_config_dir = os.environ.get("SPACETRY_MISSION_CONFIG_DIR")

    if not world_sdf:
        die("SPACETRY_WORLD_SDF env var not set")
    if not mission_config_dir:
        die("SPACETRY_MISSION_CONFIG_DIR env var not set")

    world_path = Path(world_sdf)
    config_dir = Path(mission_config_dir)

    world_names = extract_included_names(world_path)

    waypoints = load_yaml(config_dir / "waypoints.yaml")
    objects = load_yaml(config_dir / "objects.yaml")

    validate_waypoints(waypoints)
    validate_objects(objects, world_names)

if __name__ == "__main__":
    main()
