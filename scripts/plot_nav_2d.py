#!/usr/bin/env python3
import argparse
import struct
import json
import os
import re
import shlex
import subprocess
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# Host usage:
# python3 scripts/plot_nav_2d.py logs/icra_isrs_26/trace0
#
# Internal container usage:
# python3 scripts/plot_nav_2d.py \
#   /ws/logs/icra_isrs_26/trace0/records/<scenario>/rosbags/<bag_dir> \
#   --internal \
#   --output /ws/logs/icra_isrs_26/trace0/records/<scenario>/rosbags/<bag_dir>/navigation_2d.png


os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Ellipse, Polygon


ODOM_TOPIC = "/mobile_base_controller/odom"
TIMESTAMP_SUFFIX = re.compile(r"_(\d{8}T\d{6}Z)$")
TITLE_FONT_SIZE = 18
AXIS_LABEL_FONT_SIZE = 15
TICK_LABEL_FONT_SIZE = 13
ANNOTATION_FONT_SIZE = 12
LEGEND_FONT_SIZE = 12


def resolve_bag_storage(path: Path) -> Tuple[Path, str]:
    if path.is_file():
        if path.suffix == ".mcap":
            return path, "mcap"
        if path.suffix == ".db3":
            return path, "sqlite3"
        raise RuntimeError(f"Unsupported bag file format: '{path}'.")

    metadata_path = path / "metadata.yaml"
    if metadata_path.exists():
        try:
            import yaml

            with metadata_path.open("r", encoding="utf-8") as stream:
                metadata = yaml.safe_load(stream) or {}
            storage_id = (
                metadata.get("rosbag2_bagfile_information", {})
                .get("storage_identifier")
            )
            if storage_id:
                return path, str(storage_id)
        except Exception:
            pass

    mcap_files = sorted(path.glob("*.mcap"))
    if len(mcap_files) == 1:
        return mcap_files[0], "mcap"
    if len(mcap_files) > 1:
        raise RuntimeError(
            f"Bag directory '{path}' contains multiple .mcap files but no metadata.yaml."
        )

    db3_files = sorted(path.glob("*.db3"))
    if len(db3_files) == 1:
        return db3_files[0], "sqlite3"
    if len(db3_files) > 1:
        raise RuntimeError(
            f"Bag directory '{path}' contains multiple .db3 files but no metadata.yaml."
        )

    raise RuntimeError(
        f"Could not determine storage format for bag path '{path}'."
    )


def read_odom_samples(path: Path, topic_name: str) -> List[Tuple[float, float, float]]:
    read_errors = []

    try:
        return read_odom_samples_rosbag2_py(path, topic_name)
    except Exception as exc:
        read_errors.append(f"rosbag2_py path failed: {exc}")

    try:
        return read_odom_samples_rosbags(path, topic_name)
    except Exception as exc:
        read_errors.append(f"rosbags path failed: {exc}")

    raise RuntimeError(
        "Unable to read the rosbag on this machine. "
        "Install either ROS 2 Python bag libraries or the pure-Python "
        "`rosbags` package, then retry.\n"
        + "\n".join(read_errors)
    )


def read_odom_points(bag_path: Path, topic_name: str) -> List[Tuple[float, float]]:
    return [(x, y) for _, x, y in read_odom_samples(bag_path, topic_name)]


def normalize_samples(
    samples: List[Tuple[int, float, float]]
) -> List[Tuple[float, float, float]]:
    if not samples:
        return []
    first_stamp_ns = samples[0][0]
    return [
        ((stamp_ns - first_stamp_ns) / 1_000_000_000.0, x, y)
        for stamp_ns, x, y in samples
    ]


def read_odom_samples_rosbag2_py(
    bag_path: Path, topic_name: str
) -> List[Tuple[float, float, float]]:
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

    storage_uri, storage_id = resolve_bag_storage(bag_path)
    storage_options = StorageOptions(uri=str(storage_uri), storage_id=storage_id)
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    if topic_name not in topic_types:
        available = ", ".join(sorted(topic_types))
        raise RuntimeError(
            f"Topic '{topic_name}' not found in bag '{bag_path}'. Available topics: {available}"
        )

    msg_type_name = topic_types[topic_name]
    points: List[Tuple[int, float, float]] = []
    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        if topic != topic_name:
            continue
        msg = deserialize_message(data, Odometry)
        if not isinstance(msg, Odometry):
            continue
        position = msg.pose.pose.position
        points.append((int(stamp_ns), float(position.x), float(position.y)))
    if not points:
        return []
    return normalize_samples(points)


def read_odom_samples_rosbags(
    bag_path: Path, topic_name: str
) -> List[Tuple[float, float, float]]:
    from rosbags.highlevel import AnyReader

    points: List[Tuple[int, float, float]] = []
    with AnyReader([bag_path]) as reader:
        connections = [x for x in reader.connections if x.topic == topic_name]
        if not connections:
            available = ", ".join(sorted({x.topic for x in reader.connections}))
            raise RuntimeError(
                f"Topic '{topic_name}' not found in bag '{bag_path}'. Available topics: {available}"
            )
        for connection, stamp_ns, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            position = msg.pose.pose.position
            points.append((int(stamp_ns), float(position.x), float(position.y)))
    if not points:
        return []
    return normalize_samples(points)


def add_marker(
    x: float,
    y: float,
    label: str,
    color: str,
    marker: str,
) -> None:
    plt.scatter([x], [y], color=color, marker=marker, s=90, label=label, zorder=3)


def add_annotation(
    x: float,
    y: float,
    text: str,
    color: str,
    rotation: float = 0.0,
    xytext: tuple[float, float] = (6, 6),
    ha: str = "left",
) -> None:
    plt.annotate(
        text,
        (x, y),
        textcoords="offset points",
        xytext=xytext,
        fontsize=ANNOTATION_FONT_SIZE,
        color=color,
        rotation=rotation,
        ha=ha,
    )


def has_nearby_marker(
    markers: List[Dict[str, object]],
    x: float,
    y: float,
    tolerance: float = 0.75,
) -> bool:
    for marker_info in markers:
        marker_x = float(marker_info["x"])
        marker_y = float(marker_info["y"])
        if abs(marker_x - x) <= tolerance and abs(marker_y - y) <= tolerance:
            return True
    return False


def nearest_sample_at_time(
    samples: List[Tuple[float, float, float]], event_time_s: float
) -> Optional[Tuple[float, float]]:
    if not samples:
        return None
    _, x, y = min(samples, key=lambda sample: abs(sample[0] - event_time_s))
    return (x, y)


def load_json_file(path: Path) -> Optional[dict | list]:
    if not path.exists():
        return None
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def load_jsonl_file(path: Path) -> List[dict]:
    if not path.exists():
        return []
    entries: List[dict] = []
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            stripped = line.strip()
            if not stripped:
                continue
            entries.append(json.loads(stripped))
    return entries


def load_waypoints(repo_root: Path) -> Dict[str, Tuple[float, float]]:
    waypoints_path = repo_root / "src" / "spacetry_mission" / "config" / "waypoints.yaml"
    if not waypoints_path.exists():
        return {}

    import yaml

    with waypoints_path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}

    result: Dict[str, Tuple[float, float]] = {}
    for name, waypoint in (data.get("waypoints") or {}).items():
        result[name] = (float(waypoint["x"]), float(waypoint["y"]))
    return result


def load_world_landmarks(repo_root: Path) -> Dict[str, Tuple[float, float]]:
    world_path = repo_root / "src" / "spacetry_world" / "worlds" / "mars_outpost.sdf"
    if not world_path.exists():
        return {}

    content = world_path.read_text(encoding="utf-8")
    landmarks: Dict[str, Tuple[float, float]] = {}

    for name in ("block_island", "science_rock_01", "outpost_habitat_01"):
        pattern = re.compile(
            rf"<name>{re.escape(name)}</name>.*?<pose>([^<]+)</pose>",
            re.DOTALL,
        )
        match = pattern.search(content)
        if not match:
            continue
        pose_values = match.group(1).split()
        if len(pose_values) < 2:
            continue
        landmarks[name] = (float(pose_values[0]), float(pose_values[1]))

    return landmarks


def parse_world_object_poses(repo_root: Path) -> Dict[str, Dict[str, float]]:
    world_path = repo_root / "src" / "spacetry_world" / "worlds" / "mars_outpost.sdf"
    if not world_path.exists():
        return {}

    content = world_path.read_text(encoding="utf-8")
    objects: Dict[str, Dict[str, float]] = {}

    include_pattern = re.compile(
        r"<include>\s*(?:(?:<name>(?P<name>[^<]+)</name>)|(?:<uri>(?P<uri>[^<]+)</uri>)|(?:<pose>(?P<pose>[^<]+)</pose>))*.*?</include>",
        re.DOTALL,
    )
    for match in include_pattern.finditer(content):
        block = match.group(0)
        name_match = re.search(r"<name>([^<]+)</name>", block)
        pose_matches = re.findall(r"<pose>([^<]+)</pose>", block)
        if not name_match or not pose_matches:
            continue
        name = name_match.group(1).strip()
        pose_values = pose_matches[-1].split()
        if len(pose_values) < 6:
            continue
        objects[name] = {
            "x": float(pose_values[0]),
            "y": float(pose_values[1]),
            "z": float(pose_values[2]),
            "roll": float(pose_values[3]),
            "pitch": float(pose_values[4]),
            "yaw": float(pose_values[5]),
        }
    return objects


def read_stl_vertices(stl_path: Path) -> List[Tuple[float, float, float]]:
    data = stl_path.read_bytes()
    if len(data) < 84:
        return []

    face_count = struct.unpack("<I", data[80:84])[0]
    expected_size = 84 + face_count * 50
    if expected_size == len(data):
        vertices: List[Tuple[float, float, float]] = []
        offset = 84
        for _ in range(face_count):
            offset += 12  # normal
            for _ in range(3):
                x, y, z = struct.unpack("<fff", data[offset : offset + 12])
                vertices.append((x, y, z))
                offset += 12
            offset += 2  # attribute byte count
        return vertices

    text = data.decode("utf-8", errors="ignore")
    vertices = []
    for line in text.splitlines():
        stripped = line.strip()
        if not stripped.startswith("vertex "):
            continue
        _, xs, ys, zs = stripped.split()
        vertices.append((float(xs), float(ys), float(zs)))
    return vertices


def compute_block_island_footprint(repo_root: Path) -> Optional[Dict[str, object]]:
    model_path = repo_root / "src" / "spacetry_models" / "models" / "block_island" / "model.sdf"
    mesh_path = repo_root / "src" / "spacetry_models" / "models" / "block_island" / "block_island.stl"
    world_objects = parse_world_object_poses(repo_root)
    if not model_path.exists() or not mesh_path.exists() or "block_island" not in world_objects:
        return None

    model_sdf = model_path.read_text(encoding="utf-8")
    scale_match = re.search(r"<scale>\s*([^\s]+)\s+([^\s]+)\s+([^\s]+)\s*</scale>", model_sdf)
    scale_x = scale_y = 1.0
    if scale_match:
        scale_x = float(scale_match.group(1))
        scale_y = float(scale_match.group(2))

    vertices = read_stl_vertices(mesh_path)
    if not vertices:
        return None

    min_x = min(vertex[0] for vertex in vertices) * scale_x
    max_x = max(vertex[0] for vertex in vertices) * scale_x
    min_y = min(vertex[1] for vertex in vertices) * scale_y
    max_y = max(vertex[1] for vertex in vertices) * scale_y

    corners = [
        (min_x, min_y),
        (max_x, min_y),
        (max_x, max_y),
        (min_x, max_y),
    ]
    world_pose = world_objects["block_island"]
    yaw = float(world_pose["yaw"])
    cos_yaw = float(__import__("math").cos(yaw))
    sin_yaw = float(__import__("math").sin(yaw))

    transformed = []
    for local_x, local_y in corners:
        world_x = world_pose["x"] + local_x * cos_yaw - local_y * sin_yaw
        world_y = world_pose["y"] + local_x * sin_yaw + local_y * cos_yaw
        transformed.append((world_x, world_y))

    return {
        "name": "block_island",
        "polygon": transformed,
        "center": (world_pose["x"], world_pose["y"]),
    }


def scenario_root_for_bag(bag_path: Path) -> Optional[Path]:
    if bag_path.parent.name != "rosbags":
        if bag_path.parent.parent.name == "rosbags":
            return bag_path.parent.parent.parent
        return None
    return bag_path.parent.parent


def load_scenario_config(path: Path) -> Dict[str, object]:
    if not path.exists():
        return {}
    import yaml

    with path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def archived_scenario_package_root(scenario_root: Path) -> Optional[Path]:
    trace_root = scenario_root.parent.parent
    candidate = trace_root / scenario_root.name
    return candidate if candidate.exists() else None


def scenario_run_id_for_bag(bag_path: Path) -> Optional[str]:
    name = bag_path.name if bag_path.is_dir() else bag_path.stem
    if re.fullmatch(r"\d{8}T\d{6}Z", name):
        return name
    match = TIMESTAMP_SUFFIX.search(name)
    if match:
        return match.group(1)
    return None


def scenario_record_files(scenario_root: Path, run_id: Optional[str]) -> Dict[str, Path]:
    scenario_name = scenario_root.name
    runtime_dir = scenario_root / "runtime"
    metrics_dir = scenario_root / "metrics"
    if run_id:
        metrics_path = metrics_dir / f"{run_id}_metrics.json"
        timeline_json_path = runtime_dir / f"{run_id}_timeline.json"
        timeline_jsonl_path = runtime_dir / f"{run_id}_timeline.jsonl"
    else:
        metrics_path = metrics_dir / f"{scenario_name}_metrics.json"
        timeline_json_path = runtime_dir / f"{scenario_name}_timeline.json"
        timeline_jsonl_path = runtime_dir / f"{scenario_name}_timeline.jsonl"
    return {
        "metrics": metrics_path,
        "timeline_json": timeline_json_path,
        "timeline_jsonl": timeline_jsonl_path,
        "report": scenario_root / f"{scenario_name}_report.md",
    }


def load_scenario_metrics_and_timeline(
    scenario_root: Path,
    bag_path: Path,
) -> Tuple[Dict[str, object], List[dict]]:
    record_files = scenario_record_files(
        scenario_root,
        scenario_run_id_for_bag(bag_path),
    )
    metrics = load_json_file(record_files["metrics"])
    if not isinstance(metrics, dict):
        metrics = {}

    timeline_json = load_json_file(record_files["timeline_json"])
    if isinstance(timeline_json, list):
        timeline = timeline_json
    else:
        timeline = load_jsonl_file(record_files["timeline_jsonl"])
    return metrics, timeline


def build_scenario_markers(
    repo_root: Path,
    bag_path: Path,
    samples: List[Tuple[float, float, float]],
) -> List[Dict[str, object]]:
    scenario_root = scenario_root_for_bag(bag_path)
    if scenario_root is None:
        return []

    metrics, timeline = load_scenario_metrics_and_timeline(scenario_root, bag_path)
    archived_package_root = archived_scenario_package_root(scenario_root)
    scenario_config = {}
    if archived_package_root is not None:
        scenario_config = load_scenario_config(
            archived_package_root / "config" / "scenario_config.yaml"
        )

    waypoints = load_waypoints(repo_root)
    world_landmarks = load_world_landmarks(repo_root)
    markers: List[Dict[str, object]] = []
    seen_positions: set[Tuple[str, int, int]] = set()

    def append_marker(marker_info: Dict[str, object]) -> None:
        key = (
            str(marker_info["label"]),
            round(float(marker_info["x"]) * 10),
            round(float(marker_info["y"]) * 10),
        )
        if key in seen_positions:
            return
        seen_positions.add(key)
        markers.append(marker_info)

    mission = scenario_config.get("mission") if isinstance(scenario_config, dict) else None
    if isinstance(mission, dict):
        start_waypoint = mission.get("start_waypoint")
        if isinstance(start_waypoint, dict) and {"x", "y"} <= set(start_waypoint):
            append_marker(
                {
                    "x": float(start_waypoint["x"]),
                    "y": float(start_waypoint["y"]),
                    "label": "Dock Pad",
                    "color": "#2ca02c",
                    "marker": "o",
                    "annotation": str(start_waypoint.get("name", "dock_pad_01")),
                }
            )
        goal_waypoint = mission.get("goal_waypoint")
        if isinstance(goal_waypoint, dict) and {"x", "y"} <= set(goal_waypoint):
            append_marker(
                {
                    "x": float(goal_waypoint["x"]),
                    "y": float(goal_waypoint["y"]),
                    "label": "Science Target",
                    "color": "#9467bd",
                    "marker": "*",
                    "annotation": str(goal_waypoint.get("name", "science_rock_01")),
                }
            )

    start_waypoint = metrics.get("start_waypoint")
    if isinstance(start_waypoint, str) and start_waypoint in waypoints:
        x, y = waypoints[start_waypoint]
        append_marker(
            {
                "x": x,
                "y": y,
                "label": "Dock Pad",
                "color": "#2ca02c",
                "marker": "o",
                "annotation": start_waypoint,
            }
        )

    goal_waypoint = metrics.get("goal_waypoint")
    if isinstance(goal_waypoint, str) and goal_waypoint in waypoints:
        x, y = waypoints[goal_waypoint]
        append_marker(
            {
                "x": x,
                "y": y,
                "label": "Science Target",
                "color": "#9467bd",
                "marker": "*",
                "annotation": goal_waypoint,
            }
        )

    if "outpost_habitat_01" in world_landmarks:
        x, y = world_landmarks["outpost_habitat_01"]
        append_marker(
            {
                "x": x,
                "y": y,
                "label": "Outpost",
                "color": "#8c564b",
                "marker": "P",
                "annotation": "outpost_habitat_01",
            }
        )

    if "science_rock_01" in world_landmarks:
        x, y = world_landmarks["science_rock_01"]
        append_marker(
            {
                "x": x,
                "y": y,
                "label": "Science Rock",
                "color": "#9467bd",
                "marker": "*",
                "annotation": "science_rock_01",
            }
        )

    event_styles = {
        "fault_injected": ("Fault injected", "#ff7f0e", "^", "runtime rock"),
        "fault_injection_verified": ("Injection verified", "#ff7f0e", "^", None),
        "fault_encountered": ("Fault encountered", "#bcbd22", "v", None),
        "fault_detection_candidate": ("Obstacle detection", "#17becf", "s", None),
        "fault_detection_attributed": ("Detection attributed", "#2ca02c", "s", None),
        "goal_reached": ("Goal reached", "#1f77b4", "X", "goal"),
    }
    for event in timeline:
        if not isinstance(event, dict):
            continue
        event_name = event.get("event")
        if event_name not in event_styles:
            continue
        xy = None
        details_candidates = []
        details = event.get("details")
        if isinstance(details, dict):
            details_candidates.append(details)
        payload = event.get("payload")
        if isinstance(payload, dict):
            details_candidates.append(payload)
        injected_fault_pose = event.get("injected_fault_pose")
        if isinstance(injected_fault_pose, dict):
            details_candidates.append(injected_fault_pose)
        active_context = event.get("active_context_at_reaction")
        if isinstance(active_context, dict):
            details_candidates.append(active_context)

        for details_candidate in details_candidates:
            details = details_candidate
            pose = details.get("pose")
            if isinstance(pose, dict) and {"x", "y"} <= set(pose):
                xy = (float(pose["x"]), float(pose["y"]))
                break
            if {"x", "y"} <= set(details):
                xy = (float(details["x"]), float(details["y"]))
                break
            if "distance_to_fault_m" in details and event_name == "fault_encountered":
                injected_marker = next(
                    (
                        marker
                        for marker in markers
                        if str(marker["label"]) == "Injection point"
                    ),
                    None,
                )
                if injected_marker is not None:
                    time_s = event.get("sim_time_s")
                    if isinstance(time_s, (int, float)):
                        xy = nearest_sample_at_time(samples, float(time_s))
                        break
        if xy is None:
            time_s = event.get("sim_time_s")
            if not isinstance(time_s, (int, float)):
                continue
            xy = nearest_sample_at_time(samples, float(time_s))
        if xy is None:
            continue
        label, color, marker, annotation = event_styles[event_name]
        append_marker(
            {
                "x": xy[0],
                "y": xy[1],
                "label": label,
                "color": color,
                "marker": marker,
                "annotation": annotation if event_name != "fault_injected" else "runtime rock",
            }
        )
    return markers


def draw_world_context(repo_root: Path) -> None:
    footprint = compute_block_island_footprint(repo_root)
    if footprint is not None:
        polygon = Polygon(
            footprint["polygon"],
            closed=True,
            facecolor="#c49c94",
            edgecolor="#8c564b",
            linewidth=1.5,
            alpha=0.35,
            label="block_island",
            zorder=1,
        )
        plt.gca().add_patch(polygon)
        center_x, center_y = footprint["center"]
        add_annotation(
            center_x,
            center_y,
            "block_island",
            "#8c564b",
            rotation=90.0,
            xytext=(6, -12),
        )

    world_landmarks = load_world_landmarks(repo_root)
    for name, (x, y) in world_landmarks.items():
        if name == "block_island":
            continue
        if name == "outpost_habitat_01":
            patch = Circle((x, y), radius=2.0, facecolor="#8c564b", alpha=0.15, edgecolor="#8c564b", linewidth=1.2, zorder=1)
            plt.gca().add_patch(patch)
        elif name == "science_rock_01":
            patch = Ellipse((x, y), width=2.6, height=2.0, angle=0.0, facecolor="#9467bd", alpha=0.12, edgecolor="#9467bd", linewidth=1.2, zorder=1)
            plt.gca().add_patch(patch)


def scenario_summary_lines(
    bag_path: Path,
    repo_root: Path,
) -> List[str]:
    scenario_root = scenario_root_for_bag(bag_path)
    if scenario_root is None:
        return []
    metrics, _timeline = load_scenario_metrics_and_timeline(scenario_root, bag_path)
    if not metrics:
        return []

    lines = [
        f"Scenario: {scenario_root.name}",
        f"Outcome: {metrics.get('outcome_assessment', 'n/a')}",
        f"Termination: {metrics.get('termination_reason', 'n/a')}",
        f"Elapsed: {metrics.get('elapsed_s', 'n/a')} s",
        f"Injection success: {metrics.get('injection_success', 'n/a')}",
        f"Encountered: {metrics.get('injected_uncertainty_encounter_status', 'n/a')}",
        f"Reaction attributed: {metrics.get('reaction_attribution_status', 'n/a')}",
        f"Detection attributed: {metrics.get('detection_attribution_status', 'n/a')}",
        f"Route deviation: {metrics.get('route_deviation_m', 'n/a')} m",
        f"Collision proxy min dist: {metrics.get('collision_proxy_min_distance_m', 'n/a')} m",
    ]

    safety = metrics.get("safety_preservation")
    if isinstance(safety, dict):
        lines.extend(
            [
                f"MR_009: {safety.get('MR_009_RETURN_TO_OUTPOST_ON_LOW_BATTERY', 'n/a')}",
                f"MR_011: {safety.get('MR_011_MAINTAIN_SPEED_WHEN_FULL_BATTERY', 'n/a')}",
                f"Collision proxy safe: {safety.get('collision_with_dynamic_obstacle', 'n/a')}",
            ]
        )

    goals = metrics.get("goal_viability")
    if isinstance(goals, dict):
        lines.extend(
            [
                f"science_rock_01 reached: {goals.get('science_rock_01_reached', 'n/a')}",
                f"mission deadline met: {goals.get('mission_deadline_met', 'n/a')}",
            ]
        )

    return lines
def plot_path(
    points: List[Tuple[float, float]],
    output_path: Path,
    title: str,
    start_label: str,
    end_label: str,
    injection_point: Optional[Tuple[float, float]],
    repo_root: Path,
    bag_path: Path,
    scenario_markers: Optional[List[Dict[str, object]]] = None,
) -> None:
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    resolved_markers = scenario_markers or []

    fig, ax = plt.subplots(figsize=(10, 7.2))
    plt.sca(ax)
    draw_world_context(repo_root)
    ax.plot(xs, ys, linewidth=2.0, color="#1f77b4", label="Rover path")
    if not has_nearby_marker(resolved_markers, xs[0], ys[0]):
        add_marker(xs[0], ys[0], start_label, "#2ca02c", "o")
        add_annotation(xs[0], ys[0], "rover", "#2ca02c")
    add_marker(xs[-1], ys[-1], end_label, "#d62728", "X")

    if injection_point is not None:
        add_marker(
            injection_point[0],
            injection_point[1],
            "Injection point",
            "#ff7f0e",
            "^",
        )

    seen_labels = {
        start_label,
        end_label,
    }
    if injection_point is not None:
        seen_labels.add("Injection point")

    for marker_info in resolved_markers:
        label = str(marker_info["label"])
        add_marker(
            float(marker_info["x"]),
            float(marker_info["y"]),
            label if label not in seen_labels else "_nolegend_",
            str(marker_info["color"]),
            str(marker_info["marker"]),
        )
        annotation = marker_info.get("annotation")
        if isinstance(annotation, str) and annotation:
            annotation_xytext = (6, 6)
            annotation_ha = "left"
            if annotation in {"outpost_habitat_01", "science_rock_01"}:
                annotation_xytext = (-8, 4)
                annotation_ha = "right"
            add_annotation(
                float(marker_info["x"]),
                float(marker_info["y"]),
                annotation,
                str(marker_info["color"]),
                xytext=annotation_xytext,
                ha=annotation_ha,
            )
        seen_labels.add(label)

    ax.set_xlabel("X [m]", fontsize=AXIS_LABEL_FONT_SIZE)
    ax.set_ylabel("Y [m]", fontsize=AXIS_LABEL_FONT_SIZE)
    ax.set_title(title, fontsize=TITLE_FONT_SIZE)
    current_right = ax.get_xlim()[1]
    current_bottom, current_top = ax.get_ylim()
    ax.set_xlim(left=-2.0, right=max(70.0, current_right))
    ax.set_ylim(bottom=current_bottom, top=current_top)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    ax.tick_params(axis="both", labelsize=TICK_LABEL_FONT_SIZE)

    world_landmarks = load_world_landmarks(repo_root)
    outpost_xy = world_landmarks.get("outpost_habitat_01")
    science_xy = world_landmarks.get("science_rock_01")
    legend_kwargs = {
        "fontsize": LEGEND_FONT_SIZE,
        "framealpha": 0.95,
        "borderpad": 0.8,
        "labelspacing": 0.6,
        "handlelength": 1.8,
    }
    if outpost_xy is not None and science_xy is not None:
        legend_kwargs.update(
            {
                "loc": "center",
                "bbox_to_anchor": (
                    ((outpost_xy[0] + science_xy[0]) / 2.0) - 3.0,
                    (outpost_xy[1] + science_xy[1]) / 2.0,
                ),
                "bbox_transform": ax.transData,
            }
        )
    else:
        legend_kwargs["loc"] = "upper left"
    ax.legend(**legend_kwargs)

    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def is_bag_directory(path: Path) -> bool:
    return path.is_dir() and (
        (path / "metadata.yaml").exists() or any(path.glob("*.mcap"))
    )


def candidate_bag_directories(trace_path: Path) -> List[Path]:
    if is_bag_directory(trace_path):
        return [trace_path]

    records_dir = trace_path / "records"
    search_root = records_dir if records_dir.is_dir() else trace_path

    candidates = [path for path in search_root.rglob("*") if is_bag_directory(path)]
    return sorted(set(candidates))


def bag_sort_key(path: Path) -> Tuple[int, str]:
    match = TIMESTAMP_SUFFIX.search(path.name)
    if match:
        return (1, match.group(1))
    try:
        return (0, f"{path.stat().st_mtime_ns:020d}")
    except FileNotFoundError:
        return (0, "0")


def choose_latest_bag(trace_path: Path) -> Path:
    candidates = candidate_bag_directories(trace_path)
    if not candidates:
        raise RuntimeError(
            f"No rosbag directory found under '{trace_path}'. "
            "Expected a bag directory or a trace folder containing records/*/rosbags/*."
        )
    return sorted(candidates, key=bag_sort_key)[-1]


def find_repo_root() -> Path:
    return Path(__file__).resolve().parent.parent


def to_container_path(path: Path, repo_root: Path) -> Path:
    resolved = path.resolve()
    try:
        relative = resolved.relative_to(repo_root)
    except ValueError as exc:
        raise RuntimeError(
            f"Path '{resolved}' must be inside the repository root '{repo_root}'."
        ) from exc
    return Path("/ws") / relative


def run_in_docker(
    trace_path: Path,
    output_path: Optional[Path],
    title: str,
    start_label: str,
    end_label: str,
    injection_point: Optional[Tuple[float, float]],
    topic: str,
) -> None:
    repo_root = find_repo_root()
    bag_path = choose_latest_bag(trace_path)
    resolved_output = output_path or (bag_path / "navigation_2d.png")

    bag_path_in_container = to_container_path(bag_path, repo_root)
    output_path_in_container = to_container_path(resolved_output, repo_root)
    script_in_container = Path("/ws/scripts/plot_nav_2d.py")

    inner_command = [
        "python3",
        shlex.quote(str(script_in_container)),
        shlex.quote(str(bag_path_in_container)),
        "--internal",
        "--topic",
        shlex.quote(topic),
        "--output",
        shlex.quote(str(output_path_in_container)),
        "--title",
        shlex.quote(title),
        "--start-label",
        shlex.quote(start_label),
        "--end-label",
        shlex.quote(end_label),
    ]
    if injection_point is not None:
        inner_command.extend(
            [
                "--injection-x",
                str(injection_point[0]),
                "--injection-y",
                str(injection_point[1]),
            ]
        )

    docker_command = [
        "docker",
        "run",
        "--rm",
        "--platform",
        "linux/amd64",
        "-v",
        f"{repo_root}:/ws",
        "-w",
        "/ws",
        "spacetry:dev",
        "bash",
        "-lc",
        (
            "source /opt/ros/spaceros/setup.bash && "
            "source /etc/profile && "
            + " ".join(inner_command)
        ),
    ]

    print(f"Using rosbag: {bag_path}")
    subprocess.run(docker_command, check=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Generate a 2D rover trajectory plot from a SpaceTry trace folder. "
            "The plotting step runs inside the Docker image."
        )
    )
    parser.add_argument(
        "input_path",
        type=Path,
        help=(
            "Path to a trace/log folder such as logs/icra_isrs_26/trace0. "
            "In internal mode this may also be a direct rosbag directory."
        ),
    )
    parser.add_argument(
        "--topic",
        default=ODOM_TOPIC,
        help=f"Odometry topic to plot. Default: {ODOM_TOPIC}",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output PNG path. Defaults to <bag_path>/navigation_2d.png",
    )
    parser.add_argument(
        "--title",
        default="Runtime Obstacle Response Scenario",
        help="Plot title",
    )
    parser.add_argument("--start-label", default="Start")
    parser.add_argument("--end-label", default="Final Pose")
    parser.add_argument("--injection-x", type=float, default=None)
    parser.add_argument("--injection-y", type=float, default=None)
    parser.add_argument(
        "--internal",
        action="store_true",
        help=argparse.SUPPRESS,
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    repo_root = find_repo_root()
    injection_point = None
    if args.injection_x is not None and args.injection_y is not None:
        injection_point = (args.injection_x, args.injection_y)

    if not args.internal:
        run_in_docker(
            trace_path=args.input_path,
            output_path=args.output,
            title=args.title,
            start_label=args.start_label,
            end_label=args.end_label,
            injection_point=injection_point,
            topic=args.topic,
        )
        return

    bag_path = args.input_path
    output_path = args.output or (bag_path / "navigation_2d.png")
    samples = read_odom_samples(bag_path, args.topic)
    points = [(x, y) for _, x, y in samples]
    if not points:
        raise RuntimeError(
            f"No odometry samples found on topic '{args.topic}' in bag '{bag_path}'."
        )

    scenario_markers = build_scenario_markers(repo_root, bag_path, samples)

    plot_path(
        points=points,
        output_path=output_path,
        title=args.title,
        start_label=args.start_label,
        end_label=args.end_label,
        injection_point=injection_point,
        repo_root=repo_root,
        bag_path=bag_path,
        scenario_markers=scenario_markers,
    )
    print(f"Wrote 2D navigation plot to: {output_path}")


if __name__ == "__main__":
    main()
