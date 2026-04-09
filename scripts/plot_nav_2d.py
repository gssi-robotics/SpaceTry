#!/usr/bin/env python3
import argparse
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


ODOM_TOPIC = "/mobile_base_controller/odom"
TIMESTAMP_SUFFIX = re.compile(r"_(\d{8}T\d{6}Z)$")


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


def add_annotation(x: float, y: float, text: str, color: str) -> None:
    plt.annotate(
        text,
        (x, y),
        textcoords="offset points",
        xytext=(6, 6),
        fontsize=9,
        color=color,
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


def scenario_root_for_bag(bag_path: Path) -> Optional[Path]:
    if bag_path.parent.name != "rosbags":
        if bag_path.parent.parent.name == "rosbags":
            return bag_path.parent.parent.parent
        return None
    return bag_path.parent.parent


def build_scenario_markers(
    repo_root: Path,
    bag_path: Path,
    samples: List[Tuple[float, float, float]],
) -> List[Dict[str, object]]:
    scenario_root = scenario_root_for_bag(bag_path)
    if scenario_root is None:
        return []

    scenario_name = scenario_root.name
    metrics = load_json_file(scenario_root / "metrics" / f"{scenario_name}_metrics.json")
    timeline = load_json_file(scenario_root / "runtime" / f"{scenario_name}_timeline.json")
    if not isinstance(metrics, dict):
        metrics = {}
    if not isinstance(timeline, list):
        timeline = []

    waypoints = load_waypoints(repo_root)
    world_landmarks = load_world_landmarks(repo_root)
    markers: List[Dict[str, object]] = []

    start_waypoint = metrics.get("start_waypoint")
    if isinstance(start_waypoint, str) and start_waypoint in waypoints:
        x, y = waypoints[start_waypoint]
        markers.append(
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
        markers.append(
            {
                "x": x,
                "y": y,
                "label": "Science Target",
                "color": "#9467bd",
                "marker": "*",
                "annotation": goal_waypoint,
            }
        )

    obstacle_xy = metrics.get("obstacle_xy")
    if isinstance(obstacle_xy, dict) and {"x", "y"} <= set(obstacle_xy):
        markers.append(
            {
                "x": float(obstacle_xy["x"]),
                "y": float(obstacle_xy["y"]),
                "label": "Injection point",
                "color": "#ff7f0e",
                "marker": "^",
                "annotation": "runtime rock",
            }
        )

    if "block_island" in world_landmarks:
        x, y = world_landmarks["block_island"]
        markers.append(
            {
                "x": x,
                "y": y,
                "label": "Baseline hazard",
                "color": "#7f7f7f",
                "marker": "v",
                "annotation": "block_island",
            }
        )

    event_styles = {
        "obstacle_injection_triggered": ("Trigger point", "#8c564b", "P", "trigger"),
        "motion_reaction_detected": ("Motion reaction", "#e377c2", "D", "reaction"),
        "obstacle_detected": ("Obstacle detected", "#17becf", "s", "detected"),
    }
    for event in timeline:
        if not isinstance(event, dict):
            continue
        event_name = event.get("event")
        if event_name not in event_styles:
            continue
        time_s = event.get("time_s")
        if not isinstance(time_s, (int, float)):
            continue
        xy = nearest_sample_at_time(samples, float(time_s))
        if xy is None:
            continue
        label, color, marker, annotation = event_styles[event_name]
        markers.append(
            {
                "x": xy[0],
                "y": xy[1],
                "label": label,
                "color": color,
                "marker": marker,
                "annotation": annotation,
            }
        )

    return markers


def plot_path(
    points: List[Tuple[float, float]],
    output_path: Path,
    title: str,
    start_label: str,
    end_label: str,
    injection_point: Optional[Tuple[float, float]],
    scenario_markers: Optional[List[Dict[str, object]]] = None,
) -> None:
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    resolved_markers = scenario_markers or []

    plt.figure(figsize=(10, 8))
    plt.plot(xs, ys, linewidth=2.0, color="#1f77b4", label="Rover path")
    if not has_nearby_marker(resolved_markers, xs[0], ys[0]):
        add_marker(xs[0], ys[0], start_label, "#2ca02c", "o")
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
            add_annotation(
                float(marker_info["x"]),
                float(marker_info["y"]),
                annotation,
                str(marker_info["color"]),
            )
        seen_labels.add(label)

    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title(title)
    plt.axis("equal")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(output_path, dpi=160)
    plt.close()


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
        default="Autonomy Evaluation: Runtime Obstacle Response Against Baseline Hazards",
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
        scenario_markers=scenario_markers,
    )
    print(f"Wrote 2D navigation plot to: {output_path}")


if __name__ == "__main__":
    main()
