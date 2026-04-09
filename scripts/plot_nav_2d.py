#!/usr/bin/env python3
import argparse
import os
import re
import shlex
import subprocess
from pathlib import Path
from typing import List, Optional, Tuple

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


def read_odom_points(bag_path: Path, topic_name: str) -> List[Tuple[float, float]]:
    read_errors = []

    try:
        return read_odom_points_rosbag2_py(bag_path, topic_name)
    except Exception as exc:
        read_errors.append(f"rosbag2_py path failed: {exc}")

    try:
        return read_odom_points_rosbags(bag_path, topic_name)
    except Exception as exc:
        read_errors.append(f"rosbags path failed: {exc}")

    raise RuntimeError(
        "Unable to read the rosbag on this machine. "
        "Install either ROS 2 Python bag libraries or the pure-Python "
        "`rosbags` package, then retry.\n"
        + "\n".join(read_errors)
    )


def read_odom_points_rosbag2_py(
    bag_path: Path, topic_name: str
) -> List[Tuple[float, float]]:
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from rosidl_runtime_py.utilities import get_message

    storage_options = StorageOptions(uri=str(bag_path), storage_id="mcap")
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

    msg_type = get_message(topic_types[topic_name])
    points: List[Tuple[float, float]] = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != topic_name:
            continue
        msg = deserialize_message(data, msg_type)
        if not isinstance(msg, Odometry):
            continue
        position = msg.pose.pose.position
        points.append((float(position.x), float(position.y)))
    return points


def read_odom_points_rosbags(
    bag_path: Path, topic_name: str
) -> List[Tuple[float, float]]:
    from rosbags.highlevel import AnyReader

    points: List[Tuple[float, float]] = []
    with AnyReader([bag_path]) as reader:
        connections = [x for x in reader.connections if x.topic == topic_name]
        if not connections:
            available = ", ".join(sorted({x.topic for x in reader.connections}))
            raise RuntimeError(
                f"Topic '{topic_name}' not found in bag '{bag_path}'. Available topics: {available}"
            )

        for connection, _, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            position = msg.pose.pose.position
            points.append((float(position.x), float(position.y)))
    return points


def add_marker(
    x: float,
    y: float,
    label: str,
    color: str,
    marker: str,
) -> None:
    plt.scatter([x], [y], color=color, marker=marker, s=90, label=label, zorder=3)


def plot_path(
    points: List[Tuple[float, float]],
    output_path: Path,
    title: str,
    start_label: str,
    end_label: str,
    injection_point: Optional[Tuple[float, float]],
) -> None:
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]

    plt.figure(figsize=(10, 8))
    plt.plot(xs, ys, linewidth=2.0, color="#1f77b4", label="Rover path")
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
        default="Rover 2D Navigation Path",
        help="Plot title",
    )
    parser.add_argument("--start-label", default="Start")
    parser.add_argument("--end-label", default="End")
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
    points = read_odom_points(bag_path, args.topic)
    if not points:
        raise RuntimeError(
            f"No odometry samples found on topic '{args.topic}' in bag '{bag_path}'."
        )

    plot_path(
        points=points,
        output_path=output_path,
        title=args.title,
        start_label=args.start_label,
        end_label=args.end_label,
        injection_point=injection_point,
    )
    print(f"Wrote 2D navigation plot to: {output_path}")


if __name__ == "__main__":
    main()
