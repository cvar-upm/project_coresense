#!/usr/bin/env python3
"""
plot_simulation.py -- visualise a multi-drone auction simulation.

Reads a ROS 2 bag and produces a matplotlib figure showing:
  - The arena boundary
  - Each drone's start position         (*)
  - Assigned waypoints, visit-order labelled  (*)
  - Planned path                        (dashed arrows)
  - Actual odometry trace               (solid line, semi-transparent)

If no bag is available (or --static is passed) it renders the mission
layout directly from begin5_mission.py so you can preview the setup
before running.

Usage
-----
    python3 plot_simulation.py                         # latest bag in rosbag/rosbags/
    python3 plot_simulation.py --bag /path/to/bag/dir
    python3 plot_simulation.py --out figure.png
    python3 plot_simulation.py --drones drone0 drone1
    python3 plot_simulation.py --static                # no bag needed
"""

__authors__ = 'Guillermo GP-Lenza'
__license__ = 'BSD-3-Clause'

import argparse
import importlib.util
import json
import sys
from pathlib import Path

import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
import yaml

# -- Defaults ------------------------------------------------------------------
SCRIPT_DIR = Path(__file__).parent
BAG_ROOT = SCRIPT_DIR / 'rosbag' / 'rosbags'
MISSION_FILE = SCRIPT_DIR / 'begin5_mission.py'
WORLD_FILE = SCRIPT_DIR / 'config' / 'world5drones_solar.yaml'

ARENA_HALF = 10.0  # arena spans [-10, 10] x [-10, 10] m

DRONE_STARTS_DEFAULT = {
    'drone0': ( -9.0,  2.0),
    'drone1': (-9.0,  0.0),
    'drone2': (-9.0, -2.0),
    'drone3': ( -8.0, 1.0),
    'drone4': ( -8.0,  -1.0),
}


def load_drone_starts(world_file: Path) -> dict:
    """Parse drone starting positions from a world YAML file.

    Expects top-level drone keys (e.g. drone0, drone1, ...) each with:
        platform.ros__parameters.vehicle_initial_pose.x / .y
    Returns {drone_name: (x, y)}.
    """
    data = yaml.safe_load(world_file.read_text())
    starts = {}
    for key, val in data.items():
        if key.startswith('/**') or not isinstance(val, dict):
            continue
        try:
            pose = val['platform']['ros__parameters']['vehicle_initial_pose']
            starts[key] = (float(pose['x']), float(pose['y']))
        except (KeyError, TypeError):
            continue
    return starts

COLORS = [
    '#1f77b4',  # blue
    '#ff7f0e',  # orange
    '#2ca02c',  # green
    '#d62728',  # red
    '#9467bd',  # purple
    '#8c564b',  # brown
    '#e377c2',  # pink
    '#7f7f7f',  # grey
]
# ------------------------------------------------------------------------------


# -- Bag reading ---------------------------------------------------------------

def _find_latest_bag(bag_root: Path) -> Path | None:
    """Return the most recently modified bag directory, or None."""
    candidates = sorted(
        bag_root.glob('*/metadata.yaml'),
        key=lambda p: p.stat().st_mtime,
    )
    return candidates[-1].parent if candidates else None


def _storage_id(bag_path: Path) -> str:
    """Read storage_identifier from metadata.yaml (sqlite3 or mcap)."""
    meta = bag_path / 'metadata.yaml'
    if meta.exists():
        text = meta.read_text()
        if 'mcap' in text:
            return 'mcap'
    return 'sqlite3'


def read_bag(bag_path: Path, drones: list) -> tuple[dict, dict]:
    """
    Parse a ROS 2 bag and return:
        poses         -- {drone: [(x, y), ...]}  (from self_localization/pose)
        planned_paths -- {drone: [[x,y,z], ...]} (from follow_path MissionUpdate)
    """
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError:
        print('[plot] rosbag2_py / rclpy not available -- cannot read bag.', file=sys.stderr)
        return {d: [] for d in drones}, {d: None for d in drones}

    pose_topics    = {f'/{d}/self_localization/pose' for d in drones}
    mission_topics = {f'/{d}/mission_update'         for d in drones}

    poses         = {d: [] for d in drones}
    planned_paths = {d: None for d in drones}

    storage_opts = rosbag2_py.StorageOptions(
        uri=str(bag_path), storage_id=_storage_id(bag_path)
    )
    conv_opts = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_opts, conv_opts)

    type_map = {info.name: info.type for info in reader.get_all_topics_and_types()}

    while reader.has_next():
        topic, raw, _ = reader.read_next()

        if topic in pose_topics:
            drone = topic.split('/')[1]
            if topic not in type_map:
                continue
            msg = deserialize_message(raw, get_message(type_map[topic]))
            poses[drone].append((msg.pose.position.x, msg.pose.position.y))

        elif topic in mission_topics:
            drone = topic.split('/')[1]
            if topic not in type_map:
                continue
            msg = deserialize_message(raw, get_message(type_map[topic]))
            try:
                mission = json.loads(msg.mission)
                for item in mission.get('plan', []):
                    if item.get('behavior') == 'follow_path':
                        # Take the last follow_path seen (most complete assignment)
                        planned_paths[drone] = item['args']['path']
            except (json.JSONDecodeError, KeyError, TypeError):
                pass

    return poses, planned_paths


# -- Static layout from begin5_mission.py -------------------------------------

def load_mission_points(mission_file: Path) -> list[dict]:
    """Import AUCTION_POINTS from begin5_mission.py without executing main()."""
    spec = importlib.util.spec_from_file_location('begin5_mission', mission_file)
    mod  = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return getattr(mod, 'AUCTION_POINTS', [])


# -- Plotting ------------------------------------------------------------------

def _draw_arrow(ax, x0, y0, x1, y1, color):
    """Draw a line segment with an arrowhead at the midpoint."""
    mx, my = (x0 + x1) / 2, (y0 + y1) / 2
    # Segment
    ax.plot([x0, x1], [y0, y1], '--', color=color, linewidth=1.4, alpha=0.85, zorder=3)
    # Arrowhead at midpoint
    dx, dy = x1 - x0, y1 - y0
    ax.annotate(
        '', xy=(mx + dx * 0.01, my + dy * 0.01), xytext=(mx, my),
        arrowprops=dict(arrowstyle='->', color=color, lw=1.4),
        zorder=4,
    )


def plot_results(
    drones: list,
    poses: dict,
    planned_paths: dict,
    drone_starts: dict,
    arena_half: float,
    title: str,
    output: Path | None,
) -> None:
    fig, ax = plt.subplots(figsize=(11, 11))

    # Arena
    rect = plt.Rectangle(
        (-arena_half, -arena_half), 2 * arena_half, 2 * arena_half,
        linewidth=2, edgecolor='#333333', facecolor='#f5f5f5', zorder=0,
    )
    ax.add_patch(rect)

    # Grid
    ax.set_xticks(np.arange(-arena_half, arena_half + 1, 2))
    ax.set_yticks(np.arange(-arena_half, arena_half + 1, 2))
    ax.grid(True, alpha=0.25, linewidth=0.5)

    legend_handles = []

    for drone in drones:
        # Derive a stable color index from the trailing digits in the drone name
        # (e.g. "drone0" → 0, "drone1" → 1) so the mapping never shifts when
        # a subset of drones is plotted.
        digits = ''.join(c for c in drone if c.isdigit())
        color_idx = int(digits) if digits else hash(drone)
        color = COLORS[color_idx % len(COLORS)]

        # -- Actual trajectory ----------------------------------------------
        pts = poses.get(drone, [])
        if pts:
            xs, ys = zip(*pts)
            ax.plot(xs, ys, '-', color=color, linewidth=1.5, alpha=0.35, zorder=2)
            # Mark last recorded position (where the drone stopped) with an X
            ax.scatter([xs[-1]], [ys[-1]], color=color, s=200, marker='X',
                       zorder=7, edgecolors='black', linewidths=0.8)

        # -- Planned path with arrows ---------------------------------------
        path = planned_paths.get(drone)
        if path:
            px = [p[0] for p in path]
            py = [p[1] for p in path]

            for j in range(len(px) - 1):
                _draw_arrow(ax, px[j], py[j], px[j + 1], py[j + 1], color)

            # Waypoints
            ax.scatter(px, py, color=color, s=90, zorder=5,
                       edgecolors='white', linewidths=0.8)
            for j, (x, y) in enumerate(zip(px, py)):
                ax.annotate(
                    str(j + 1), (x, y),
                    textcoords='offset points', xytext=(5, 5),
                    fontsize=8, color=color, fontweight='bold',
                )

        # -- Start position -------------------------------------------------
        sx, sy = drone_starts.get(drone, (0.0, 0.0))
        ax.scatter([sx], [sy], color=color, s=250, marker='*', zorder=6,
                   edgecolors='black', linewidths=0.6)
        ax.annotate(
            drone, (sx, sy),
            textcoords='offset points', xytext=(6, -14),
            fontsize=9, fontweight='bold', color=color,
        )

        legend_handles.append(
            mpatches.Patch(color=color, label=drone)
        )

    # Legend entries for marker styles
    legend_handles += [
        plt.Line2D([0], [0], marker='*', color='grey', linestyle='None',
                   markersize=12, label='start position'),
        plt.Line2D([0], [0], marker='X', color='grey', linestyle='None',
                   markersize=10, label='stop position'),
        plt.Line2D([0], [0], marker='o', color='grey', linestyle='None',
                   markersize=8, label='waypoint (planned order)'),
        plt.Line2D([0], [0], color='grey', linestyle='--', label='planned path'),
        plt.Line2D([0], [0], color='grey', linestyle='-', alpha=0.4,
                   linewidth=2, label='actual trajectory'),
    ]

    ax.set_xlim(-arena_half - 1.5, arena_half + 1.5)
    ax.set_ylim(-arena_half - 1.5, arena_half + 1.5)
    ax.set_aspect('equal')
    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_title(title, fontsize=14, fontweight='bold', pad=12)
    ax.legend(handles=legend_handles, loc='upper right', fontsize=9,
              framealpha=0.9)

    plt.tight_layout()

    if output:
        plt.savefig(output, dpi=150, bbox_inches='tight')
        print(f'[plot] saved -> {output}')
    else:
        plt.show()


# -- Static mode: layout from mission file -------------------------------------

def static_plot(drones: list, mission_file: Path, drone_starts: dict,
                arena_half: float, output: Path | None) -> None:
    """Render the mission definition (no bag needed)."""
    points = load_mission_points(mission_file)
    if not points:
        print(f'[plot] Could not load AUCTION_POINTS from {mission_file}', file=sys.stderr)
        sys.exit(1)

    # Build a fake "planned_paths" assigning points to the nearest drone start
    # so the figure renders something meaningful even before the auction runs.
    assigned: dict = {d: [] for d in drones}
    starts_arr = np.array([drone_starts[d] for d in drones])

    for pt in points:
        x, y = pt['features'][0], pt['features'][1]
        dists = np.linalg.norm(starts_arr - np.array([x, y]), axis=1)
        nearest = drones[int(np.argmin(dists))]
        assigned[nearest].append([x, y, 1.0])

    # Sort each drone's points by nearest-neighbour from its start
    planned: dict = {}
    for drone, pts in assigned.items():
        if not pts:
            planned[drone] = []
            continue
        sx, sy = drone_starts[drone]
        remaining = pts.copy()
        ordered = []
        cur = [sx, sy, 1.0]
        while remaining:
            closest = min(remaining,
                          key=lambda p: (p[0] - cur[0])**2 + (p[1] - cur[1])**2)
            ordered.append(closest)
            remaining.remove(closest)
            cur = closest
        planned[drone] = ordered

    plot_results(
        drones=drones,
        poses={d: [] for d in drones},
        planned_paths=planned,
        drone_starts=drone_starts,
        arena_half=arena_half,
        title='Mission Layout (static -- no bag)',
        output=output,
    )


# -- Entry point ---------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description='Plot auction simulation results')
    parser.add_argument(
        '--bag', type=Path, default=None,
        help='Path to ROS 2 bag directory. Defaults to the latest bag in rosbag/rosbags/',
    )
    parser.add_argument(
        '--drones', nargs='+',
        default=list(DRONE_STARTS_DEFAULT.keys()),
        help='Drone namespaces to include (default: drone0..drone4)',
    )
    parser.add_argument(
        '--out', type=Path, default=None,
        help='Save figure to this path instead of showing interactively',
    )
    parser.add_argument(
        '--static', action='store_true',
        help='Render mission layout from begin5_mission.py without reading a bag',
    )
    parser.add_argument(
        '--mission', type=Path, default=MISSION_FILE,
        help=f'Mission file for --static mode (default: {MISSION_FILE})',
    )
    parser.add_argument(
        '--world', type=Path, default=WORLD_FILE,
        help=f'World YAML file with drone starting positions (default: {WORLD_FILE})',
    )
    args = parser.parse_args()

    # Load drone starts from world file if it exists, else fall back to defaults
    if args.world and args.world.exists():
        world_starts = load_drone_starts(args.world)
    else:
        if args.world:
            print(f'[plot] World file not found: {args.world} -- using defaults', file=sys.stderr)
        world_starts = DRONE_STARTS_DEFAULT

    # If --drones was not explicitly set, derive the list from the world file
    if args.drones == list(DRONE_STARTS_DEFAULT.keys()) and world_starts is not DRONE_STARTS_DEFAULT:
        drones = sorted(world_starts.keys())
    else:
        drones = args.drones

    drone_starts = {d: world_starts.get(d, DRONE_STARTS_DEFAULT.get(d, (0.0, 0.0))) for d in drones}

    # -- Static mode -----------------------------------------------------------
    if args.static:
        static_plot(drones, args.mission, drone_starts, ARENA_HALF, args.out)
        return

    # -- Bag mode --------------------------------------------------------------
    bag_path = args.bag
    if bag_path is None:
        bag_path = _find_latest_bag(BAG_ROOT)
        if bag_path is None:
            print(
                '[plot] No bag found. Use --bag <path> or --static to render '
                'the mission layout without a bag.',
                file=sys.stderr,
            )
            sys.exit(1)
        print(f'[plot] using bag: {bag_path}')

    poses, planned_paths = read_bag(bag_path, drones)

    any_path = any(v for v in planned_paths.values())
    if not any_path:
        print('[plot] No follow_path missions found in bag. '
              'Showing actual trajectories only.')

    plot_results(
        drones=drones,
        poses=poses,
        planned_paths=planned_paths,
        drone_starts=drone_starts,
        arena_half=ARENA_HALF,
        title=f'Auction Simulation -- {bag_path.name}',
        output=args.out,
    )


if __name__ == '__main__':
    main()
