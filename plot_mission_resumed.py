#!/usr/bin/env python3

# Copyright 2024 Universidad Politécnica de Madrid
# SPDX-License-Identifier: BSD-3-Clause

"""
plot_mission_resumed.py -- visualise the drone-failure + resume scenario.

Reads a ROS 2 bag and a progress JSON file and produces a single figure showing:
  - All drone trajectories from the bag          (solid line, semi-transparent)
  - Phase-2 follow_path plans                    (dashed arrows, visit-order labelled)
  - Each drone's start / hover position          (*)
  - Failed drone's last position                 (X marker)
  - Waypoints visited in phase 1                 (filled green)
  - Waypoints re-auctioned in phase 2            (open orange)

If no bag is available (or --static is passed) it renders the mission layout
directly from the mission file so you can preview the setup before running.

Usage
-----
    python3 plot_mission_resumed.py                         # latest bag in rosbag/rosbags/
    python3 plot_mission_resumed.py --bag /tmp/solar_bag
    python3 plot_mission_resumed.py --bag /tmp/solar_bag --progress /tmp/mission_progress.json
    python3 plot_mission_resumed.py --out figure.png
    python3 plot_mission_resumed.py --static
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

# -- Defaults ------------------------------------------------------------------
SCRIPT_DIR = Path(__file__).parent
BAG_ROOT = SCRIPT_DIR / 'rosbag' / 'rosbags'
MISSION_FILE = SCRIPT_DIR / 'begin5_mission_solar.py'
PROGRESS_FILE = Path('/tmp/mission_progress.json')

ARENA_HALF = 11.0  # solar grid spans x∈[-9,9], y∈[-6,6]

DRONE_STARTS_DEFAULT = {
    'drone0': (-9.0,  2.0),
    'drone1': (-9.0,  0.0),
    'drone2': (-9.0, -2.0),
    'drone3': (-8.0,  1.0),
    'drone4': (-8.0, -1.0),
}

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
    candidates = sorted(
        bag_root.glob('*/metadata.yaml'),
        key=lambda p: p.stat().st_mtime,
    )
    return candidates[-1].parent if candidates else None


def _storage_id(bag_path: Path) -> str:
    meta = bag_path / 'metadata.yaml'
    if meta.exists() and 'mcap' in meta.read_text():
        return 'mcap'
    return 'sqlite3'


def read_bag(bag_path: Path, drones: list) -> tuple[dict, dict]:
    """
    Parse a ROS 2 bag and return:
        poses         -- {drone: [(x, y), ...]}  (from self_localization/pose)
        planned_paths -- {drone: [[x,y,z], ...]} (last follow_path seen per drone)
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
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_opts, rosbag2_py.ConverterOptions('', ''))
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
                        planned_paths[drone] = item['args']['path']
            except (json.JSONDecodeError, KeyError, TypeError):
                pass

    return poses, planned_paths


# -- Static layout from mission file ------------------------------------------

def load_mission_points(mission_file: Path) -> list[dict]:
    spec = importlib.util.spec_from_file_location('_mission', str(mission_file))
    mod  = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return getattr(mod, 'AUCTION_POINTS', [])


# -- Plotting ------------------------------------------------------------------

def _draw_arrow(ax, x0, y0, x1, y1, color):
    mx, my = (x0 + x1) / 2, (y0 + y1) / 2
    ax.plot([x0, x1], [y0, y1], '--', color=color, linewidth=1.4, alpha=0.85, zorder=3)
    dx, dy = x1 - x0, y1 - y0
    ax.annotate(
        '', xy=(mx + dx * 0.01, my + dy * 0.01), xytext=(mx, my),
        arrowprops=dict(arrowstyle='->', color=color, lw=1.4),
        zorder=4,
    )


def plot_results(
    drones: list,
    failed_drone: str | None,
    poses: dict,
    planned_paths: dict,
    drone_starts: dict,
    all_points: list,
    visited_names: set,
    arena_half: float,
    title: str,
    output: Path | None,
) -> None:
    fig, ax = plt.subplots(figsize=(13, 11))

    # Arena boundary
    rect = plt.Rectangle(
        (-arena_half, -arena_half), 2 * arena_half, 2 * arena_half,
        linewidth=2, edgecolor='#333333', facecolor='#f5f5f5', zorder=0,
    )
    ax.add_patch(rect)

    # Grid
    ax.set_xticks(np.arange(-10, 11, 2))
    ax.set_yticks(np.arange(-8,  9,  2))
    ax.grid(True, alpha=0.25, linewidth=0.5)

    # -- Waypoint grid --------------------------------------------------------
    for pt in all_points:
        name = pt['name']
        x, y = pt['features'][0], pt['features'][1]
        if name in visited_names:
            ax.scatter(x, y, color='#2ecc71', s=60, zorder=2,
                       edgecolors='#27ae60', linewidths=0.8)
        else:
            ax.scatter(x, y, color='#fff3cd', s=60, zorder=2,
                       edgecolors='#e67e22', linewidths=1.1)
        ax.annotate(
            name.replace('panel_', ''), (x, y),
            textcoords='offset points', xytext=(3, 3),
            fontsize=6, color='#555555',
        )

    legend_handles = []

    for i, drone in enumerate(drones):
        color = COLORS[i % len(COLORS)]
        is_failed = (drone == failed_drone)

        # -- Actual trajectory ------------------------------------------------
        pts = poses.get(drone, [])
        if pts:
            xs, ys = zip(*pts)
            alpha = 0.8 if is_failed else 0.35
            ax.plot(xs, ys, '-', color=color, linewidth=1.5, alpha=alpha, zorder=2)

        # -- Phase-2 planned path with arrows ---------------------------------
        path = planned_paths.get(drone)
        if path and not is_failed:
            px = [p[0] for p in path]
            py = [p[1] for p in path]
            for j in range(len(px) - 1):
                _draw_arrow(ax, px[j], py[j], px[j + 1], py[j + 1], color)
            ax.scatter(px, py, color=color, s=90, zorder=5,
                       edgecolors='white', linewidths=0.8)
            for j, (x, y) in enumerate(zip(px, py)):
                ax.annotate(
                    str(j + 1), (x, y),
                    textcoords='offset points', xytext=(5, 5),
                    fontsize=8, color=color, fontweight='bold',
                )

        # -- Start / hover position -------------------------------------------
        sx, sy = drone_starts.get(drone, (0.0, 0.0))
        ax.scatter([sx], [sy], color=color, s=250, marker='*', zorder=6,
                   edgecolors='black', linewidths=0.6)
        ax.annotate(
            drone, (sx, sy),
            textcoords='offset points', xytext=(6, -14),
            fontsize=9, fontweight='bold', color=color,
        )

        # -- Failure marker at last known position ----------------------------
        if is_failed and pts:
            lx, ly = pts[-1]
            ax.scatter([lx], [ly], color=color, s=300, marker='X', zorder=7,
                       edgecolors='black', linewidths=1.0)
            ax.annotate(
                'FAILED', (lx, ly),
                textcoords='offset points', xytext=(8, 6),
                fontsize=8, fontweight='bold', color=color,
                bbox=dict(boxstyle='round,pad=0.2', facecolor='white',
                          edgecolor=color, alpha=0.85),
            )

        label = f'{drone} (failed)' if is_failed else drone
        legend_handles.append(mpatches.Patch(color=color, label=label))

    # Legend entries for marker styles
    legend_handles += [
        plt.Line2D([0], [0], marker='*', color='grey', linestyle='None',
                   markersize=12, label='start / hover position'),
        plt.Line2D([0], [0], marker='X', color='grey', linestyle='None',
                   markersize=10, label='failure position'),
        plt.Line2D([0], [0], marker='o', color='#2ecc71', linestyle='None',
                   markersize=8, markeredgecolor='#27ae60',
                   label='visited in phase 1'),
        plt.Line2D([0], [0], marker='o', color='#fff3cd', linestyle='None',
                   markersize=8, markeredgecolor='#e67e22', markeredgewidth=1.1,
                   label='re-auctioned in phase 2'),
        plt.Line2D([0], [0], marker='o', color='grey', linestyle='None',
                   markersize=8, label='waypoint (phase-2 visit order)'),
        plt.Line2D([0], [0], color='grey', linestyle='--', label='phase-2 planned path'),
        plt.Line2D([0], [0], color='grey', linestyle='-', alpha=0.4,
                   linewidth=2, label='actual trajectory'),
    ]

    ax.set_xlim(-arena_half - 0.5, arena_half + 0.5)
    ax.set_ylim(-arena_half + 2.0, arena_half - 2.0)
    ax.set_aspect('equal')
    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_title(title, fontsize=14, fontweight='bold', pad=12)
    ax.legend(handles=legend_handles, loc='upper right', fontsize=9, framealpha=0.9)

    plt.tight_layout()

    if output:
        plt.savefig(output, dpi=150, bbox_inches='tight')
        print(f'[plot] saved -> {output}')
    else:
        plt.show()


# -- Static mode ---------------------------------------------------------------

def static_plot(
    drones: list,
    failed_drone: str | None,
    mission_file: Path,
    drone_starts: dict,
    visited_names: set,
    arena_half: float,
    output: Path | None,
) -> None:
    """Render the mission definition without a bag."""
    points = load_mission_points(mission_file)
    if not points:
        print(f'[plot] Could not load AUCTION_POINTS from {mission_file}', file=sys.stderr)
        sys.exit(1)

    plot_results(
        drones=drones,
        failed_drone=failed_drone,
        poses={d: [] for d in drones},
        planned_paths={d: None for d in drones},
        drone_starts=drone_starts,
        all_points=points,
        visited_names=visited_names,
        arena_half=arena_half,
        title='Mission Layout — Drone Failure & Resume (static, no bag)',
        output=output,
    )


# -- Entry point ---------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description='Plot drone-failure and resume mission results'
    )
    parser.add_argument(
        '--bag', type=Path, default=None,
        help='Path to ROS 2 bag directory. Defaults to the latest bag in rosbag/rosbags/',
    )
    parser.add_argument(
        '--progress', type=Path, default=PROGRESS_FILE,
        help=f'Progress JSON written by track_progress.py (default: {PROGRESS_FILE})',
    )
    parser.add_argument(
        '--mission', type=Path, default=MISSION_FILE,
        help=f'Mission file containing AUCTION_POINTS (default: {MISSION_FILE})',
    )
    parser.add_argument(
        '--failed', default='drone2',
        help='Namespace of the drone that failed (default: drone2)',
    )
    parser.add_argument(
        '--drones', nargs='+',
        default=list(DRONE_STARTS_DEFAULT.keys()),
        help='All drone namespaces including the failed one (default: drone0..drone4)',
    )
    parser.add_argument(
        '--out', type=Path, default=None,
        help='Save figure to this path instead of showing interactively',
    )
    parser.add_argument(
        '--static', action='store_true',
        help='Render mission layout from the mission file without reading a bag',
    )
    args = parser.parse_args()

    drones = args.drones
    failed_drone = args.failed
    drone_starts = {d: DRONE_STARTS_DEFAULT.get(d, (0.0, 0.0)) for d in drones}

    # -- Load visited waypoints from progress file ----------------------------
    visited_names: set = set()
    if args.progress.exists():
        with open(args.progress) as f:
            prog = json.load(f)
        visited_names = set(prog.get('visited', []))
        print(f'[plot] progress: {len(visited_names)} visited, '
              f'{prog.get("remaining_count", "?")} remaining')
    else:
        print(f'[plot] progress file not found ({args.progress}) -- '
              'all waypoints shown as remaining', file=sys.stderr)

    # -- Static mode ----------------------------------------------------------
    if args.static:
        static_plot(
            drones=drones,
            failed_drone=failed_drone,
            mission_file=args.mission,
            drone_starts=drone_starts,
            visited_names=visited_names,
            arena_half=ARENA_HALF,
            output=args.out,
        )
        return

    # -- Bag mode -------------------------------------------------------------
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

    all_points = load_mission_points(args.mission) if args.mission.exists() else []

    plot_results(
        drones=drones,
        failed_drone=failed_drone,
        poses=poses,
        planned_paths=planned_paths,
        drone_starts=drone_starts,
        all_points=all_points,
        visited_names=visited_names,
        arena_half=ARENA_HALF,
        title=f'Drone Failure & Recovery -- {bag_path.name}  '
              f'({failed_drone} failed)',
        output=args.out,
    )


if __name__ == '__main__':
    main()
