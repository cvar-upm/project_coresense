#!/usr/bin/env python3
"""
Visualize a CoreSense missions_json file with greedy-sequential auction simulation.

Each waypoint is colored by the drone that the greedy-sequential algorithm assigns it to,
and arrows show the traversal order per drone (path built greedily from last assigned point).

Usage:
    python3 visualize_mission.py missions_json/begin3_mission.json -w config/world_swarm.yaml
    python3 visualize_mission.py missions_json/begin5_mission.json -w config/world5drones.yaml -o out.png
"""

import argparse
import json
import math
import re
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.patheffects as pe
import numpy as np
import yaml
from scipy.spatial import ConvexHull


# ── Palette ───────────────────────────────────────────────────────────────────

DRONE_PALETTE = [
    '#E74C3C',  # red
    '#3498DB',  # blue
    '#2ECC71',  # green
    '#F39C12',  # orange
    '#9B59B6',  # purple
    '#1ABC9C',  # teal
]


# ── Parsing ───────────────────────────────────────────────────────────────────

def parse_world_yaml(world_path):
    """Return {drone_id: (x, y)} from a world YAML file."""
    with open(world_path) as f:
        data = yaml.safe_load(f)
    positions = {}
    for key, value in data.items():
        if isinstance(key, str) and key.startswith('drone'):
            try:
                pose = value['platform']['ros__parameters']['vehicle_initial_pose']
                positions[key] = (float(pose.get('x', 0.0)), float(pose.get('y', 0.0)))
            except (KeyError, TypeError):
                pass
    return positions


def parse_mission(mission_path):
    """Return (mission dict, bidders list, flat waypoint list [(name, x, y)])."""
    with open(mission_path) as f:
        mission = json.load(f)

    bidders, elements = [], []
    for behaviors in mission['drones'].values():
        for b in behaviors:
            if b.get('behavior') == 'auction':
                args = b['args']
                bidders = args.get('bidders', [])
                for elem in args.get('elements', []):
                    fn = elem.get('feature_names', ['x', 'y'])
                    feats = elem['features']
                    x = feats[fn.index('x')] if 'x' in fn else feats[0]
                    y = feats[fn.index('y')] if 'y' in fn else feats[1]
                    elements.append((elem['name'], float(x), float(y)))
                break
        if bidders:
            break

    return mission, bidders, elements


def parse_area_from_comment(comment):
    """Try to extract (xmin, ymin, xmax, ymax) from _comment string."""
    m = re.search(r'Area:\s*\(([^)]+)\)\s*to\s*\(([^)]+)\)', comment or '')
    if m:
        try:
            lo = [float(v) for v in m.group(1).split(',')]
            hi = [float(v) for v in m.group(2).split(',')]
            return lo[0], lo[1], hi[0], hi[1]
        except (ValueError, IndexError):
            pass
    return None


def parse_starts_from_comment(comment):
    """
    Try to extract drone starts from _comment, e.g.
    'Drone starts: drone0(-2,2) drone1(2,2)'
    Returns {drone_id: (x, y)} or {}.
    """
    positions = {}
    for m in re.finditer(r'(drone\d+)\(([^)]+)\)', comment or ''):
        drone_id = m.group(1)
        coords = [float(v) for v in m.group(2).split(',')]
        if len(coords) >= 2:
            positions[drone_id] = (coords[0], coords[1])
    return positions


# ── Greedy sequential simulation ──────────────────────────────────────────────

def simulate_greedy_sequential(drone_start_positions, waypoints, max_points=None):
    """
    Simulate the greedy-sequential coordinate_item auction.

    Algorithm:
      While unassigned items remain and at least one drone is under its cap:
        - For every (drone, item) pair where drone hasn't reached max_points,
          compute Euclidean distance from the drone's CURRENT position.
        - The globally cheapest (drone, item) pair wins.
        - Drone position advances to that item.

    Args:
        max_points: if set, each drone wins at most this many waypoints
                    (mirrors bundle_size in config/config.yaml). Points that
                    cannot be assigned because all drones are capped are
                    returned as unassigned.

    Returns:
        assignments  : {drone_id: [(name, x, y), ...]}   (in assignment order)
        unassigned   : [(name, x, y)]  — non-empty only when max_points is set
    """
    current_pos = {d: list(pos) for d, pos in drone_start_positions.items()}
    assignments = {d: [] for d in drone_start_positions}
    remaining   = list(waypoints)

    while remaining:
        # Drones still allowed to bid
        eligible = {
            d: pos for d, pos in current_pos.items()
            if max_points is None or len(assignments[d]) < max_points
        }
        if not eligible:
            break  # all drones capped — remaining points are unassigned

        best_cost     = math.inf
        best_drone    = None
        best_item_idx = None

        for i, (name, x, y) in enumerate(remaining):
            for drone_id, (dx, dy) in eligible.items():
                cost = math.hypot(x - dx, y - dy)
                if cost < best_cost:
                    best_cost     = cost
                    best_drone    = drone_id
                    best_item_idx = i

        item = remaining.pop(best_item_idx)
        name, x, y = item
        assignments[best_drone].append(item)
        current_pos[best_drone] = [x, y]

    return assignments, remaining  # remaining = unassigned


# ── Drawing helpers ───────────────────────────────────────────────────────────

def draw_convex_hull(ax, points_xy, color, alpha=0.12):
    """Draw a shaded convex hull around a set of (x, y) points."""
    pts = np.array(points_xy)
    if len(pts) < 3:
        return
    try:
        hull = ConvexHull(pts)
        verts = pts[hull.vertices]
        poly = plt.Polygon(verts, closed=True,
                           facecolor=color, alpha=alpha,
                           edgecolor=color, linewidth=1.5,
                           linestyle='--', zorder=2)
        ax.add_patch(poly)
    except Exception:
        pass


def draw_path_arrows(ax, start_pos, waypoint_sequence, color):
    """Draw arrows: start → wp[0] → wp[1] → … for one drone's route."""
    if not waypoint_sequence:
        return

    all_points = [start_pos] + [(x, y) for _, x, y in waypoint_sequence]
    for (x0, y0), (x1, y1) in zip(all_points, all_points[1:]):
        ax.annotate(
            '', xy=(x1, y1), xytext=(x0, y0),
            arrowprops=dict(
                arrowstyle='->', color=color,
                lw=1.4, connectionstyle='arc3,rad=0.08',
            ),
            zorder=3,
        )


# ── Main plot ─────────────────────────────────────────────────────────────────

def plot_mission(mission_path, world_path=None, output_path=None, max_points=None):
    mission, bidders, waypoints = parse_mission(mission_path)
    comment = mission.get('_comment', '')

    # --- Drone start positions (world YAML beats comment) ---
    drone_positions = parse_starts_from_comment(comment)
    if world_path:
        drone_positions.update(parse_world_yaml(world_path))

    # Keep only bidders that have a known start position; warn for the rest
    active_drones = {d: drone_positions[d] for d in bidders if d in drone_positions}
    missing = [d for d in bidders if d not in drone_positions]
    if missing:
        print(f"Warning: no start position for {missing}; they are excluded from simulation.")
        print("  Pass -w <world.yaml> or list starts in _comment as 'drone0(x,y) ...'")

    if not active_drones:
        print("Error: no drone positions available — cannot simulate. "
              "Provide a world YAML with -w.", file=sys.stderr)
        sys.exit(1)

    # --- Simulate auction ---
    assignments, unassigned = simulate_greedy_sequential(
        active_drones, waypoints, max_points=max_points)

    # --- Area bounding box ---
    area = parse_area_from_comment(comment)

    # --- Figure ---
    fig, ax = plt.subplots(figsize=(10, 9))
    ax.set_aspect('equal')
    ax.grid(True, linestyle='--', alpha=0.35, zorder=0)
    ax.set_xlabel('X (m)', fontsize=11)
    ax.set_ylabel('Y (m)', fontsize=11)
    cap_str = f', cap={max_points}/drone' if max_points is not None else ''
    ax.set_title(f'Mission: {Path(mission_path).stem}  '
                 f'({len(waypoints)} waypoints, {len(active_drones)} drones{cap_str})',
                 fontsize=13, fontweight='bold')

    # origin crosshairs
    ax.axhline(0, color='#CCCCCC', linewidth=0.8, zorder=1)
    ax.axvline(0, color='#CCCCCC', linewidth=0.8, zorder=1)

    # area box
    if area:
        xmin, ymin, xmax, ymax = area
        rect = mpatches.FancyBboxPatch(
            (xmin, ymin), xmax - xmin, ymax - ymin,
            boxstyle='square,pad=0', linewidth=2,
            edgecolor='#90A4AE', facecolor='#F5F8FA', zorder=0,
        )
        ax.add_patch(rect)
        ax.text(xmin + 0.05, ymax - 0.15, f'({xmin},{ymin})→({xmax},{ymax})',
                fontsize=7.5, color='#90A4AE')

    # --- Per-drone rendering ---
    all_xs, all_ys = [], []
    legend_handles = []

    for idx, drone_id in enumerate(sorted(active_drones)):
        color  = DRONE_PALETTE[idx % len(DRONE_PALETTE)]
        start  = active_drones[drone_id]
        route  = assignments.get(drone_id, [])

        all_xs.append(start[0]); all_ys.append(start[1])

        # Convex hull of assigned waypoints
        wps_xy = [(x, y) for _, x, y in route]
        draw_convex_hull(ax, wps_xy, color)

        # Path arrows (start → wp0 → wp1 → …)
        draw_path_arrows(ax, start, route, color)

        # Waypoint scatter + labels
        if wps_xy:
            xs, ys = zip(*wps_xy)
            all_xs.extend(xs); all_ys.extend(ys)
            ax.scatter(xs, ys, s=110, color=color, zorder=5,
                       edgecolors='white', linewidths=1.2)
            for order, (name, x, y) in enumerate(route, 1):
                short = name.replace('point_', '')
                ax.annotate(
                    f'{short} ({order})', (x, y),
                    textcoords='offset points', xytext=(6, 5),
                    fontsize=7.5, color=color, fontweight='bold',
                    path_effects=[pe.withStroke(linewidth=2, foreground='white')],
                )

        # Drone start marker (triangle)
        ax.plot(*start, marker='^', markersize=14, color=color,
                markeredgecolor='black', markeredgewidth=0.9, zorder=7)
        ax.annotate(
            f'{drone_id}\n({start[0]:.1f},{start[1]:.1f})', start,
            textcoords='offset points', xytext=(-4, -22),
            fontsize=8, color=color, fontweight='bold', ha='center',
            path_effects=[pe.withStroke(linewidth=2, foreground='white')],
        )

        handle = mpatches.Patch(
            color=color,
            label=f'{drone_id}  →  {len(route)} pts',
        )
        legend_handles.append(handle)

    # --- Auto-zoom ---
    if all_xs and all_ys:
        margin = max(1.0, (max(all_xs) - min(all_xs)) * 0.12)
        ax.set_xlim(min(all_xs) - margin, max(all_xs) + margin)
        ax.set_ylim(min(all_ys) - margin, max(all_ys) + margin)

    # --- Unassigned points (shown when cap is active) ---
    if unassigned:
        uxs = [x for _, x, _ in unassigned]
        uys = [y for _, _, y in unassigned]
        all_xs.extend(uxs); all_ys.extend(uys)
        ax.scatter(uxs, uys, s=110, color='#BDC3C7', zorder=5,
                   edgecolors='#7F8C8D', linewidths=1.2, marker='x')
        for name, x, y in unassigned:
            ax.annotate(
                name.replace('point_', '') + ' (—)',
                (x, y), textcoords='offset points', xytext=(6, 5),
                fontsize=7.5, color='#7F8C8D',
                path_effects=[pe.withStroke(linewidth=2, foreground='white')],
            )
        legend_handles.append(mpatches.Patch(
            facecolor='#BDC3C7', edgecolor='#7F8C8D',
            label=f'Unassigned ({len(unassigned)} pts — cap reached)',
        ))

    # --- Legend + note ---
    legend_handles.append(mpatches.Patch(
        facecolor='none', edgecolor='none',
        label='Numbers = greedy assignment order',
    ))
    ax.legend(handles=legend_handles, loc='upper left',
              fontsize=9, framealpha=0.92, edgecolor='#CCCCCC')

    plt.tight_layout()

    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"Saved → {output_path}")
    else:
        plt.show()


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Visualize a CoreSense mission JSON with greedy-sequential simulation.')
    parser.add_argument('mission',
                        help='Path to mission JSON (e.g. missions_json/begin3_mission.json)')
    parser.add_argument('-w', '--world', default=None,
                        help='World YAML for drone start positions (overrides _comment)')
    parser.add_argument('-o', '--output', default=None,
                        help='Save figure to file instead of showing interactively')
    parser.add_argument('-b', '--bundle-size', type=int, default=None,
                        metavar='N',
                        help='Max waypoints per drone (mirrors bundle_size in config.yaml). '
                             'Points left over when all drones are capped are shown as unassigned.')
    args = parser.parse_args()
    plot_mission(args.mission, args.world, args.output, max_points=args.bundle_size)


if __name__ == '__main__':
    main()
