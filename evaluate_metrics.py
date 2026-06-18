#!/usr/bin/env python3
"""
evaluate_metrics.py -- compute mission performance metrics from a ROS 2 bag.

Metrics
-------
  Per-drone:
    - Path length          total distance flown (m)
    - Tracking error       mean deviation from motion reference (m)
    - Tasks assigned       waypoints dispatched via mission_update
    - Tasks visited        waypoints the drone came within VISIT_THRESHOLD of
    - Replan times         list of inter-update intervals (s)

  Swarm-level:
    - Makespan             wall-clock duration of the full mission (s)
    - Capacity utilization (visited waypoints / makespan) / (assigned waypoints / makespan)
                           = total_visited / total_assigned
    - Load balance         Jain's fairness index on path lengths (0–1)
    - Auction convergence  mean time from auctionStatus started to completed KB facts (s)

Usage
-----
    python3 evaluate_metrics.py                          # latest bag
    python3 evaluate_metrics.py --bag /path/to/bag/dir
    python3 evaluate_metrics.py --drones drone0 drone1
    python3 evaluate_metrics.py --world config/world5drones_solar.yaml
"""

__authors__ = 'Guillermo GP-Lenza'
__license__ = 'BSD-3-Clause'

import argparse
import json
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import yaml

SCRIPT_DIR = Path(__file__).parent
BAG_ROOT = SCRIPT_DIR / 'rosbag' / 'rosbags'
WORLD_FILE = SCRIPT_DIR / 'config' / 'world5drones_solar.yaml'

DRONE_STARTS_DEFAULT = {
    'drone0': (-9.0,  2.0),
    'drone1': (-9.0,  0.0),
    'drone2': (-9.0, -2.0),
    'drone3': (-8.0,  1.0),
    'drone4': (-8.0, -1.0),
}

# Drone considered to have visited a waypoint when within this radius (m)
# Mirrors the go_to_threshold used in plot_simulation_goto.py
VISIT_THRESHOLD = 0.4


# ---------------------------------------------------------------------------
# Data container
# ---------------------------------------------------------------------------

@dataclass
class DroneRecord:
    ns: str
    # (timestamp_s, x, y, z)
    poses: List[Tuple[float, float, float, float]] = field(default_factory=list)
    # (timestamp_s, ref_x, ref_y, ref_z)
    ref_poses: List[Tuple[float, float, float, float]] = field(default_factory=list)
    # timestamps of each mission_update message
    mission_update_times: List[float] = field(default_factory=list)
    # (timestamp_s, action) for each mission_update, action per as2_msgs/MissionUpdate
    # (EXECUTE=0, PAUSE=3, STOP=5, ...)
    mission_update_actions: List[Tuple[float, int]] = field(default_factory=list)
    # accumulated planned waypoints as [x, y, z], deduplicated by rounded position
    planned_waypoints: List[List[float]] = field(default_factory=list)
    # (timestamp_s, predicate, object) triples from kb/add_fact, e.g.
    # ('auctionStatus', 'started') or ('droneStatus', 'failed')
    kb_events: List[Tuple[float, str, str]] = field(default_factory=list)


# ---------------------------------------------------------------------------
# Bag helpers (mirrors existing plot_simulation.py conventions)
# ---------------------------------------------------------------------------

def _storage_id(bag_path: Path) -> str:
    meta = bag_path / 'metadata.yaml'
    if meta.exists() and 'mcap' in meta.read_text():
        return 'mcap'
    return 'sqlite3'


def _find_latest_bag(root: Path) -> Optional[Path]:
    candidates = sorted(root.glob('*/metadata.yaml'), key=lambda p: p.stat().st_mtime)
    return candidates[-1].parent if candidates else None


def load_drone_starts(world_file: Path) -> dict:
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


# ---------------------------------------------------------------------------
# Bag reader
# ---------------------------------------------------------------------------

def read_bag(bag_path: Path, drones: List[str]) -> Dict[str, DroneRecord]:
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError:
        print('[metrics] rosbag2_py / rclpy not available.', file=sys.stderr)
        sys.exit(1)

    records: Dict[str, DroneRecord] = {ns: DroneRecord(ns=ns) for ns in drones}
    # keyed by (round(x,1), round(y,1)) to deduplicate waypoints across updates
    _wp_seen: Dict[str, dict] = {ns: {} for ns in drones}

    pose_topics    = {f'/{d}/self_localization/pose': d   for d in drones}
    ref_topics     = {f'/{d}/motion_reference/pose': d    for d in drones}
    mission_topics = {f'/{d}/mission_update': d           for d in drones}
    kb_fact_topics  = {f'/{d}/kb/add_fact': d                    for d in drones}

    interesting = set(pose_topics) | set(ref_topics) | set(mission_topics) | set(kb_fact_topics)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=_storage_id(bag_path)),
        rosbag2_py.ConverterOptions('', ''),
    )
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}

    while reader.has_next():
        topic, raw, ts_ns = reader.read_next()
        if topic not in interesting or topic not in type_map:
            continue

        t = ts_ns * 1e-9
        msg = deserialize_message(raw, get_message(type_map[topic]))

        if topic in pose_topics:
            ns = pose_topics[topic]
            p = msg.pose.position
            records[ns].poses.append((t, p.x, p.y, p.z))

        elif topic in ref_topics:
            ns = ref_topics[topic]
            p = msg.pose.position
            records[ns].ref_poses.append((t, p.x, p.y, p.z))

        elif topic in mission_topics:
            ns = mission_topics[topic]
            records[ns].mission_update_times.append(t)
            records[ns].mission_update_actions.append((t, int(msg.action)))
            try:
                plan = json.loads(msg.mission).get('plan', [])
                for item in plan:
                    behavior = item.get('behavior', '')
                    args = item.get('args', {})
                    if behavior == 'land':
                        continue
                    if behavior in ('follow_path',) and 'path' in args:
                        for wp in args['path']:
                            x, y, z = float(wp[0]), float(wp[1]), float(wp[2])
                            key = (round(x, 1), round(y, 1))
                            _wp_seen[ns].setdefault(key, [x, y, z])
                    elif behavior in ('go_to', 'collision_avoidance'):
                        x = float(args.get('x', 0.0))
                        y = float(args.get('y', 0.0))
                        z = float(args.get('z', 1.0))
                        key = (round(x, 1), round(y, 1))
                        _wp_seen[ns].setdefault(key, [x, y, z])
            except (json.JSONDecodeError, KeyError, TypeError, IndexError):
                pass

        elif topic in kb_fact_topics:
            ns = kb_fact_topics[topic]
            # kb/add_fact payload is "<subject> <predicate> <object>"
            parts = msg.data.split(maxsplit=2)
            if len(parts) == 3:
                records[ns].kb_events.append((t, parts[1], parts[2]))

    for ns in drones:
        records[ns].planned_waypoints = list(_wp_seen[ns].values())

    return records


# ---------------------------------------------------------------------------
# Per-drone metrics
# ---------------------------------------------------------------------------

def path_length(poses: List[Tuple]) -> float:
    if len(poses) < 2:
        return 0.0
    pts = np.array([[p[1], p[2], p[3]] for p in poses])
    return float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))


def tracking_error(record: DroneRecord) -> Optional[float]:
    """Mean L2 distance between actual pose and nearest reference pose (m)."""
    if len(record.poses) < 2 or len(record.ref_poses) < 2:
        return None
    ref_t = np.array([p[0] for p in record.ref_poses])
    ref_xyz = np.array([[p[1], p[2], p[3]] for p in record.ref_poses])
    errors = []
    for t, x, y, z in record.poses:
        idx = int(np.searchsorted(ref_t, t))
        idx = min(idx, len(ref_xyz) - 1)
        errors.append(np.linalg.norm(np.array([x, y, z]) - ref_xyz[idx]))
    return float(np.mean(errors))



def count_visited(poses: List[Tuple], waypoints: List[List[float]],
                  threshold: float = VISIT_THRESHOLD) -> int:
    """Count waypoints the drone came within `threshold` metres of at any point."""
    if not poses or not waypoints:
        return 0
    pos_xy = np.array([[p[1], p[2]] for p in poses])
    visited = 0
    for wp in waypoints:
        dists = np.linalg.norm(pos_xy - np.array([wp[0], wp[1]]), axis=1)
        if dists.min() <= threshold:
            visited += 1
    return visited


def replan_times(record: DroneRecord) -> List[float]:
    """Inter-update intervals (s) — each gap is one time-to-replan sample."""
    times = sorted(record.mission_update_times)
    if len(times) < 2:
        return []
    return [times[i + 1] - times[i] for i in range(len(times) - 1)]


# ---------------------------------------------------------------------------
# Swarm-level metrics
# ---------------------------------------------------------------------------

def makespan(records: Dict[str, DroneRecord]) -> float:
    starts = [r.poses[0][0]  for r in records.values() if r.poses]
    ends   = [r.poses[-1][0] for r in records.values() if r.poses]
    if not starts or not ends:
        return 0.0
    return float(max(ends) - min(starts))


def capacity_utilization(records: Dict[str, DroneRecord]) -> Optional[float]:
    """
    productivity     = total waypoints visited / makespan
    max_productivity = total waypoints assigned / makespan
    utilization      = productivity / max_productivity = visited / assigned
    """
    total_assigned = sum(len(r.planned_waypoints) for r in records.values())
    if total_assigned == 0:
        return None
    total_visited = sum(
        count_visited(r.poses, r.planned_waypoints) for r in records.values()
    )
    return float(total_visited / total_assigned)


def jains_fairness(values: List[float]) -> float:
    """Jain's fairness index on a list of values (1.0 = perfectly equal)."""
    if not values or all(v == 0 for v in values):
        return 1.0
    arr = np.array(values, dtype=float)
    return float(np.sum(arr) ** 2 / (len(arr) * np.sum(arr ** 2)))


def auction_convergence(records: Dict[str, DroneRecord]) -> Optional[float]:
    """
    Mean auction duration: time from 'auctionStatus started' to 'auctionStatus
    completed' KB facts, per drone (averaged over every auction round seen).

    These facts are written by AuctionBehavior::on_activate/on_execution_end
    on every drone that runs an auction, so they're available even when the
    action server never publishes feedback (e.g. single-round convergence).
    """
    durations = []
    for r in records.values():
        pending_start: Optional[float] = None
        auction_events = [(t, obj) for t, pred, obj in sorted(r.kb_events) if pred == 'auctionStatus']
        for t, event in auction_events:
            if event == 'started':
                pending_start = t
            elif event == 'completed' and pending_start is not None:
                durations.append(t - pending_start)
                pending_start = None
    return float(np.mean(durations)) if durations else None


# as2_msgs/msg/MissionUpdate action codes
_MISSION_UPDATE_EXECUTE = 0
_MISSION_UPDATE_STOP = 5


def time_to_reassignment(records: Dict[str, DroneRecord],
                         failed_drone: str) -> Optional[float]:
    """
    Gap from drone failure to the replanned EXECUTE mission reaching survivors (s).

    Failure time uses the actual observable failure marker, mode-dependent:
      - decentralized: 'droneStatus failed' KB fact published on the failed drone
        (injected directly by run_experiments.py's _inject_failure_kb)
      - centralized: MissionUpdate.STOP sent to the failed drone — mission_executor.py
        only ever sends STOP as part of its failure-triggered _do_replan(), never
        otherwise, so its first occurrence is the failure instant.
    Both are real ROS messages already in the bag; neither requires re-running.

    Falls back to the failed drone's last pose timestamp if neither marker is
    present (e.g. very old bags), since the platform node isn't actually killed
    in either injection method — that fallback is approximate at best.
    """
    failed = records.get(failed_drone)
    if not failed:
        return None

    failure_candidates = [
        t for t, pred, obj in failed.kb_events
        if pred == 'droneStatus' and obj == 'failed'
    ] + [
        t for t, action in failed.mission_update_actions
        if action == _MISSION_UPDATE_STOP
    ]
    if failure_candidates:
        failure_t = min(failure_candidates)
    elif failed.poses:
        failure_t = failed.poses[-1][0]
    else:
        return None

    # Prefer the replanned EXECUTE mission specifically — PAUSE is sent to
    # survivors immediately (synchronously with the failed drone's STOP), so
    # counting any mission_update would just measure that near-zero gap.
    execute_times = [
        t for ns, r in records.items() if ns != failed_drone
        for t, action in r.mission_update_actions
        if action == _MISSION_UPDATE_EXECUTE and t > failure_t
    ]
    if execute_times:
        return float(min(execute_times) - failure_t)

    # Fall back to any mission update (bags recorded before action tracking).
    earliest = min(
        (t for ns, r in records.items()
         if ns != failed_drone
         for t in r.mission_update_times
         if t > failure_t),
        default=None,
    )
    return float(earliest - failure_t) if earliest is not None else None


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------

def print_report(records: Dict[str, DroneRecord],
                 failed_drone: Optional[str] = None) -> None:
    print('\n' + '=' * 50)
    print('  Mission Metrics Report')
    print('=' * 50)

    lengths = {ns: path_length(r.poses) for ns, r in records.items()}

    print('\nPer-drone metrics:')
    for ns in sorted(records):
        r = records[ns]
        te     = tracking_error(r)
        rtimes = replan_times(r)

        n_assigned = len(r.planned_waypoints)
        n_visited  = count_visited(r.poses, r.planned_waypoints)

        print(f'\n  {ns}:')
        print(f'    Path length      : {lengths[ns]:.2f} m')
        print(f'    Tracking error   : {te:.3f} m' if te is not None else '    Tracking error   : N/A (no reference topic)')
        print(f'    Waypoints        : {n_visited}/{n_assigned} visited'
              + (f'  ({n_visited/n_assigned:.0%})' if n_assigned else ''))
        if rtimes:
            print(f'    Replan times (s) : mean={np.mean(rtimes):.2f}  '
                  f'min={min(rtimes):.2f}  max={max(rtimes):.2f}  n={len(rtimes)}')
        else:
            print('    Replan times (s) : no replanning events detected')

    ms = makespan(records)
    cap = capacity_utilization(records)
    fair = jains_fairness(list(lengths.values()))
    conv = auction_convergence(records)

    print('\nSwarm-level metrics:')
    print(f'  Makespan             : {ms:.2f} s')
    total_assigned = sum(len(r.planned_waypoints) for r in records.values())
    total_visited  = sum(count_visited(r.poses, r.planned_waypoints) for r in records.values())
    print(f'  Capacity utilization : {cap:.1%}  ({total_visited}/{total_assigned} waypoints visited)'
          if cap is not None else '  Capacity utilization : N/A (no waypoints found in bag)')
    print(f'  Load balance (Jain)  : {fair:.3f}  (1.0 = perfectly equal)')
    if conv is not None:
        print(f'  Auction convergence  : {conv:.3f} s')
    else:
        print('  Auction convergence  : N/A (no auctionStatus KB facts found)')

    if failed_drone is not None:
        ttr = time_to_reassignment(records, failed_drone)
        if ttr is not None:
            print(f'  Time-to-reassign     : {ttr:.3f} s  (after {failed_drone} failure)')
        else:
            print(f'  Time-to-reassign     : N/A (no reassignment detected after {failed_drone} failure)')

    print()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description='Evaluate Aerostack2 mission metrics from a rosbag.')
    parser.add_argument('--bag', type=Path, default=None,
                        help='Path to ROS 2 bag directory (default: latest in rosbag/rosbags/)')
    parser.add_argument('--drones', nargs='+', default=None,
                        help='Drone namespaces (default: derived from world file)')
    parser.add_argument('--world', type=Path, default=WORLD_FILE,
                        help=f'World YAML for drone start positions (default: {WORLD_FILE})')
    parser.add_argument('--failed-drone', type=str, default=None,
                        help='Namespace of the drone whose failure was injected (enables time-to-reassignment metric)')
    args = parser.parse_args()

    # Resolve drone list
    if args.world and args.world.exists():
        world_starts = load_drone_starts(args.world)
    else:
        world_starts = DRONE_STARTS_DEFAULT

    drones = args.drones or sorted(world_starts.keys()) or list(DRONE_STARTS_DEFAULT.keys())

    # Resolve bag path
    bag_path = args.bag
    if bag_path is None:
        bag_path = _find_latest_bag(BAG_ROOT)
        if bag_path is None:
            print('[metrics] No bag found. Use --bag <path>.', file=sys.stderr)
            sys.exit(1)
    if not bag_path.exists():
        candidate = BAG_ROOT / bag_path
        if candidate.exists():
            bag_path = candidate
        else:
            print(f'[metrics] Bag not found: {bag_path}', file=sys.stderr)
            sys.exit(1)

    print(f'[metrics] reading bag : {bag_path}')
    print(f'[metrics] drones      : {drones}')

    records = read_bag(bag_path, drones)
    print_report(records, failed_drone=args.failed_drone)


if __name__ == '__main__':
    main()
