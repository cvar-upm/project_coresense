#!/usr/bin/env python3

# Copyright 2024 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names
#      of its contributors may be used to endorse or promote products derived
#      from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Snapshot current drone positions and remaining waypoints, then generate a
fresh mission file and world config for a new experiment run.

Workflow
--------
1.  Start the experiment normally alongside track_progress.py.
2.  Stop the mission mid-way (stop_mission.py) — drones hover in place.
3.  Run this script:
        python3 snapshot_mission.py \\
            --mission begin5_mission_solar.py \\
            --drones drone0,drone1,drone3,drone4 \\
            --progress /tmp/mission_progress.json \\
            -s
4.  Kill aerostack2 completely:
        bash stop_as2.bash
        tmux kill-server    # if tmux sessions linger
5.  Relaunch with the generated world config:
        bash launch_as2.bash -m -n drone0,drone1,drone3,drone4
    (uses config/world_snapshot.yaml which was updated by this script)
6.  Start the generated mission:
        python3 snapshot_mission_gen.py -s
"""

__authors__ = 'Guillermo GP-Lenza'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import argparse
import importlib.util
import json
import math
import os
import sys
import threading
from pathlib import Path

import yaml
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# ── Defaults ──────────────────────────────────────────────────────────────────
SCRIPT_DIR = Path(__file__).parent
DEFAULT_WORLD_IN  = SCRIPT_DIR / 'config' / 'world5drones_solar.yaml'
DEFAULT_WORLD_OUT = SCRIPT_DIR / 'config' / 'world_snapshot.yaml'
DEFAULT_MISSION_OUT = SCRIPT_DIR / 'snapshot_mission_gen.py'
DEFAULT_CONFIG    = SCRIPT_DIR / 'config' / 'config.yaml'
DEFAULT_PROGRESS  = Path('/tmp/mission_progress.json')
POSE_TIMEOUT_S    = 10.0   # max seconds to wait for first pose per drone
# ──────────────────────────────────────────────────────────────────────────────


# ── ROS pose collector ────────────────────────────────────────────────────────

class PoseCollector(Node):
    """Subscribe to each drone's pose and collect one sample per drone."""

    def __init__(self, drones: list, use_sim_time: bool = False):
        super().__init__('snapshot_pose_collector')
        self.set_parameters([
            rclpy.parameter.Parameter(
                'use_sim_time',
                rclpy.parameter.Parameter.Type.BOOL,
                use_sim_time,
            )
        ])
        self._poses: dict = {}
        self._lock = threading.Lock()
        self._drones = list(drones)
        for drone in drones:
            self.create_subscription(
                PoseStamped,
                f'/{drone}/self_localization/pose',
                lambda msg, d=drone: self._cb(msg, d),
                qos_profile_sensor_data,
            )

    def _cb(self, msg: PoseStamped, drone: str) -> None:
        with self._lock:
            if drone not in self._poses:
                x = msg.pose.position.x
                y = msg.pose.position.y
                z = msg.pose.position.z
                self._poses[drone] = (x, y, z)
                self.get_logger().info(
                    f'  {drone}: ({x:.3f}, {y:.3f}, {z:.3f})'
                )

    def all_received(self) -> bool:
        with self._lock:
            return all(d in self._poses for d in self._drones)

    def poses(self) -> dict:
        with self._lock:
            return dict(self._poses)


# ── File generators ───────────────────────────────────────────────────────────

def _gps_origin(world_yaml_path: Path) -> dict:
    """Read the GPS origin block from an existing world YAML."""
    with open(world_yaml_path) as f:
        data = yaml.safe_load(f)
    try:
        return data['/**']['platform']['ros__parameters']['gps_origin']
    except (KeyError, TypeError):
        return {'latitude': 40.4405287, 'longitude': -3.6898277, 'altitude': 100.0}


def write_world_yaml(poses: dict, gps_origin: dict, out_path: Path) -> None:
    """Write a world YAML with drone spawn positions set to current x/y, z=0."""
    lines = [
        '/**:',
        '  platform:',
        '    ros__parameters:',
        '      gps_origin:',
        f"        latitude: {gps_origin['latitude']}",
        f"        longitude: {gps_origin['longitude']}",
        f"        altitude: {gps_origin['altitude']}",
        '',
        '# World config generated by snapshot_mission.py.',
        '# Drone spawn positions reflect where they hovered when the experiment was stopped.',
        '# z is always 0 (ground) — the simulation resets drones to ground on relaunch.',
        '',
    ]
    for drone, (x, y, _z) in sorted(poses.items()):
        lines += [
            f'{drone}:',
            '  platform:',
            '    ros__parameters:',
            '      vehicle_initial_pose:',
            f'        x: {x:.3f}',
            f'        y: {y:.3f}',
            '        z: 0.0',
            '',
        ]
    out_path.write_text('\n'.join(lines))
    print(f'  World config  -> {out_path}')


def write_mission_py(
    drones: list,
    remaining_points: list,
    out_path: Path,
    takeoff_height: float = 1.0,
    takeoff_speed: float = 0.7,
) -> None:
    """Write a ready-to-run mission Python file."""
    auctioneer = drones[0]
    participants = drones[1:]

    # Build AUCTION_POINTS literal
    pts_lines = ['AUCTION_POINTS = [']
    for p in remaining_points:
        pts_lines.append(
            f"    {{'name': {p['name']!r}, "
            f"'feature_names': {p['feature_names']!r}, "
            f"'features': {p['features']!r}}},"
        )
    pts_lines.append(']')
    auction_points_block = '\n'.join(pts_lines)

    # ALL_DRONES list
    all_drones_repr = repr(drones)

    # Participant takeoff sends
    if participants:
        participant_block = (
            "        # Participants: takeoff only (they join the auction via CA gateway)\n"
            f"        for drone in {participants!r}:\n"
            "            self._send_mission(\n"
            "                drone,\n"
            "                [{'behavior': 'takeoff',\n"
            "                  'args': {'height': TAKEOFF_HEIGHT, 'speed': TAKEOFF_SPEED}}],\n"
            "            )\n"
        )
    else:
        participant_block = "        # No additional participants.\n"

    content = f'''\
#!/usr/bin/env python3

# Copyright 2024 Universidad Politécnica de Madrid
# SPDX-License-Identifier: BSD-3-Clause

"""Snapshot mission — generated by snapshot_mission.py.

Covers only the waypoints that were unvisited when the previous experiment
was stopped.  Drone starting positions are set in config/world_snapshot.yaml.

Usage:
    python3 {out_path.name} [-s]
"""

import argparse
import json
from time import sleep

from as2_msgs.msg import MissionUpdate
import rclpy
from rclpy.node import Node

# ── Parameters ────────────────────────────────────────────────────────────────
ALL_DRONES = {all_drones_repr}

TAKEOFF_HEIGHT = {takeoff_height}   # metres
TAKEOFF_SPEED  = {takeoff_speed}    # m/s

# Remaining inspection waypoints (unvisited at snapshot time).
{auction_points_block}

MISSION_ID = 0
# ──────────────────────────────────────────────────────────────────────────────


class BeginMission(Node):
    """Publishes EXECUTE missions to the ROS 2 mission interpreter adapter."""

    def __init__(self, use_sim_time: bool = False):
        super().__init__('begin_mission')
        self.set_parameters([
            rclpy.parameter.Parameter(
                'use_sim_time', rclpy.parameter.Parameter.Type.BOOL, use_sim_time
            )
        ])
        self._mission_pub = {{
            drone: self.create_publisher(MissionUpdate, f'/{{drone}}/mission_update', 10)
            for drone in ALL_DRONES
        }}

    def _send_mission(self, drone_id: str, plan: list) -> None:
        mission = {{'target': drone_id, 'plan': plan}}
        msg = MissionUpdate()
        msg.drone_id = drone_id
        msg.mission_id = MISSION_ID
        msg.action = MissionUpdate.EXECUTE
        msg.mission = json.dumps(mission)
        self._mission_pub[drone_id].publish(msg)
        self.get_logger().info(f\'[{{drone_id}}] mission sent: {{[item["behavior"] for item in plan]}}\')

    def run(self) -> None:
        sleep(1.0)

        # Auctioneer: takeoff → auction
        self._send_mission(
            {auctioneer!r},
            [
                {{'behavior': 'takeoff',
                  'args': {{'height': TAKEOFF_HEIGHT, 'speed': TAKEOFF_SPEED}}}},
                {{'behavior': 'auction',
                  'args': {{
                      'name': 'snapshot_point_assignment',
                      'elements': AUCTION_POINTS,
                      'auction_type': 'coordinate_item',
                      'bidders': ALL_DRONES,
                  }}}},
            ],
        )
{participant_block}

def main():
    parser = argparse.ArgumentParser(description='Run snapshot mission')
    parser.add_argument('-s', '--use_sim_time', action='store_true', default=False)
    args = parser.parse_args()

    rclpy.init()
    node = BeginMission(use_sim_time=args.use_sim_time)
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
'''
    out_path.write_text(content)
    out_path.chmod(0o755)
    print(f'  Mission file  -> {out_path}')


def update_bundle_size(config_path: Path, n_drones: int, n_points: int) -> int:
    """Update bundle_size in config.yaml so all points get assigned."""
    bundle_size = math.ceil(n_points / n_drones)
    if not config_path.exists():
        return bundle_size
    text = config_path.read_text()
    import re
    new_text = re.sub(r'(bundle_size\s*:\s*)\d+', rf'\g<1>{bundle_size}', text)
    if new_text != text:
        config_path.write_text(new_text)
        print(f'  bundle_size   -> {bundle_size}  (updated {config_path})')
    else:
        print(f'  bundle_size   -> {bundle_size}  (set manually in {config_path})')
    return bundle_size


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description='Snapshot drone positions + remaining waypoints and generate a new experiment'
    )
    parser.add_argument(
        '--mission', required=True,
        help='Original mission file (e.g. begin5_mission_solar.py)',
    )
    parser.add_argument(
        '--drones', required=True,
        help='Comma-separated active drone namespaces (exclude failed drones)',
    )
    parser.add_argument(
        '--progress', type=Path, default=DEFAULT_PROGRESS,
        help=f'Progress JSON from track_progress.py (default: {DEFAULT_PROGRESS})',
    )
    parser.add_argument(
        '--world-in', type=Path, default=DEFAULT_WORLD_IN,
        help=f'Original world YAML to copy GPS origin from (default: {DEFAULT_WORLD_IN})',
    )
    parser.add_argument(
        '--world-out', type=Path, default=DEFAULT_WORLD_OUT,
        help=f'Output world YAML path (default: {DEFAULT_WORLD_OUT})',
    )
    parser.add_argument(
        '--mission-out', type=Path, default=DEFAULT_MISSION_OUT,
        help=f'Output mission Python file (default: {DEFAULT_MISSION_OUT})',
    )
    parser.add_argument(
        '--config', type=Path, default=DEFAULT_CONFIG,
        help=f'config.yaml to update bundle_size in (default: {DEFAULT_CONFIG})',
    )
    parser.add_argument(
        '-s', '--use_sim_time', action='store_true', default=False,
        help='Use simulation time',
    )
    args = parser.parse_args()

    # ── Load original mission ────────────────────────────────────────────────
    mission_path = args.mission
    if not os.path.isabs(mission_path):
        mission_path = os.path.join(os.getcwd(), mission_path)
    if not os.path.exists(mission_path):
        print(f'ERROR: mission file not found: {mission_path}', file=sys.stderr)
        sys.exit(1)

    spec = importlib.util.spec_from_file_location('_mission', mission_path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    all_points = {p['name']: p for p in mod.AUCTION_POINTS}

    # ── Determine remaining points ───────────────────────────────────────────
    if args.progress.exists():
        with open(args.progress) as f:
            prog = json.load(f)
        visited = set(prog.get('visited', []))
        remaining = [p for name, p in all_points.items() if name not in visited]
        print(f'Progress: {len(visited)} visited, {len(remaining)} remaining '
              f'(from {args.progress})')
    else:
        remaining = list(all_points.values())
        print(f'No progress file found — using all {len(remaining)} points.')

    if not remaining:
        print('All waypoints already visited — nothing to snapshot.')
        sys.exit(0)

    # ── Collect drone poses ──────────────────────────────────────────────────
    drones = [d.strip() for d in args.drones.split(',') if d.strip()]
    print(f'\nCollecting poses for: {drones}')
    print(f'(timeout: {POSE_TIMEOUT_S}s per drone)')

    rclpy.init()
    collector = PoseCollector(drones=drones, use_sim_time=args.use_sim_time)

    import time
    deadline = time.monotonic() + POSE_TIMEOUT_S
    while not collector.all_received() and time.monotonic() < deadline:
        rclpy.spin_once(collector, timeout_sec=0.1)

    poses = collector.poses()
    collector.destroy_node()
    rclpy.shutdown()

    missing = [d for d in drones if d not in poses]
    if missing:
        print(f'WARNING: no pose received for: {missing}', file=sys.stderr)
        print('These drones will be excluded from the snapshot.', file=sys.stderr)
        drones = [d for d in drones if d in poses]
        if not drones:
            print('ERROR: no drone poses received at all.', file=sys.stderr)
            sys.exit(1)

    # ── Generate outputs ─────────────────────────────────────────────────────
    print(f'\nGenerating files for {len(drones)} drone(s), {len(remaining)} point(s):')

    gps_orig = _gps_origin(args.world_in) if args.world_in.exists() else \
        {'latitude': 40.4405287, 'longitude': -3.6898277, 'altitude': 100.0}

    # Preserve drone order from the input --drones argument
    ordered_poses = {d: poses[d] for d in drones}

    write_world_yaml(ordered_poses, gps_orig, args.world_out)
    write_mission_py(drones, remaining, args.mission_out)
    bundle = update_bundle_size(args.config, len(drones), len(remaining))

    print(f'\nDone. Next steps:')
    print(f'  1. Kill all aerostack2 sessions:')
    print(f'       bash stop_as2.bash && tmux kill-server')
    print(f'  2. Relaunch with the snapshot world (only active drones):')
    print(f'       bash launch_as2.bash -m -n {",".join(drones)}')
    print(f'     (launch_as2.bash uses world5drones_solar.yaml by default —')
    print(f'      edit line ~35 to: simulation_config="config/world_snapshot.yaml")')
    print(f'  3. Start the snapshot mission:')
    print(f'       python3 {args.mission_out.name} -s')
    print(f'  4. Start the progress tracker alongside:')
    print(f'       python3 track_progress.py \\')
    print(f'           --mission {args.mission_out.name} \\')
    print(f'           --drones {",".join(drones)} -s')


if __name__ == '__main__':
    main()
