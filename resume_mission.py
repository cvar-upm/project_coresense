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

"""Re-run the auction on waypoints that were not yet visited.

Reads the progress file written by track_progress.py, removes already-visited
points from the original mission's AUCTION_POINTS, and launches a new auction
with the remaining points.

Typical workflow
----------------
1. Start the experiment normally and run track_progress.py alongside it.
2. Stop the mission in-place (drones keep hovering, simulator stays running):
       python3 stop_mission.py -s
3. Run resume_mission.py — drones re-auction the unvisited waypoints from
   their current positions (no takeoff needed, they are already airborne):
       python3 resume_mission.py --mission begin5_mission_solar.py -s

Do NOT re-launch aerostack2: that resets the simulation and drone positions.

Usage:
    python3 resume_mission.py \\
        --mission begin5_mission_solar.py \\
        --progress /tmp/mission_progress.json \\
        -s
"""

import argparse
import importlib.util
import json
import math
import os
import sys
from time import sleep

from as2_msgs.msg import MissionUpdate
import rclpy
from rclpy.node import Node


def _load_mission_module(mission_path: str):
    """Dynamically load a mission file and return the module."""
    spec = importlib.util.spec_from_file_location('_mission', mission_path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class ResumeMission(Node):
    """Sends a new auction mission covering only the unvisited waypoints."""

    def __init__(
        self,
        drones: list,
        remaining_points: list,
        use_sim_time: bool = False,
    ):
        super().__init__('resume_mission')
        self.set_parameters([
            rclpy.parameter.Parameter(
                'use_sim_time',
                rclpy.parameter.Parameter.Type.BOOL,
                use_sim_time,
            )
        ])
        self._drones = drones
        self._remaining_points = remaining_points
        self._mission_pub = {
            drone: self.create_publisher(MissionUpdate, f'/{drone}/mission_update', 10)
            for drone in drones
        }

    def _send_mission(self, drone_id: str, plan: list) -> None:
        import json as _json
        mission = {'target': drone_id, 'plan': plan}
        msg = MissionUpdate()
        msg.drone_id = drone_id
        msg.mission_id = 0
        msg.action = MissionUpdate.EXECUTE
        msg.mission = _json.dumps(mission)
        self._mission_pub[drone_id].publish(msg)
        behaviors = [item['behavior'] for item in plan]
        self.get_logger().info(f'[{drone_id}] resume mission sent: {behaviors}')

    def run(self) -> None:
        sleep(1.0)

        n = len(self._remaining_points)
        self.get_logger().info(
            f'Resuming auction with {n} remaining waypoint(s) '
            f'across {len(self._drones)} drone(s)'
        )

        if n == 0:
            self.get_logger().info('Nothing to do — all waypoints already visited.')
            return

        auctioneer = self._drones[0]

        # Drones are already airborne — send auction directly, no takeoff step.
        # Bid costs are computed from each drone's current hover position, so
        # the assignment naturally reflects where each drone is right now.
        self._send_mission(
            auctioneer,
            [
                {
                    'behavior': 'auction',
                    'args': {
                        'name': 'resume_point_assignment',
                        'elements': self._remaining_points,
                        'auction_type': 'coordinate_item',
                        'bidders': self._drones,
                    },
                },
            ],
        )

        # Participants receive the StartAuction via CA gateway — no explicit
        # mission needed.  Send an empty EXECUTE so the interpreter is in a
        # clean state and ready to process the incoming auction goal.
        for drone in self._drones[1:]:
            self._send_mission(drone, [])


def main():
    parser = argparse.ArgumentParser(
        description='Resume an interrupted mission by re-auctioning unvisited waypoints'
    )
    parser.add_argument(
        '--mission', required=True,
        help='Original mission file (e.g. begin5_mission_solar.py)',
    )
    parser.add_argument(
        '--progress', default='/tmp/mission_progress.json',
        help='Progress JSON written by track_progress.py '
             '(default: /tmp/mission_progress.json)',
    )
    parser.add_argument(
        '--drones', default='drone0,drone1,drone2,drone3,drone4',
        help='Comma-separated drone namespaces (default: drone0..drone4)',
    )
    parser.add_argument(
        '-s', '--use_sim_time', action='store_true', default=False,
        help='Use simulation time',
    )
    args = parser.parse_args()

    # ── Load original mission ──────────────────────────────────────────────
    mission_path = args.mission
    if not os.path.isabs(mission_path):
        mission_path = os.path.join(os.getcwd(), mission_path)
    if not os.path.exists(mission_path):
        print(f'ERROR: mission file not found: {mission_path}', file=sys.stderr)
        sys.exit(1)

    mod = _load_mission_module(mission_path)
    all_points = {p['name']: p for p in mod.AUCTION_POINTS}

    # ── Load progress file ─────────────────────────────────────────────────
    progress_path = args.progress
    if not os.path.exists(progress_path):
        print(
            f'ERROR: progress file not found: {progress_path}\n'
            'Make sure track_progress.py was running during the experiment.',
            file=sys.stderr,
        )
        sys.exit(1)

    with open(progress_path) as f:
        progress = json.load(f)

    visited = set(progress.get('visited', []))
    remaining_points = [p for name, p in all_points.items() if name not in visited]

    total = len(all_points)
    done = len(visited)
    left = len(remaining_points)
    print(f'Progress: {done}/{total} visited, {left} remaining')
    if visited:
        print(f'  Skipping: {sorted(visited)}')
    if remaining_points:
        print(f'  Re-auctioning: {[p["name"] for p in remaining_points]}')

    if left == 0:
        print('All waypoints already visited — nothing to resume.')
        sys.exit(0)

    # ── Warn if bundle_size may need adjusting ────────────────────────────
    n_drones = len(args.drones.split(','))
    if left < n_drones:
        print(
            f'WARNING: only {left} point(s) remain for {n_drones} drones. '
            'Consider reducing the number of drones or lowering bundle_size '
            'in config/config.yaml.'
        )

    # ── Launch resume mission ──────────────────────────────────────────────
    drones = [d.strip() for d in args.drones.split(',') if d.strip()]

    rclpy.init()
    node = ResumeMission(
        drones=drones,
        remaining_points=remaining_points,
        use_sim_time=args.use_sim_time,
    )
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
