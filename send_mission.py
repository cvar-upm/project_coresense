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

"""Coverage-planner mission sender with auction-based task allocation.

Loads a YAML mission file, runs MissionInterpreter.planner() (which uses
swarm_pylib internally) to generate coverage waypoints, then sends each
drone a MissionUpdate: drone0 takes off and runs an auction with all planned
waypoints as elements; the rest of the swarm takes off and bids.

YAML mission format is the same used by mission_executor.py.

Usage:
    python3 send_mission.py missions/two_drones_area_coverage.yaml [-s] [-v]
"""

__authors__ = 'Guillermo GP-Lenza'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import argparse
import json
import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import yaml

import rclpy
from rclpy.node import Node
from as2_msgs.msg import MissionUpdate

from as2_interface.mission_manager import MissionInterpreter
from utils.waypoint_markers import publish_panel_markers, WaypointMarkerTracker

GROUND_Z = 0.1
POSE_TIMEOUT = 10.0
WAYPOINT_REACH_DIST = 0.5


def read_initial_positions(names: list, use_sim_time: bool, verbose: bool) -> dict:
    from as2_python_api.drone_interface import DroneInterface

    interfaces = {
        name: DroneInterface(drone_id=name, use_sim_time=use_sim_time, verbose=verbose)
        for name in names
    }

    def _has_pose(iface):
        return not any(math.isnan(v) for v in iface.position)

    deadline = time.time() + POSE_TIMEOUT
    pending = set(names)
    while pending and time.time() < deadline:
        pending = {n for n in pending if not _has_pose(interfaces[n])}
        if pending:
            time.sleep(0.1)

    if pending:
        raise RuntimeError(f"Timed out waiting for pose from: {', '.join(pending)}")

    positions = {name: list(interfaces[name].position) for name in names}

    for iface in interfaces.values():
        iface.shutdown()

    return positions


def collect_waypoints(mission: dict) -> list:
    """Collect all unique above-ground coverage waypoints from the planned mission.

    For Area legs the first and last waypoints are drone transit positions
    (takeoff origin and next-leg destination), not coverage points, so they
    are excluded.
    """
    waypoints = []
    seen = set()
    for _uav, legs in mission.items():
        for leg in legs:
            if leg['name'] == 'LandPoint':
                continue
            points = leg['values']
            if leg['name'] == 'Area' and len(points) > 2:
                points = points[1:-1]
            for point in points:
                if float(point[2]) <= GROUND_Z:
                    continue
                key = (round(float(point[0]), 3), round(float(point[1]), 3), round(float(point[2]), 3))
                if key not in seen:
                    seen.add(key)
                    waypoints.append([float(point[0]), float(point[1]), float(point[2])])
    return waypoints


def build_auction_mission(uav_names: list, takeoff_height: float, waypoints: list) -> dict:
    """Build per-drone plans.

    drone0 (auctioneer): takeoff + auction over all coverage waypoints.
    All other drones: takeoff only (they participate as bidders).
    """
    elements = [
        {
            'name': f'wp_{i}',
            'feature_names': ['x', 'y', 'z'],
            'features': [wp[0], wp[1], wp[2]],
        }
        for i, wp in enumerate(waypoints)
    ]

    plans = {}
    auctioneer = uav_names[0]
    for name in uav_names:
        plan = [{'behavior': 'takeoff', 'args': {'height': takeoff_height, 'speed': 1.0}}]
        if name == auctioneer:
            plan.append({
                'behavior': 'auction',
                'args': {
                    'name': 'coverage_allocation',
                    'auction_type': 'coordinate_item',
                    'bidders': list(uav_names),
                    'elements': elements,
                },
            })
        plans[name] = plan

    return plans


class MissionSender(Node):
    """Publishes EXECUTE missions built from the coverage planner + auction."""

    def __init__(self, plans: dict, waypoints: list, drone_names: list,
                 mission_id: int = 0, use_sim_time: bool = False, no_panels: bool = False):
        super().__init__('mission_sender')
        self.set_parameters([
            rclpy.parameter.Parameter(
                'use_sim_time', rclpy.parameter.Parameter.Type.BOOL, use_sim_time
            )
        ])
        self._mission_id = mission_id
        self._plans = plans
        self._waypoints = waypoints
        self._drone_names = drone_names
        self._no_panels = no_panels
        self._wp_tracker = None
        self._done_event = threading.Event()
        self._pubs = {
            drone: self.create_publisher(MissionUpdate, f'/{drone}/mission_update', 10)
            for drone in plans
        }

    def _on_pose(self, msg):
        if self._wp_tracker is None:
            return
        pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        self._wp_tracker.mark_visited_near(pos, WAYPOINT_REACH_DIST)
        if self._wp_tracker.all_visited():
            self._done_event.set()

    def run(self) -> None:
        from geometry_msgs.msg import PoseStamped
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

        timeout = 10.0
        for drone_id, pub in self._pubs.items():
            print(f'[{drone_id}] waiting for mission_executor subscriber...')
            deadline = time.time() + timeout
            while pub.get_subscription_count() == 0:
                if time.time() > deadline:
                    print(
                        f'[{drone_id}] WARNING: no subscriber after {timeout}s — '
                        'is mission_executor running for this drone?'
                    )
                    break
                rclpy.spin_once(self, timeout_sec=0.05)
            print(f'[{drone_id}] subscriber ready ({pub.get_subscription_count()} connected)')

        if not self._no_panels:
            print('[send_mission] publishing solar panel markers...')
            publish_panel_markers(self, self._waypoints)
            print('[send_mission] panel markers published')
            print('[send_mission] publishing waypoint markers...')
            self._wp_tracker = WaypointMarkerTracker(self, self._waypoints)
            print('[send_mission] waypoint markers published')

            pose_qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            )
            for name in self._drone_names:
                self.create_subscription(
                    PoseStamped,
                    f'/{name}/self_localization/pose',
                    self._on_pose,
                    pose_qos,
                )

        print('[send_mission] sending mission plans to drones...')
        for drone_id, plan in self._plans.items():
            msg = MissionUpdate()
            msg.drone_id = drone_id
            msg.mission_id = self._mission_id
            msg.action = MissionUpdate.EXECUTE
            msg.mission = json.dumps({'target': drone_id, 'plan': plan})
            self._pubs[drone_id].publish(msg)
            self.get_logger().info(
                f'[{drone_id}] sent: {[step["behavior"] for step in plan]}'
            )

        if not self._no_panels:
            print('[send_mission] monitoring poses for waypoint completion...')
            while not self._done_event.is_set():
                rclpy.spin_once(self, timeout_sec=0.1)
            print('[send_mission] all waypoints visited')


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Coverage-planner mission sender with auction-based task allocation'
    )
    parser.add_argument('mission', help='Path to the YAML mission file')
    parser.add_argument('-s', '--use-sim-time', action='store_true', default=False,
                        help='Use simulation time')
    parser.add_argument('-v', '--verbose', action='store_true', default=False,
                        help='Verbose drone interface output')
    parser.add_argument('--no-panels', action='store_true',
                        help='Skip publishing solar panel markers to the AR overlay')
    args = parser.parse_args()

    with open(args.mission, 'r', encoding='utf-8') as f:
        payload = yaml.safe_load(f)

    use_cartesian = bool(payload.get('use_cartesian_coordinates', False))
    takeoff_height = float(payload.get('takeoff_height', 2.0))
    max_wp_distance = payload.get('max_wp_distance')
    if max_wp_distance is not None:
        max_wp_distance = float(max_wp_distance)
    filter_collinear = bool(payload.get('filter_collinear', True))
    uav_names = [str(u) for u in payload['uavList']]

    rclpy.init()

    print('[send_mission] reading initial drone positions...')
    initial_positions = read_initial_positions(uav_names, args.use_sim_time, args.verbose)
    print(f'[send_mission] positions: {initial_positions}')

    print('[send_mission] running coverage planner...')
    _, mission = MissionInterpreter.planner(
        0, payload, use_cartesian, initial_positions, max_wp_distance, filter_collinear
    )

    waypoints = collect_waypoints(mission)
    print(f'[plan] {len(waypoints)} unique coverage waypoints for {len(uav_names)} drones')

    plans = build_auction_mission(uav_names, takeoff_height, waypoints)

    node = MissionSender(plans, waypoints, drone_names=uav_names, mission_id=0,
                         use_sim_time=args.use_sim_time, no_panels=args.no_panels)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print('Missions sent')
    return 0


if __name__ == '__main__':
    sys.exit(main())
