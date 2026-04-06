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

"""Generic mission launcher — loads mission definition from a YAML file.

YAML schema
-----------
mission_id: 0          # integer sent in every MissionUpdate

takeoff:
  height: 1.0          # metres
  speed:  0.7          # m/s

auctioneer: drone0     # drone that runs the auction (must be in drones list)

drones:                # all participating drones (auctioneer + bidders)
  - drone0
  - drone1

auction:
  name:         point_assignment
  auction_type: coordinate_item
  elements:
    - name:          point_A
      feature_names: [x, y]
      features:      [3.0, 10.0]
    # …

The auctioneer receives:  takeoff → auction
Every other drone receives: takeoff only

Usage:
    python3 begin_mission_yaml.py missions/begin_mission.yaml [-s]
"""

__authors__ = 'Guillermo GP-Lenza'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import argparse
import json
from time import sleep

import yaml
from as2_msgs.msg import MissionUpdate
import rclpy
from rclpy.node import Node


class BeginMission(Node):
    """Publishes EXECUTE missions to the ROS 2 mission interpreter adapter."""

    def __init__(self, cfg: dict, use_sim_time: bool = False):
        super().__init__('begin_mission')
        self.set_parameters([
            rclpy.parameter.Parameter(
                'use_sim_time', rclpy.parameter.Parameter.Type.BOOL, use_sim_time
            )
        ])

        self._mission_id = cfg.get('mission_id', 0)
        self._drones = cfg['drones']
        self._auctioneer = cfg['auctioneer']
        self._takeoff_height = cfg['takeoff']['height']
        self._takeoff_speed = cfg['takeoff']['speed']
        self._auction = cfg['auction']

        self._mission_pub = {
            drone: self.create_publisher(MissionUpdate, f'/{drone}/mission_update', 10)
            for drone in self._drones
        }

    def _send_mission(self, drone_id: str, plan: list) -> None:
        mission = {'target': drone_id, 'plan': plan}
        msg = MissionUpdate()
        msg.drone_id = drone_id
        msg.mission_id = self._mission_id
        msg.action = MissionUpdate.EXECUTE
        msg.mission = json.dumps(mission)
        self._mission_pub[drone_id].publish(msg)
        self.get_logger().info(
            f'[{drone_id}] mission sent: {[item["behavior"] for item in plan]}'
        )

    def run(self) -> None:
        # Allow publishers time to connect before sending.
        sleep(1.0)

        takeoff_step = {
            'behavior': 'takeoff',
            'args': {'height': self._takeoff_height, 'speed': self._takeoff_speed},
        }
        auction_step = {
            'behavior': 'auction',
            'args': {
                'name': self._auction['name'],
                'elements': self._auction['elements'],
                'auction_type': self._auction['auction_type'],
                'bidders': self._drones,
            },
        }

        # Auctioneer: takeoff → auction
        self._send_mission(self._auctioneer, [takeoff_step, auction_step])

        # Bidders: takeoff only
        for drone in self._drones:
            if drone != self._auctioneer:
                self._send_mission(drone, [takeoff_step])


def main():
    parser = argparse.ArgumentParser(description='Generic YAML mission launcher')
    parser.add_argument('mission_file', help='Path to the mission YAML file')
    parser.add_argument(
        '-s', '--use_sim_time', action='store_true', default=False,
        help='Use simulation time'
    )
    args = parser.parse_args()

    with open(args.mission_file, 'r') as f:
        cfg = yaml.safe_load(f)

    rclpy.init()
    node = BeginMission(cfg, use_sim_time=args.use_sim_time)
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
