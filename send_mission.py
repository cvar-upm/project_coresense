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

"""Generic JSON mission sender — loads a mission JSON and publishes MissionUpdate messages.

JSON schema
-----------
{
  "mission_id": 0,
  "drones": {
    "drone0": [
      {"behavior": "takeoff", "args": {"height": 1.0, "speed": 0.7}},
      {"behavior": "auction", "args": { ... }}
    ],
    "drone1": [
      {"behavior": "takeoff", "args": {"height": 1.0, "speed": 0.7}}
    ]
  }
}

Each key under "drones" is the drone namespace; its value is the plan array sent
verbatim inside the MissionUpdate.mission JSON string.

Usage:
    python3 send_mission.py missions_json/begin_mission.json [-s]
"""

__authors__ = 'Guillermo GP-Lenza'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import argparse
import json
from time import sleep

import rclpy
from rclpy.node import Node
from as2_msgs.msg import MissionUpdate


class MissionSender(Node):
    """Publishes EXECUTE missions loaded from a JSON file."""

    def __init__(self, cfg: dict, use_sim_time: bool = False):
        super().__init__('mission_sender')
        self.set_parameters([
            rclpy.parameter.Parameter(
                'use_sim_time', rclpy.parameter.Parameter.Type.BOOL, use_sim_time
            )
        ])

        self._mission_id = cfg.get('mission_id', 0)
        self._drones: dict = cfg['drones']

        self._pubs = {
            drone: self.create_publisher(MissionUpdate, f'/{drone}/mission_update', 10)
            for drone in self._drones
        }

    def run(self) -> None:
        sleep(1.0)
        for drone_id, plan in self._drones.items():
            msg = MissionUpdate()
            msg.drone_id = drone_id
            msg.mission_id = self._mission_id
            msg.action = MissionUpdate.EXECUTE
            msg.mission = json.dumps({'target': drone_id, 'plan': plan})
            self._pubs[drone_id].publish(msg)
            self.get_logger().info(
                f'[{drone_id}] sent: {[step["behavior"] for step in plan]}'
            )


def main():
    parser = argparse.ArgumentParser(description='JSON mission sender')
    parser.add_argument('mission_file', help='Path to the mission JSON file')
    parser.add_argument(
        '-s', '--use_sim_time', action='store_true', default=False,
        help='Use simulation time'
    )
    args = parser.parse_args()

    with open(args.mission_file, 'r') as f:
        cfg = json.load(f)

    rclpy.init()
    node = MissionSender(cfg, use_sim_time=args.use_sim_time)
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
