#!/usr/bin/env python3

# Copyright 2024 Universidad Politécnica de Madrid
# SPDX-License-Identifier: BSD-3-Clause

"""Stop the current mission on all drones without killing aerostack2.

Drones will abort their current behaviour and hover in place.
The simulator keeps running, drone positions are preserved.

Usage:
    python3 stop_mission.py [-s]
    python3 stop_mission.py --drones drone0,drone1,drone2 [-s]
"""

import argparse
from time import sleep

from as2_msgs.msg import MissionUpdate
import rclpy
from rclpy.node import Node


class MissionStopper(Node):
    def __init__(self, drones: list, use_sim_time: bool = False):
        super().__init__('mission_stopper')
        self.set_parameters([
            rclpy.parameter.Parameter(
                'use_sim_time', rclpy.parameter.Parameter.Type.BOOL, use_sim_time
            )
        ])
        self._pubs = {
            d: self.create_publisher(MissionUpdate, f'/{d}/mission_update', 10)
            for d in drones
        }
        self._drones = drones

    def run(self) -> None:
        sleep(0.5)
        for drone in self._drones:
            msg = MissionUpdate()
            msg.drone_id = drone
            msg.mission_id = 0
            msg.action = MissionUpdate.STOP
            self._pubs[drone].publish(msg)
            self.get_logger().info(f'[{drone}] STOP sent — drone will hover in place')


def main():
    parser = argparse.ArgumentParser(description='Stop the current mission on all drones')
    parser.add_argument(
        '--drones', default='drone0,drone1,drone2,drone3,drone4',
        help='Comma-separated drone namespaces (default: drone0..drone4)',
    )
    parser.add_argument('-s', '--use_sim_time', action='store_true', default=False)
    args = parser.parse_args()

    drones = [d.strip() for d in args.drones.split(',') if d.strip()]
    rclpy.init()
    node = MissionStopper(drones=drones, use_sim_time=args.use_sim_time)
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
