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

"""Track which auction waypoints have been visited during a mission.

Subscribes to each drone's self_localization/pose and marks a waypoint as
visited when any drone comes within --threshold metres of it.  Writes a JSON
progress file that resume_mission.py uses to skip already-visited points.

Run this in a separate terminal BEFORE starting the mission script:

    python3 track_progress.py \\
        --mission begin5_mission_solar.py \\
        --drones drone0,drone1,drone2,drone3,drone4 \\
        --output /tmp/mission_progress.json \\
        -s

Stop it with Ctrl+C (or when the experiment is killed) — it saves the final
state on exit.
"""

import argparse
import importlib.util
import json
import math
import os
import signal
import sys
import threading
from datetime import datetime

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def _load_auction_points(mission_path: str) -> list:
    """Dynamically import AUCTION_POINTS from a mission file."""
    spec = importlib.util.spec_from_file_location('_mission', mission_path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.AUCTION_POINTS


class ProgressTracker(Node):
    """ROS2 node that monitors drone positions and records visited waypoints."""

    def __init__(
        self,
        drones: list,
        auction_points: list,
        output_path: str,
        mission_path: str,
        threshold: float = 0.7,
        use_sim_time: bool = False,
    ):
        super().__init__('progress_tracker')
        self.set_parameters([
            rclpy.parameter.Parameter(
                'use_sim_time',
                rclpy.parameter.Parameter.Type.BOOL,
                use_sim_time,
            )
        ])

        self._threshold = threshold
        self._output_path = output_path
        self._mission_path = os.path.abspath(mission_path)

        # name -> (x, y)
        # Use feature_names to locate x/y in case feature order ever changes.
        def _xy(p):
            names = p.get('feature_names', [])
            feats = p['features']
            xi = names.index('x') if 'x' in names else 0
            yi = names.index('y') if 'y' in names else 1
            return feats[xi], feats[yi]

        self._points = {p['name']: _xy(p) for p in auction_points}
        self._visited: set = set()
        self._lock = threading.Lock()

        for drone in drones:
            self.create_subscription(
                PoseStamped,
                f'/{drone}/self_localization/pose',
                lambda msg, d=drone: self._pose_cb(msg, d),
                qos_profile_sensor_data,
            )

        # Write immediately so the file exists before the first timer tick,
        # then keep updating every 2 s so it survives a hard kill.
        self._save()
        self.create_timer(2.0, self._save)

        self.get_logger().info(
            f'Tracking {len(self._points)} waypoints for {drones} '
            f'(threshold={threshold} m) → {output_path}'
        )

    def _pose_cb(self, msg: PoseStamped, drone: str) -> None:
        x = msg.pose.position.x
        y = msg.pose.position.y
        newly = []
        with self._lock:
            for name, (px, py) in self._points.items():
                if name in self._visited:
                    continue
                if math.sqrt((x - px) ** 2 + (y - py) ** 2) <= self._threshold:
                    self._visited.add(name)
                    newly.append(name)
        for name in newly:
            self.get_logger().info(f'[{drone}] visited {name}')

    def _save(self) -> None:
        with self._lock:
            visited = sorted(self._visited)
            remaining = sorted(n for n in self._points if n not in self._visited)
        data = {
            'mission': self._mission_path,
            'saved_at': datetime.now().isoformat(timespec='seconds'),
            'total': len(self._points),
            'visited_count': len(visited),
            'remaining_count': len(remaining),
            'visited': visited,
            'remaining': remaining,
        }
        tmp = self._output_path + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(data, f, indent=2)
        os.replace(tmp, self._output_path)

    def shutdown(self) -> None:
        self._save()
        with self._lock:
            v = len(self._visited)
            r = len(self._points) - v
        self.get_logger().info(
            f'Final save: {v} visited, {r} remaining → {self._output_path}'
        )


def main():
    parser = argparse.ArgumentParser(
        description='Track which auction waypoints each drone has visited'
    )
    parser.add_argument(
        '--mission', required=True,
        help='Path to the mission file (e.g. begin5_mission_solar.py)',
    )
    parser.add_argument(
        '--drones', default='drone0,drone1,drone2,drone3,drone4',
        help='Comma-separated drone namespaces (default: drone0..drone4)',
    )
    parser.add_argument(
        '--output', default='/tmp/mission_progress.json',
        help='Output JSON file (default: /tmp/mission_progress.json)',
    )
    parser.add_argument(
        '--threshold', type=float, default=0.7,
        help='Distance in metres that counts as "visited" (default: 0.7)',
    )
    parser.add_argument(
        '-s', '--use_sim_time', action='store_true', default=False,
        help='Use simulation time',
    )
    args = parser.parse_args()

    mission_path = args.mission
    if not os.path.isabs(mission_path):
        mission_path = os.path.join(os.getcwd(), mission_path)
    if not os.path.exists(mission_path):
        print(f'ERROR: mission file not found: {mission_path}', file=sys.stderr)
        sys.exit(1)

    auction_points = _load_auction_points(mission_path)
    drones = [d.strip() for d in args.drones.split(',') if d.strip()]

    rclpy.init()
    node = ProgressTracker(
        drones=drones,
        auction_points=auction_points,
        output_path=args.output,
        mission_path=mission_path,
        threshold=args.threshold,
        use_sim_time=args.use_sim_time,
    )

    def _on_signal(sig, frame):
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
