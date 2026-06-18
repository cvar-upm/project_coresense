import argparse
import json
import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import yaml

from as2_interface.mission_manager import MissionInterpreter
from swarm_pylib.swarm_pylib import Swarm
from utils.waypoint_markers import publish_panel_markers, WaypointMarkerTracker

GROUND_Z = 0.1
POSE_TIMEOUT = 10.0
WAYPOINT_REACH_DIST = 0.5  # metres: drone is considered to have visited a waypoint within this radius


def build_flight_plan(mission: dict, takeoff_height: float) -> dict:
    flight = {}
    for uav, legs in mission.items():
        waypoints = []
        for leg in legs:
            speed = float(leg["speed"])
            if leg["name"] == "LandPoint":
                continue
            for point in leg["values"]:
                if float(point[2]) <= GROUND_Z:
                    continue
                waypoints.append(([float(point[0]), float(point[1]), float(point[2])], speed))
        flight[uav] = {"takeoff_height": takeoff_height, "waypoints": waypoints}
    return flight


def build_drone_plan(takeoff_height: float, waypoints: list) -> list:
    steps = [{"behavior": "takeoff", "args": {"height": takeoff_height, "speed": 1.0}}]
    for goal, speed in waypoints:
        steps.append({"behavior": "go_to", "args": {
            "x": goal[0], "y": goal[1], "z": goal[2], "speed": float(speed)
        }})
    steps.append({"behavior": "land", "args": {"speed": 0.5}})
    return steps


def _nearest_speed(pt: list, positions: list, speeds: list) -> float:
    """Return the speed of the waypoint in positions closest to pt."""
    best_spd = speeds[0]
    best_d = float('inf')
    for pos, spd in zip(positions, speeds):
        d = sum((a - b) ** 2 for a, b in zip(pt, pos))
        if d < best_d:
            best_d = d
            best_spd = spd
    return best_spd


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


class MissionExecutorNode:
    """Long-running ROS 2 node that:
    - Sends the initial coverage missions via MissionUpdate.EXECUTE.
    - Tracks each drone's position against its waypoint list.
    - When a failure-injection condition is met, stops the failed drone,
      pauses survivors, replans with swarm_pylib, and sends new missions.
    """

    def __init__(self, flight: dict, failure_cfg: dict, takeoff_height: float, use_sim_time: bool,
                 wp_tracker=None):
        import rclpy
        from rclpy.node import Node
        from as2_msgs.msg import MissionUpdate
        from geometry_msgs.msg import PoseStamped

        self._MissionUpdate = MissionUpdate
        self._flight = flight
        self._failure_cfg = failure_cfg  # {drone_name: {'at_waypoint': N}}
        self._takeoff_height = takeoff_height

        class _Node(Node):
            pass

        self._node = _Node('mission_executor')
        self._node.set_parameters([
            rclpy.parameter.Parameter(
                'use_sim_time', rclpy.parameter.Parameter.Type.BOOL, use_sim_time
            )
        ])

        self._lock = threading.Lock()

        # Per-drone waypoint tracking
        self._wp_pos = {name: [list(wp[0]) for wp in plan['waypoints']]
                        for name, plan in flight.items()}
        self._wp_spd = {name: [float(wp[1]) for wp in plan['waypoints']]
                        for name, plan in flight.items()}
        self._next_wp = {name: 0 for name in flight}
        self._cur_pos = {name: [0.0, 0.0, 0.0] for name in flight}
        self._active = set(flight.keys())
        self._failed: set = set()
        self._replanned = False
        self._failure_event = threading.Event()
        self._done_event    = threading.Event()
        self._total_wps     = sum(len(wps) for wps in self._wp_pos.values())
        self._visited_wps   = 0
        self._wp_tracker = wp_tracker

        self._pubs = {
            name: self._node.create_publisher(MissionUpdate, f'/{name}/mission_update', 10)
            for name in flight
        }

        from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
        pose_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        for name in flight:
            self._node.create_subscription(
                PoseStamped,
                f'/{name}/self_localization/pose',
                self._make_pose_cb(name),
                pose_qos
            )

    def _make_pose_cb(self, name: str):
        def _cb(msg):
            pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
            with self._lock:
                self._cur_pos[name] = pos
                # Advance the waypoint counter while the drone is within reach of the next waypoint
                while self._next_wp[name] < len(self._wp_pos[name]):
                    wp = self._wp_pos[name][self._next_wp[name]]
                    dist = math.sqrt(sum((a - b) ** 2 for a, b in zip(pos, wp)))
                    if dist < WAYPOINT_REACH_DIST:
                        visited_idx = self._next_wp[name]
                        self._next_wp[name] += 1
                        self._visited_wps += 1
                        drone_total = len(self._wp_pos[name])
                        print(
                            f'[{name}] waypoint {visited_idx + 1}/{drone_total} visited'
                            f'  —  fleet {self._visited_wps}/{self._total_wps}',
                            flush=True,
                        )
                        if self._wp_tracker:
                            self._wp_tracker.mark_visited(name, visited_idx)
                    else:
                        break
                if (self._total_wps > 0
                        and all(self._next_wp[n] >= len(self._wp_pos[n])
                                for n in self._active)):
                    self._done_event.set()
                # Check failure-injection condition
                fail = self._failure_cfg.get(name)
                if fail and not self._replanned and name not in self._failed:
                    if self._next_wp[name] >= fail['at_waypoint']:
                        self._failed.add(name)
                        self._failure_event.set()
        return _cb

    def _publish(self, name: str, action: int, steps: list = None) -> None:
        msg = self._MissionUpdate()
        msg.drone_id = name
        msg.mission_id = 0
        msg.action = action
        if steps is not None:
            msg.mission = json.dumps({'target': name, 'plan': steps})
        self._pubs[name].publish(msg)

    def _do_replan(self) -> None:
        with self._lock:
            survivors = sorted(self._active - self._failed)
            failed = sorted(self._failed)

        if not survivors:
            print('[replan] no survivors — cannot redistribute')
            return

        # 1. Stop failed drones, pause survivors (hover)
        for name in failed:
            self._publish(name, self._MissionUpdate.STOP)
        for name in survivors:
            self._publish(name, self._MissionUpdate.PAUSE)
        print(f'[replan] stopped {failed}, paused {survivors}')

        time.sleep(0.3)  # let drones stabilise before reading their poses

        with self._lock:
            # Collect remaining (unvisited) waypoints from all failed drones
            remaining_positions: list = []
            remaining_speeds: list = []
            for name in failed:
                idx = self._next_wp[name]
                remaining_positions.extend(self._wp_pos[name][idx:])
                remaining_speeds.extend(self._wp_spd[name][idx:])

            # Snapshot survivors' own remaining waypoints and current positions
            survivor_own = {
                s: [(self._wp_pos[s][j], self._wp_spd[s][j])
                    for j in range(self._next_wp[s], len(self._wp_pos[s]))]
                for s in survivors
            }
            cur = {s: list(self._cur_pos[s]) for s in survivors}

        print(f'[replan] redistributing {len(remaining_positions)} waypoints among {survivors}')

        # 2. Redistribute using binpat with survivors' current positions as starting points
        if remaining_positions:
            uavs_state = {
                s: {'initial_position': cur[s], 'last_position': cur[s]}
                for s in survivors
            }
            init_list, last_list, w_list = Swarm.data_input_process(uavs_state, None)
            redistributed = Swarm.distribute_path(
                'binpat', init_list, last_list, remaining_positions, w_list
            )
        else:
            redistributed = [[] for _ in survivors]

        # 3. Build new plans, update waypoint tracking, then publish
        new_plans = {}
        for i, name in enumerate(survivors):
            share = [
                (list(pt), _nearest_speed(pt, remaining_positions, remaining_speeds))
                for pt in redistributed[i]
            ]
            combined = survivor_own[name] + share
            new_plans[name] = (combined, build_drone_plan(self._takeoff_height, combined))
            print(f'[replan] {name}: {len(survivor_own[name])} own + {len(share)} redistributed waypoints')

        with self._lock:
            self._active -= self._failed
            for name, (combined, _) in new_plans.items():
                self._wp_pos[name]  = [list(wp[0]) for wp in combined]
                self._wp_spd[name]  = [float(wp[1]) for wp in combined]
                self._next_wp[name] = 0
            self._total_wps = sum(len(self._wp_pos[n]) for n in self._active)
            if self._total_wps == 0:
                self._done_event.set()
            self._replanned = True

        for name, (_, steps) in new_plans.items():
            self._publish(name, self._MissionUpdate.EXECUTE, steps)

    def run(self) -> None:
        import rclpy

        # Wait for each drone's mission interpreter to subscribe
        timeout = 10.0
        for name, pub in self._pubs.items():
            deadline = time.time() + timeout
            while pub.get_subscription_count() == 0:
                if time.time() > deadline:
                    self._node.get_logger().warn(f'[{name}] no subscriber after {timeout}s')
                    break
                time.sleep(0.05)
            self._node.get_logger().info(f'[{name}] subscriber ready')

        # Send the initial coverage mission to every drone
        for name, plan in self._flight.items():
            steps = build_drone_plan(plan['takeoff_height'], plan['waypoints'])
            self._publish(name, self._MissionUpdate.EXECUTE, steps)
            self._node.get_logger().info(f'[{name}] sent {len(steps)} steps')

        if not self._failure_cfg:
            while not self._done_event.is_set():
                rclpy.spin_once(self._node, timeout_sec=0.1)
            return

        # Spin in background so pose callbacks fire; wait for failure signal in main thread
        def _spin():
            while not self._done_event.is_set():
                try:
                    rclpy.spin_once(self._node, timeout_sec=0.1)
                except Exception:
                    break

        spin_thread = threading.Thread(target=_spin, daemon=True)
        spin_thread.start()

        self._failure_event.wait()
        self._do_replan()
        self._done_event.wait()
        spin_thread.join(timeout=5.0)

    def destroy(self) -> None:
        self._node.destroy_node()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("mission")
    parser.add_argument("--use-sim-time", action="store_true")
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--no-panels", action="store_true",
                        help="Skip publishing solar panel markers to the AR overlay")
    args = parser.parse_args()

    with open(args.mission, "r", encoding="utf-8") as handle:
        payload = yaml.safe_load(handle)
    use_cartesian = bool(payload.get("use_cartesian_coordinates", False))
    takeoff_height = float(payload.get("takeoff_height", 2.0))
    max_wp_distance = payload.get("max_wp_distance")
    if max_wp_distance is not None:
        max_wp_distance = float(max_wp_distance)
    filter_collinear = bool(payload.get("filter_collinear", True))
    uav_names = [str(u) for u in payload["uavList"]]
    failure_cfg = payload.get('failure_injection') or {}

    import rclpy
    rclpy.init()

    initial_positions = read_initial_positions(uav_names, args.use_sim_time, args.verbose)
    _, mission = MissionInterpreter.planner(0, payload, use_cartesian, initial_positions, max_wp_distance, filter_collinear)
    flight = build_flight_plan(mission, takeoff_height)

    node = MissionExecutorNode(flight, failure_cfg, takeoff_height, args.use_sim_time)

    if not args.no_panels:
        all_waypoints = []
        for _uav, legs in mission.items():
            for leg in legs:
                if leg['name'] == 'LandPoint':
                    continue
                points = leg['values']
                if leg['name'] == 'Area' and len(points) > 2:
                    points = points[1:-1]
                for point in points:
                    if float(point[2]) > GROUND_Z:
                        all_waypoints.append([float(point[0]), float(point[1]), float(point[2])])
        publish_panel_markers(node._node, all_waypoints)

        drone_waypoints = {
            name: [wp[0] for wp in plan['waypoints']]
            for name, plan in flight.items()
        }
        node._wp_tracker = WaypointMarkerTracker(node._node, drone_waypoints)
    try:
        node.run()
    finally:
        node.destroy()
        rclpy.shutdown()

    print("Mission complete")
    return 0


if __name__ == "__main__":
    sys.exit(main())
