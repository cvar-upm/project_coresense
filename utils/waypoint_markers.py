"""Publish solar panel mesh markers and waypoint sphere markers for RViz."""

__authors__ = 'Guillermo GP-Lenza'
__license__ = 'BSD-3-Clause'

import math
import threading
import time
from pathlib import Path

from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

_PROJECT_ROOT = Path(__file__).parent.parent
_MESH_URI = 'file://' + str(
    _PROJECT_ROOT / 'assets' / 'models' / 'solar_panel' / 'meshes' / 'solar_panel.dae'
)
_PANEL_SCALE = 0.01
_PANEL_TOPIC = 'solar_panels'
_FIXED_FRAME = 'earth'


def build_panel_marker_array(waypoints: list, frame_id: str = _FIXED_FRAME) -> MarkerArray:
    """Return a MarkerArray with one MESH_RESOURCE solar panel marker per waypoint."""
    msg = MarkerArray()
    seen: set = set()
    idx = 0
    for wp in waypoints:
        key = (round(float(wp[0]), 3), round(float(wp[1]), 3))
        if key in seen:
            continue
        seen.add(key)

        m = Marker()
        m.header = Header(frame_id=frame_id)
        m.ns = 'solar_panels'
        m.id = idx
        m.type = Marker.MESH_RESOURCE
        m.action = Marker.ADD
        m.mesh_resource = _MESH_URI
        m.mesh_use_embedded_materials = True
        m.pose = Pose(
            position=Point(x=float(wp[0]), y=float(wp[1]), z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        )
        m.scale = Vector3(x=_PANEL_SCALE, y=_PANEL_SCALE, z=_PANEL_SCALE)
        m.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        msg.markers.append(m)
        idx += 1

    return msg


def publish_panel_markers(
    node,
    waypoints: list,
    topic: str = _PANEL_TOPIC,
    frame_id: str = _FIXED_FRAME,
    subscriber_timeout: float = 5.0,
) -> None:
    """Create a publisher on `topic`, wait for the overlay subscriber, then publish once."""
    import rclpy

    import rclpy.qos
    qos = rclpy.qos.QoSProfile(
        depth=10,
        durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
    )
    pub = node.create_publisher(MarkerArray, topic, qos)

    deadline = time.time() + subscriber_timeout
    while pub.get_subscription_count() == 0 and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)

    if pub.get_subscription_count() == 0:
        node.get_logger().warn(
            f'[panels] no subscriber on "{topic}" after {subscriber_timeout}s — '
            'is the overlay node running?'
        )

    msg = build_panel_marker_array(waypoints, frame_id=frame_id)
    pub.publish(msg)
    node.get_logger().info(f'[panels] published {len(msg.markers)} solar panel markers on "{topic}"')


# Per-drone sphere colors (R, G, B)
_DRONE_COLORS: list[tuple[float, float, float]] = [
    (0.18, 0.55, 1.0),   # drone0 — blue
    (1.0,  0.55, 0.0),   # drone1 — orange
    (0.15, 0.80, 0.15),  # drone2 — green
    (0.7,  0.0,  0.9),   # drone3 — purple
    (1.0,  0.1,  0.1),   # drone4 — red
]
_WAYPOINT_SPHERE_RADIUS = 0.25
_WAYPOINT_TOPIC = 'waypoints'
_WAYPOINT_NS = 'waypoints'
_LABEL_NS = 'waypoint_labels'
_LABEL_TEXT_HEIGHT = 0.3
_LABEL_Z_OFFSET = 0.35


class WaypointMarkerTracker:
    """Publishes waypoint sphere markers and supports individual deletion when visited.

    drone_waypoints can be:
      - dict[str, list]  — {drone_name: [[x, y, z], ...]}  (per-drone coloring)
      - list             — [[x, y, z], ...]  (single neutral color, keyed as '')
    """

    def __init__(self, node, drone_waypoints, topic: str = _WAYPOINT_TOPIC,
                 frame_id: str = _FIXED_FRAME):
        import rclpy.qos
        self._frame_id = frame_id

        qos = rclpy.qos.QoSProfile(
            depth=10,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
        )
        self._pub = node.create_publisher(MarkerArray, topic, qos)

        if isinstance(drone_waypoints, dict):
            items = list(drone_waypoints.items())
        else:
            items = [('', drone_waypoints)]

        # _id_map[drone_name][wp_idx] = marker_id
        self._id_map: dict[str, dict[int, int]] = {}
        # _pos_by_id[marker_id] = [x, y, z]  — used for proximity-based deletion
        self._pos_by_id: dict[int, list] = {}
        self._visited: set[int] = set()
        self._lock = threading.Lock()
        markers = []
        marker_id = 0

        for drone_idx, (drone_name, waypoints) in enumerate(items):
            r, g, b = _DRONE_COLORS[drone_idx % len(_DRONE_COLORS)]
            self._id_map[drone_name] = {}
            for wp_idx, wp in enumerate(waypoints):
                m = Marker()
                m.header = Header(frame_id=frame_id)
                m.ns = _WAYPOINT_NS
                m.id = marker_id
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose = Pose(
                    position=Point(x=float(wp[0]), y=float(wp[1]), z=float(wp[2])),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                )
                d = _WAYPOINT_SPHERE_RADIUS * 2
                m.scale = Vector3(x=d, y=d, z=d)
                m.color = ColorRGBA(r=r, g=g, b=b, a=0.85)
                markers.append(m)

                label = Marker()
                label.header = Header(frame_id=frame_id)
                label.ns = _LABEL_NS
                label.id = marker_id
                label.type = Marker.TEXT_VIEW_FACING
                label.action = Marker.ADD
                label.pose = Pose(
                    position=Point(x=float(wp[0]), y=float(wp[1]),
                                   z=float(wp[2]) + _LABEL_Z_OFFSET),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                )
                label.scale = Vector3(x=0.0, y=0.0, z=_LABEL_TEXT_HEIGHT)
                label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
                label.text = f'wp_{marker_id}'
                markers.append(label)

                self._id_map[drone_name][wp_idx] = marker_id
                self._pos_by_id[marker_id] = [float(wp[0]), float(wp[1]), float(wp[2])]
                marker_id += 1

        msg = MarkerArray()
        msg.markers = markers
        self._pub.publish(msg)
        node.get_logger().info(f'[waypoints] published {len(markers)} sphere marker(s) on "{topic}"')

    def _delete_marker(self, marker_id: int) -> None:
        arr = MarkerArray()
        for ns in (_WAYPOINT_NS, _LABEL_NS):
            m = Marker()
            m.header = Header(frame_id=self._frame_id)
            m.ns = ns
            m.id = marker_id
            m.action = Marker.DELETE
            arr.markers.append(m)
        self._pub.publish(arr)

    def mark_visited(self, drone_name: str, wp_idx: int) -> None:
        """Delete the sphere marker at the given drone/waypoint index (used by mission_executor)."""
        marker_id = self._id_map.get(drone_name, {}).get(wp_idx)
        if marker_id is None:
            return
        with self._lock:
            if marker_id in self._visited:
                return
            self._visited.add(marker_id)
        self._delete_marker(marker_id)

    def all_visited(self) -> bool:
        with self._lock:
            return len(self._pos_by_id) > 0 and len(self._visited) >= len(self._pos_by_id)

    def mark_visited_near(self, pos: list, reach_dist: float) -> bool:
        """Delete the closest unvisited sphere within reach_dist of pos (used by send_mission).

        Returns True if a marker was deleted.
        """
        with self._lock:
            best_id, best_dist = None, reach_dist
            for mid, wp in self._pos_by_id.items():
                if mid in self._visited:
                    continue
                d = math.sqrt((pos[0] - wp[0]) ** 2 + (pos[1] - wp[1]) ** 2)
                if d < best_dist:
                    best_dist = d
                    best_id = mid
            if best_id is None:
                return False
            self._visited.add(best_id)
        self._delete_marker(best_id)
        return True


def publish_waypoint_markers(node, drone_waypoints, topic: str = _WAYPOINT_TOPIC,
                             frame_id: str = _FIXED_FRAME) -> WaypointMarkerTracker:
    """Convenience wrapper: create a WaypointMarkerTracker and publish initial markers.

    Returns the tracker so callers can later call mark_visited() if needed.
    """
    return WaypointMarkerTracker(node, drone_waypoints, topic=topic, frame_id=frame_id)
