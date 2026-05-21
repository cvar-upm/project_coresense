"""
KB Monitor event handlers for project_coresense.

Handler 1 — on_auction_started:
    Triggered when any drone writes `<drone> as2:auctionStatus "started"` to the KB.
    Sends a STOP to the drone's mission interpreter so it halts its current mission.

Handler 2 — on_auction_completed:
    Triggered when `<drone> as2:auctionStatus "completed"` appears in the KB.
    Queries the KB for all points assigned to this drone, builds a follow_path
    mission JSON that visits every point in order, and sends it via EXECUTE.

Expected KB facts:
    <drone> as2:auctionStatus started            # written when auction begins
    <point_name> as2:assignedTo <drone>          # one triple per assigned point
    <point_name> as2:x <float>
    <point_name> as2:y <float>
"""

import json
import os

from as2_msgs.msg import MissionUpdate
from as2_python_api.kb_monitor.kb_event_handler import KBHandlerContext

PROGRESS_FILE = '/tmp/mission_progress.json'

FOLLOW_PATH_SPEED = 1.0  # m/s
FOLLOW_PATH_HEIGHT = 1.0  # meters (z kept constant; auction assigns 2-D points)


def on_auction_started(bindings: list, ctx: KBHandlerContext) -> None:
    """Stop the running mission so the drone hovers while the auction runs."""
    msg = MissionUpdate()
    msg.drone_id = ctx.drone_namespace
    msg.mission_id = 0  # must match the id used in begin_mission.py (TAKEOFF_MISSION_ID)
    msg.action = MissionUpdate.PAUSE
    ctx.publish_mission_update(msg)
    print(f'[kb_monitor] {ctx.drone_namespace}: auction started - mission paused')


def on_auction_completed(bindings: list, ctx: KBHandlerContext) -> None:
    """Build a follow_path mission through all KB-assigned points and execute it."""
    # Collect every point assigned to this drone from the KB.
    # The C++ behavior writes: add_fact(id_point, "as2:x", x),
    #                          add_fact(id_point, "as2:y", y),
    #                          add_fact(id_point, "as2:assignedTo", agent_id)
    print(f'[kb_monitor] {ctx.drone_namespace}: auction completed - building follow_path mission')
    assigned = ctx.query(
        [
            f'?point assignedTo {ctx.drone_namespace}',
            '?point xCoord ?x',
            '?point yCoord ?y',
        ]
    )

    if not assigned:
        print(
            f'[kb_monitor] {ctx.drone_namespace}: auction completed but no assigned points found'
        )
        return

    # Filter out points already visited in a previous auction round.
    # KB accumulates assignedTo facts across auctions, so without this filter
    # a drone would re-fly waypoints it completed in phase 1.
    visited: set = set()
    if os.path.exists(PROGRESS_FILE):
        with open(PROGRESS_FILE) as _f:
            visited = set(json.load(_f).get('visited', []))
        if visited:
            print(
                f'[kb_monitor] {ctx.drone_namespace}: '
                f'filtering out {len(visited)} already-visited point(s)'
            )

    assigned = [r for r in assigned if r['point'] not in visited]

    if not assigned:
        print(
            f'[kb_monitor] {ctx.drone_namespace}: '
            'all assigned points already visited — nothing to navigate'
        )
        return

    path = [[float(r['x']), float(r['y']), FOLLOW_PATH_HEIGHT] for r in assigned]

    pos_x = ctx.pose.pose.position.x
    pos_y = ctx.pose.pose.position.y
    print(f'[kb_monitor] {ctx.drone_namespace}: assigned points before sorting: {path}')

    # Choose the first point as the starting point, then sort the rest by distance from the previous one
    sorted_path = []
    remaining_points = path.copy()
    current_pos = [pos_x, pos_y, FOLLOW_PATH_HEIGHT]

    while remaining_points:
        # Find the closest point to the current position
        closest_point = min(
            remaining_points,
            key=lambda p: ((p[0] - current_pos[0]) ** 2 + (p[1] - current_pos[1]) ** 2) ** 0.5,
        )
        sorted_path.append(closest_point)
        remaining_points.remove(closest_point)
        current_pos = closest_point

    mission = {
        'target': ctx.drone_namespace,
        'plan': [
            {
                'behavior': 'follow_path',
                'args': {
                    'path': sorted_path,
                    'speed': FOLLOW_PATH_SPEED,
                    'yaw_angle': 0.0,
                },
            }
        ],
    }

    msg = MissionUpdate()
    msg.drone_id = ctx.drone_namespace
    msg.mission_id = 1
    msg.action = MissionUpdate.EXECUTE
    msg.mission = json.dumps(mission)
    ctx.publish_mission_update(msg)
    print(
        f'[kb_monitor] {ctx.drone_namespace}: sent follow_path mission '
        f'through {len(sorted_path)} point(s): {sorted_path}'
    )
