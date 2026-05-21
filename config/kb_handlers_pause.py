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

from as2_msgs.msg import MissionUpdate
from as2_python_api.kb_monitor.kb_event_handler import KBHandlerContext

FOLLOW_PATH_SPEED = 1.0  # m/s
FOLLOW_PATH_HEIGHT = 1.0  # meters (z kept constant; auction assigns 2-D points)


def on_auction_started(bindings: list, ctx: KBHandlerContext) -> None:
    """Stop the running mission so the drone hovers while the auction runs."""
    print(f'[kb_monitor] {ctx.drone_namespace}: auction started detected')

    # Query to confirm droneStatus is not failure
    status = ctx.query([f'{ctx.drone_namespace} droneStatus ?status'])
    if status:
        print(
            f'[kb_monitor] {ctx.drone_namespace}: no droneStatus found, cannot confirm auction start'
        )
        return

    msg = MissionUpdate()
    msg.drone_id = ctx.drone_namespace
    msg.mission_id = ctx.mission_status.mission_id
    msg.action = MissionUpdate.PAUSE
    ctx.publish_mission_update(msg)
    print(f'[kb_monitor] {ctx.drone_namespace}: auction started - mission paused')


def on_drone_failure(bindings: list, ctx: KBHandlerContext) -> None:
    """Pause the current mission and re-auction the assigned points to the remaining drones."""
    # Pause the current mission
    mid = ctx.mission_status.mission_id if ctx.mission_status else 0
    pause_msg = MissionUpdate()
    pause_msg.drone_id = ctx.drone_namespace
    pause_msg.mission_id = mid
    pause_msg.action = MissionUpdate.PAUSE
    ctx.publish_mission_update(pause_msg)
    print(f'[kb_monitor] {ctx.drone_namespace}: drone failure detected - mission paused')

    # Query the KB for all points previously assigned to this drone
    assigned = ctx.query(
        [
            f'?point assignedTo {ctx.drone_namespace}',
            '?point xCoord ?x',
            '?point yCoord ?y',
        ]
    )

    finished = ctx.query(
        [
            f'?point assignedTo {ctx.drone_namespace}',
            '?goto status finished',
            '?goto to ?point',
            '?point xCoord ?x',
            '?point yCoord ?y',
        ]
    )

    # Extract points from finished in the same format as assigned for easy comparison
    finished_points = [{'point': r['point'], 'x': r['x'], 'y': r['y']} for r in finished]

    # Filter out points that have already been visited
    assigned = [
        r
        for r in assigned
        if not any(
            r['point'] == f['point'] and r['x'] == f['x'] and r['y'] == f['y']
            for f in finished_points
        )
    ]

    # print points for debugging
    print(f'[kb_monitor] {ctx.drone_namespace}: points not completed: {assigned}')

    if not assigned:
        print(
            f'[kb_monitor] {ctx.drone_namespace}: no assigned points found, nothing to re-auction'
        )
        return

    # Query all known drones (those with any auctionStatus), exclude the failing drone
    bidders = [
        'drone0',
        'drone2',
        'drone3',
        # 'drone4',
    ]  # hardcoded for simplicity; in practice, query the KB for active drones

    if not bidders:
        print(f'[kb_monitor] {ctx.drone_namespace}: no available bidders for re-auction')
        return

    # Build AuctionItem-compatible element list from the assigned points
    elements = [
        {
            'name': r['point'],
            'feature_names': ['x', 'y'],
            'features': [float(r['x']), float(r['y'])],
        }
        for r in assigned
    ]

    # Send a new EXECUTE mission that runs an auction to redistribute the points
    mission = {
        'target': ctx.drone_namespace,
        'plan': [
            {
                'behavior': 'auction',
                'args': {
                    'name': 'failure_recovery_auction',
                    'elements': elements,
                    'auction_type': 'coordinate_item',
                    'bidders': bidders,
                },
            },
            {'behavior': 'land', 'args': {}},
        ],
    }

    exec_msg = MissionUpdate()
    exec_msg.drone_id = ctx.drone_namespace
    exec_msg.mission_id = 2  # recovery mission id
    exec_msg.action = MissionUpdate.EXECUTE
    exec_msg.mission = json.dumps(mission)
    ctx.publish_mission_update(exec_msg)
    print(
        f'[kb_monitor] {ctx.drone_namespace}: sent re-auction mission '
        f'for {len(elements)} point(s) to bidders {bidders}'
    )


def on_auction_completed(bindings: list, ctx: KBHandlerContext) -> None:
    """Build a follow_path mission through all KB-assigned points and execute it."""
    # Collect every point assigned to this drone from the KB.
    # The C++ behavior writes: add_fact(id_point, "as2:x", x),
    #                          add_fact(id_point, "as2:y", y),
    #                          add_fact(id_point, "as2:assignedTo", agent_id)

    print(f'[kb_monitor] {ctx.drone_namespace}: auction completed - building follow_path mission')

    # Remove auctionStatus facts from the kb
    print(f'[kb_monitor] {ctx.drone_namespace}: removing auctionStatus facts from KB')
    ctx.remove_fact(f'{ctx.drone_namespace} auctionStatus ?status')

    # Query the KB for all points previously assigned to this drone
    assigned = ctx.query(
        [
            f'?point assignedTo {ctx.drone_namespace}',
            '?point xCoord ?x',
            '?point yCoord ?y',
        ]
    )

    finished = ctx.query(
        [
            f'?point assignedTo {ctx.drone_namespace}',
            '?goto status finished',
            '?goto to ?point',
            '?point xCoord ?x',
            '?point yCoord ?y',
        ]
    )

    # Extract points from finished in the same format as assigned for easy comparison
    finished_points = [{'point': r['point'], 'x': r['x'], 'y': r['y']} for r in finished]

    # Filter out points that have already been visited
    assigned = [
        r
        for r in assigned
        if not any(
            r['point'] == f['point'] and r['x'] == f['x'] and r['y'] == f['y']
            for f in finished_points
        )
    ]

    if not assigned:
        print(
            f'[kb_monitor] {ctx.drone_namespace}: auction completed but no assigned points found'
        )
        return

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
            key=lambda p: (
                (p[0] - current_pos[0]) ** 2 + (p[1] - current_pos[1]) ** 2,
                p[0],
                p[1],
            ),
        )
        sorted_path.append(closest_point)
        remaining_points.remove(closest_point)
        current_pos = closest_point

    plan = [
        {
            # 'behavior': 'go_to',
            'behavior': 'collision_avoidance',
            'args': {
                'x': pt[0],
                'y': pt[1],
                'z': pt[2],
                'speed': FOLLOW_PATH_SPEED,
            },
        }
        for pt in sorted_path
    ]

    mission = {'target': ctx.drone_namespace, 'plan': plan}

    msg = MissionUpdate()
    msg.drone_id = ctx.drone_namespace
    msg.mission_id = ctx.mission_status.mission_id + 1 if ctx.mission_status else 1
    msg.action = MissionUpdate.EXECUTE
    msg.mission = json.dumps(mission)
    ctx.publish_mission_update(msg)
    print(
        f'[kb_monitor] {ctx.drone_namespace}: sent {len(sorted_path)} go_to mission(s): {sorted_path}'
    )
