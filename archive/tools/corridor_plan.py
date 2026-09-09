#!/usr/bin/env python3
"""Global path generation and goal reachability verification script per corridor width.

    python3 tools/corridor_plan.py
"""

import math
import sys

import rclpy
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node

# (Name, Corridor center y, Designed width)
CORRIDORS = [
    ('c090', 2.6, 0.90),
    ('c070', 0.6, 0.70),
    ('c055', -1.6, 0.55),
    ('c060', -3.0, 0.60),
]
X0, X1 = 2.4, 4.2                      # Both sides of wall (x=3.2)


def pose(x, y):
    from geometry_msgs.msg import PoseStamped
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation.w = 1.0
    return p


def main():
    rclpy.init()
    n = Node('corridor_plan')
    n.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    ac = ActionClient(n, ComputePathToPose, 'compute_path_to_pose')
    if not ac.wait_for_server(timeout_sec=30):
        print('compute_path_to_pose action server unavailable')
        return 1

    print('%-6s %5s  %-9s %8s %9s  %s'
          % ('Corridor', 'Width', 'Result', 'Length', 'EndGap', 'Verdict'))
    for name, y, w in CORRIDORS:
        g = ComputePathToPose.Goal()
        g.start = pose(X0, y)
        g.goal = pose(X1, y)
        g.use_start = True
        fut = ac.send_goal_async(g)
        rclpy.spin_until_future_complete(n, fut, timeout_sec=20)
        gh = fut.result()
        if gh is None or not gh.accepted:
            print('%-6s %5.2f  %-9s' % (name, w, 'Rejected'))
            continue
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(n, rf, timeout_sec=30)
        res = rf.result()
        if res is None:
            print('%-6s %5.2f  %-9s' % (name, w, 'Timeout'))
            continue
        r = res.result
        pts = [(p.pose.position.x, p.pose.position.y) for p in r.path.poses]
        if not pts:
            print('%-6s %5.2f  %-9s %8s %9s  %s'
                  % (name, w, 'NoPath', '-', '-', 'Blocked'))
            continue
        length = sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))
        gap = math.dist(pts[-1], (X1, y))
        if gap > 0.3:
            verdict = 'GoalNotReached(Truncated)'
        elif length < 2.5:
            verdict = 'Traversed'
        else:
            verdict = 'Detour(%.1fx)' % (length / (X1 - X0))
        print('%-6s %5.2f  %-9s %8.2f %9.2f  %s'
              % (name, w, 'Success', length, gap, verdict))

    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
