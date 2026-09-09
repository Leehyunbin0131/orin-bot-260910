#!/usr/bin/env python3
"""Frontier area extraction and autonomous exploration node based on /map occupancy grid.

Clusters unmapped boundary cells (frontiers) and evaluates distance and unexplored area gain
to generate autonomous mapping goals via Nav2 NavigateToPose action.
"""

import math

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from scipy import ndimage
from std_msgs.msg import Bool, ColorRGBA
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')

        p = self.declare_parameter
        # Exploration evaluation period [s]
        p('period', 2.0)
        # Minimum frontier cluster size in cells
        p('min_frontier_cells', 8)
        # Minimum clearance distance [m] (0.33 m = 0.70 m corridor width safety margin)
        p('min_clearance', 0.33)
        # Preferred cell clearance distance [m]
        p('clearance', 0.35)
        # Unexplored area gain weight (score = distance - gain * boundary_length)
        p('gain', 1.5)
        # Blacklist radius for unreachable goals [m]
        p('blacklist_radius', 0.6)
        # Blacklist TTL [s] (0 for permanent)
        p('blacklist_ttl', 120.0)
        p('startup_grace', 15.0)
        # Maximum retries for goal failure at same location
        p('max_retries', 2)
        # Minimum goal dwell time before switching [s]
        p('min_goal_dwell', 5.0)
        # Goal timeout [s]
        p('goal_timeout', 90.0)
        # Stuck detection timeout [s]
        p('stuck_time', 30.0)
        p('min_progress', 0.15)
        # Arrival tolerance [m]
        p('arrive_tolerance', 0.5)
        # Radius to consider goal cluster stale [m]
        p('goal_stale_radius', 0.7)
        # Consecutive empty frontier ticks required to finish
        p('done_ticks', 3)
        # Return to home pose after exploration completes
        p('return_home', True)
        # Retries for returning home
        p('home_retries', 5)
        # Overall exploration timeout [s] (0 for unlimited)
        p('explore_timeout', 0.0)
        p('publish_markers', True)
        # Occupancy thresholds
        p('free_threshold', 25)
        p('occupied_threshold', 65)
        p('robot_frame', 'base_footprint')
        p('map_frame', 'map')

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.period = g('period')
        self.min_cells = int(g('min_frontier_cells'))
        self.min_clearance = g('min_clearance')
        self.clearance = g('clearance')
        self.gain = g('gain')
        self.blacklist_radius = g('blacklist_radius')
        self.blacklist_ttl = g('blacklist_ttl')
        self.startup_grace = g('startup_grace')
        self.max_retries = int(g('max_retries'))
        self.min_goal_dwell = g('min_goal_dwell')
        self.goal_timeout = g('goal_timeout')
        self.stuck_time = g('stuck_time')
        self.min_progress = g('min_progress')
        self.arrive_tolerance = g('arrive_tolerance')
        self.stale_radius = g('goal_stale_radius')
        self.done_ticks = int(g('done_ticks'))
        self.return_home = g('return_home')
        self.home_retries = int(g('home_retries'))
        self.explore_timeout = g('explore_timeout')
        self.pub_markers = g('publish_markers')
        self.free_thr = int(g('free_threshold'))
        self.occ_thr = int(g('occupied_threshold'))
        self.robot_frame = g('robot_frame')
        self.map_frame = g('map_frame')

        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)
        self.ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.markers = self.create_publisher(MarkerArray, 'frontier_markers', 10)

        self.map_msg = None
        self.blacklist = []  # [[(x, y), expire_time_s, fail_count]]
        self.goal_handle = None
        self.goal_xy = None
        self.goal_time = None
        self.goal_seq = 0
        self.all_blacklisted_warned = False
        self.empty_ticks = 0

        self.home = None
        self.going_home = False
        self.home_tries = 0
        self.finished = False

        self.last_pose = None
        self.best_gap = None
        self.last_progress_t = None
        self.visited = 0
        self.t0 = None

        latched = QoSProfile(depth=1)
        latched.reliability = QoSReliabilityPolicy.RELIABLE
        latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(OccupancyGrid, '/map', self._on_map, latched)

        self.costmap_seen = None
        self.costmap = None
        self.create_subscription(
            OccupancyGrid, 'global_costmap/costmap', self._on_costmap, latched)
        self.paused = False
        self.create_subscription(Bool, 'exploration_enabled', self._on_enable, 10)

        self.create_timer(self.period, self._tick)

        self.get_logger().info(
            'Waiting for /map and navigate_to_pose action server to start exploration')

    # ---------------- Inputs ----------------
    def _on_map(self, msg):
        self.map_msg = msg

    def _on_enable(self, msg):
        val = bool(msg.data)
        if not val and not self.paused:
            self.paused = True
            self._cancel()
            self.get_logger().info('Exploration paused by external request')
        elif val and self.paused:
            self.paused = False
            self.get_logger().info('Exploration resumed')

    def _on_costmap(self, msg):
        self.costmap = msg
        if self.costmap_seen is None:
            self.costmap_seen = self.get_clock().now()
            self.get_logger().info('Global costmap verified -- Nav2 ready')

    def _costmap_ok(self, xy):
        """Check if candidate goal cell is free in global costmap."""
        cm = self.costmap
        if cm is None:
            return True
        i = cm.info
        c = int((xy[0] - i.origin.position.x) / i.resolution)
        r = int((xy[1] - i.origin.position.y) / i.resolution)
        if not (0 <= c < i.width and 0 <= r < i.height):
            return False
        v = cm.data[r * i.width + c]
        return v < 99

    def _robot_xy(self):
        try:
            tf = self.buf.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))
            return tf.transform.translation.x, tf.transform.translation.y
        except Exception:
            return None

    # ---------------- Frontiers ----------------
    def _frontiers(self):
        """Extract frontier clusters -> list of (center_xy, cell_count, cell_points)."""
        m = self.map_msg
        info = m.info
        res = info.resolution
        grid = np.asarray(m.data, dtype=np.int16).reshape(info.height, info.width)

        free = (grid >= 0) & (grid <= self.free_thr)
        unknown = grid < 0
        occupied = grid > self.occ_thr

        nbr = np.zeros_like(unknown)
        nbr[1:, :] |= unknown[:-1, :]
        nbr[:-1, :] |= unknown[1:, :]
        nbr[:, 1:] |= unknown[:, :-1]
        nbr[:, :-1] |= unknown[:, 1:]
        frontier = free & nbr

        dist = ndimage.distance_transform_edt(~occupied) * res

        lbl, n = ndimage.label(frontier, structure=np.ones((3, 3), bool))
        out = []
        for i in range(1, n + 1):
            ys, xs = np.nonzero(lbl == i)
            if len(xs) < self.min_cells:
                continue
            d = dist[ys, xs]
            if d.max() < self.min_clearance:
                continue
            cx, cy = xs.mean(), ys.mean()
            ok = np.nonzero(d >= min(self.clearance, d.max()))[0]
            k = ok[np.argmin((xs[ok] - cx) ** 2 + (ys[ok] - cy) ** 2)]
            wx = info.origin.position.x + (xs[k] + 0.5) * res
            wy = info.origin.position.y + (ys[k] + 0.5) * res
            pts = np.column_stack([
                info.origin.position.x + (xs + 0.5) * res,
                info.origin.position.y + (ys + 0.5) * res])
            out.append(((wx, wy), len(xs), pts))
        return out

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _blacklisted(self, xy):
        t = self._now()
        self.blacklist = [e for e in self.blacklist if e[1] is None or e[1] > t]
        hits = [e for e in self.blacklist if math.dist(xy, e[0]) < self.blacklist_radius]
        if not hits:
            return False, False
        perm = any(e[1] is None for e in hits)
        return True, perm

    # ---------------- Goals ----------------
    def _send_goal(self, xy, from_xy):
        yaw = math.atan2(xy[1] - from_xy[1], xy[0] - from_xy[0])
        g = NavigateToPose.Goal()
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.header.frame_id = self.map_frame
        g.pose.pose.position.x = xy[0]
        g.pose.pose.position.y = xy[1]
        g.pose.pose.orientation.z = math.sin(yaw / 2.0)
        g.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.goal_seq += 1
        seq = self.goal_seq
        self.goal_xy = xy
        self.goal_time = self.get_clock().now()
        self.best_gap = None
        self.last_progress_t = self.goal_time

        fut = self.ac.send_goal_async(g)
        fut.add_done_callback(lambda f: self._on_accepted(seq, f))
        self.get_logger().info(
            'Goal #%d -> (%.2f, %.2f), distance %.2f m'
            % (seq, xy[0], xy[1], math.dist(xy, from_xy)))

    def _on_accepted(self, seq, fut):
        if seq != self.goal_seq:
            return
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().info('Nav2 action server rejected goal -- retrying next tick')
            self.goal_xy = None
            return
        self.goal_handle = gh
        gh.get_result_async().add_done_callback(lambda f: self._on_result(seq, f))

    def _on_result(self, seq, fut):
        if seq != self.goal_seq:
            return
        status = fut.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            me = self._robot_xy()
            gap = math.dist(me, self.goal_xy) if me else 0.0
            if gap > self.arrive_tolerance:
                self.get_logger().warn(
                    'Goal reported succeeded but robot is %.2f m away -- marking unreachable' % gap)
                self._abandon()
                return
            self.visited += 1
            self.get_logger().info('Goal reached (total %d)' % self.visited)
            self.goal_handle = None
            self.goal_xy = None
        elif status == GoalStatus.STATUS_CANCELED:
            pass
        elif status == GoalStatus.STATUS_UNKNOWN:
            self.get_logger().info('Result status unknown -- retrying')
            self._cancel()
        else:
            self.get_logger().warn('Goal failed (status=%d)' % status)
            self._abandon()

    def _in_grace(self):
        if self.costmap_seen is None:
            return True
        return (self.get_clock().now() - self.costmap_seen).nanoseconds * 1e-9 < self.startup_grace

    def _abandon(self, hard=True):
        if self._in_grace():
            self.get_logger().info('Failure during startup grace period -- retrying without blacklisting')
            self._cancel()
            return
        if self.goal_xy:
            xy = self.goal_xy
            existing = [e for e in self.blacklist if math.dist(xy, e[0]) < self.blacklist_radius]
            fails = (existing[0][2] + 1) if existing else 1
            self.blacklist = [e for e in self.blacklist if math.dist(xy, e[0]) >= self.blacklist_radius]
            if hard or fails >= self.max_retries:
                self.blacklist.append([xy, None, fails])
                self.get_logger().warn(
                    '(%.2f, %.2f) failed %d times -- permanently blacklisted' % (xy[0], xy[1], fails))
            else:
                self.blacklist.append([xy, self._now() + self.blacklist_ttl, fails])
        self._cancel()

    def _least_failed(self, clusters):
        best, best_fails = None, None
        for c, _n, _pts in clusters:
            blocked, perm = self._blacklisted(c)
            if perm or not blocked:
                continue
            hits = [e for e in self.blacklist if math.dist(c, e[0]) < self.blacklist_radius]
            fails = hits[0][2] if hits else 0
            if best_fails is None or fails < best_fails:
                best, best_fails = c, fails
        return best

    def _cancel(self):
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:
                pass
        self.goal_handle = None
        self.goal_xy = None

    # ---------------- Core Loop ----------------
    def _tick(self):
        if self.finished:
            if self.going_home and not self.paused:
                self._tick_home()
            return

        if self.map_msg is None or self.paused:
            return
        if not self.ac.server_is_ready() or self.costmap_seen is None:
            return
        now = self.get_clock().now()
        if self.t0 is None:
            self.t0 = now
            self.get_logger().info('Starting exploration')
        if self.explore_timeout > 0 and \
                (now - self.t0).nanoseconds * 1e-9 > self.explore_timeout:
            self._finish('Explore timeout reached')
            return

        me = self._robot_xy()
        if me is None:
            return
        if self.home is None:
            self.home = me

        clusters = self._frontiers()

        if not clusters:
            self.empty_ticks += 1
            if self.empty_ticks >= self.done_ticks:
                self._cancel()
                self._finish('No remaining frontiers')
            return
        self.empty_ticks = 0

        cands = []
        temporary = 0
        rejected = 0
        for c, n_cells, pts in clusters:
            blocked, perm = self._blacklisted(c)
            if perm:
                continue
            if blocked:
                temporary += 1
                continue
            if not self._costmap_ok(c):
                rejected += 1
                continue
            dist = math.dist(me, c)
            score = dist - self.gain * (n_cells * self.map_msg.info.resolution)
            cands.append((score, c, pts))

        if not cands:
            if temporary == 0:
                self._cancel()
                self._finish('No reachable frontiers remaining (shaded: %d)' % len(clusters))
                return
            retry = self._least_failed(clusters)
            if retry is not None:
                if not self.all_blacklisted_warned:
                    self.get_logger().warn(
                        'All %d frontiers blacklisted -- retrying least failed target' % len(clusters))
                    self.all_blacklisted_warned = True
                if self.goal_xy is None:
                    self._send_goal(retry, me)
            return

        self.all_blacklisted_warned = False
        cands.sort(key=lambda x: x[0])
        best = cands[0][1]

        if self.goal_xy is not None:
            elapsed = (now - self.goal_time).nanoseconds * 1e-9
            if elapsed < self.min_goal_dwell:
                return
            if elapsed > self.goal_timeout:
                self.get_logger().warn('Goal timed out (%.0f s) -- abandoning' % elapsed)
                self._abandon()
                return
            gap = math.dist(me, self.goal_xy)
            if self.best_gap is None or gap < self.best_gap - self.min_progress:
                self.best_gap, self.last_progress_t = gap, now
            elif (now - self.last_progress_t).nanoseconds * 1e-9 > self.stuck_time:
                self.get_logger().warn(
                    'No progress towards goal for %.0f s (gap %.2f m) -- abandoning'
                    % (self.stuck_time, gap))
                self._abandon(hard=False)
                return
            alive = any(
                np.any(np.hypot(pts[:, 0] - self.goal_xy[0],
                                pts[:, 1] - self.goal_xy[1]) < self.stale_radius)
                for _c, _n, pts in clusters)
            if alive:
                return
            self.get_logger().info('Goal area cleared -- switching to next frontier')

        self._send_goal(best, me)

        if self.pub_markers:
            self._draw(clusters)

    def _finish(self, why):
        self.finished = True
        dt = (self.get_clock().now() - (self.t0 or self.get_clock().now())).nanoseconds * 1e-9
        self.get_logger().info(
            '=== Exploration Complete: %s === visited %d, abandoned %d, %.0f s'
            % (why, self.visited, len(self.blacklist), dt))
        if self.return_home and self.home is not None:
            self.get_logger().info('Returning to home pose (%.2f, %.2f)' % self.home)
            self.going_home = True
            self.home_tries = 0
            self._send_home()

    def _send_home(self):
        self.home_tries += 1
        self._send_goal(self.home, self._robot_xy() or self.home)

    def _tick_home(self):
        me = self._robot_xy()
        if me is not None and math.dist(me, self.home) <= self.arrive_tolerance:
            self.get_logger().info('Returned home successfully')
            self.going_home = False
            return
        if self.goal_handle is None and self.goal_xy is None:
            if self.home_tries >= self.home_retries:
                self.get_logger().warn(
                    'Return home failed %d times -- stopping at current location' % self.home_tries)
                self.going_home = False
                return
            self.get_logger().info('Retrying return home (%d/%d)'
                                   % (self.home_tries + 1, self.home_retries))
            self.send_home()

    # ---------------- Visualization ----------------
    def _draw(self, clusters):
        arr = MarkerArray()
        d = Marker()
        d.header.frame_id = self.map_frame
        d.header.stamp = self.get_clock().now().to_msg()
        d.ns = 'frontiers'
        d.id = 0
        d.action = Marker.DELETEALL
        arr.markers.append(d)

        for i, (c, _n, _pts) in enumerate(clusters):
            blocked, perm = self._blacklisted(c)
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'frontiers'
            m.id = i + 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = c[0]
            m.pose.position.y = c[1]
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.25
            if perm:
                m.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
            elif blocked:
                m.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.8)
            else:
                m.color = ColorRGBA(r=0.0, g=1.0, b=0.2, a=0.8)
            arr.markers.append(m)
        self.markers.publish(arr)


def main():
    rclpy.init()
    n = FrontierExplorer()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
