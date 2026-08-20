"""Waypoint manager: the mission brain and the operator's replacement.

"""
import math

import yaml

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Header
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger


class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager')

        self.declare_parameter('mission_file', '')
        self.declare_parameter('rate_hz', 5.0)
        self.declare_parameter('position_timeout_s', 5.0)

        p = self.get_parameter
        path = p('mission_file').value
        if not path:
            raise RuntimeError('mission_file parameter is required.')
        with open(path) as f:
            m = yaml.safe_load(f)['mission']
        self._wps = [tuple(w) for w in m['waypoints']]
        self._accept = float(m['acceptance_radius_m'])
        self._timeout = float(m['mission_timeout_s'])
        g = m['geofence']
        self._fence = (g['east_min'], g['east_max'],
                       g['north_min'], g['north_max'])

        self._pos = None
        self._last_pos_time = None
        self._idx = None            # None = idle; int = current waypoint
        self._start_time = None
        self._pos_timeout = p('position_timeout_s').value

        self.create_subscription(Point, '/guidance/position',
                                 self._on_position, 10)
        self._target_pub = self.create_publisher(Point, '/guidance/target', 10)
        self._hb_pub = self.create_publisher(Header, '/heartbeat', 10)
        self._estop_pub = self.create_publisher(Bool, '/estop_request', 10)
        self.create_service(Trigger, '~/start_mission', self._on_start)
        self.create_timer(1.0 / p('rate_hz').value, self._on_timer)

        self.get_logger().info(
            f'Waypoint manager up. {len(self._wps)} waypoints, '
            f'accept={self._accept} m, timeout={self._timeout} s, '
            f'fence={self._fence}. IDLE — call ~/start_mission.')

    # ------------------------------------------------------------------ #
    def _on_position(self, msg: Point):
        self._pos = (msg.x, msg.y)
        self._last_pos_time = self.get_clock().now()

    def _on_start(self, request, response):
        if self._pos is None:
            response.success = False
            response.message = 'Refused: no position yet (no GPS origin).'
            return response
        self._idx = 0
        self._start_time = self.get_clock().now()
        self._publish_target()
        response.success = True
        response.message = f'Mission started: {len(self._wps)} waypoints.'
        self.get_logger().info(response.message)
        return response

    def _publish_target(self):
        wp = self._wps[self._idx]
        self._target_pub.publish(Point(x=wp[0], y=wp[1], z=0.0))
        self.get_logger().info(
            f'Target {self._idx + 1}/{len(self._wps)}: '
            f'({wp[0]:.1f} E, {wp[1]:.1f} N)')

    def _abort(self, reason: str):
        self.get_logger().error(f'MISSION ABORT: {reason} — engaging e-stop.')
        self._estop_pub.publish(Bool(data=True))
        self._idx = None            # back to idle; restart is deliberate

    def _on_timer(self):
        if self._idx is None:
            return                  # idle: no heartbeat — nobody in command
        now = self.get_clock().now()

        # Position freshness: a mission flying blind is a mission aborted.
        age = (now - self._last_pos_time).nanoseconds * 1e-9
        if age > self._pos_timeout:
            self._abort(f'position stale ({age:.1f} s)')
            return

        # Geofence and timeout: the operator's judgement, mechanised.
        e, n = self._pos
        emin, emax, nmin, nmax = self._fence
        if not (emin <= e <= emax and nmin <= n <= nmax):
            self._abort(f'geofence violation at ({e:.1f}, {n:.1f})')
            return
        elapsed = (now - self._start_time).nanoseconds * 1e-9
        if elapsed > self._timeout:
            self._abort(f'mission timeout ({elapsed:.0f} s)')
            return

        # HEARTBEAT: only while running and healthy. Every check above
        # passed this tick — that is what the heartbeat certifies.
        hb = Header()
        hb.stamp = now.to_msg()
        hb.frame_id = 'waypoint_manager'
        self._hb_pub.publish(hb)

        # Advance on acceptance.
        wp = self._wps[self._idx]
        if math.hypot(e - wp[0], n - wp[1]) <= self._accept:
            self.get_logger().info(f'Waypoint {self._idx + 1} reached.')
            self._idx += 1
            if self._idx >= len(self._wps):
                self.get_logger().info('MISSION COMPLETE.')
                self._idx = None    # idle; heartbeat stops; watchdog will
                                    # trip and safe the vehicle — by design.
            else:
                self._publish_target()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()