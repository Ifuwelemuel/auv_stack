"""Waypoint manager: the mission brain and the operator's replacement.

"""
import math

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from std_msgs.msg import Bool, Header
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Trigger

from auv_control.local_frame import LocalFrame


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

        frame = m.get('frame')
        if frame not in ('local', 'global'):
            raise RuntimeError(
                f"mission 'frame' must be 'local' or 'global', got: {frame!r}. "
                f"Ambiguous coordinates are rejected, not guessed (ADR-031).")
        self._frame_kind = frame
        self._raw_wps = [tuple(w) for w in m['waypoints']]
        # local: usable immediately. global: conversion deferred until the
        # origin arrives (it may not exist yet — born at the first fix).
        self._wps = self._raw_wps if frame == 'local' else None

        self._accept = float(m['acceptance_radius_m'])
        self._timeout = float(m['mission_timeout_s'])
        g = m['geofence']
        self._fence = (g['east_min'], g['east_max'],
                       g['north_min'], g['north_max'])

        self._origin = None
        self._pos = None
        self._last_pos_time = None
        self._idx = None            # None = idle; int = current waypoint
        self._start_time = None
        self._pos_timeout = p('position_timeout_s').value

        self.create_subscription(Point, '/guidance/position',
                                 self._on_position, 10)

        # Origin from the frame authority: latched (transient-local) so this
        # node receives it even if it boots after the origin was born.
        latched = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(NavSatFix, '/guidance/origin',
                                 self._on_origin, latched)

        self._target_pub = self.create_publisher(Point, '/guidance/target', 10)
        self._hb_pub = self.create_publisher(Header, '/heartbeat', 10)
        self._estop_pub = self.create_publisher(Bool, '/estop_request', 10)
        self.create_service(Trigger, '~/start_mission', self._on_start)
        self.create_timer(1.0 / p('rate_hz').value, self._on_timer)

        # Boot log must speak the truth of BOTH states: a global mission
        # has no converted waypoints yet, and len(None) is a crash.
        wps_desc = (f'{len(self._wps)} waypoints' if self._wps is not None
                    else f'{len(self._raw_wps)} GLOBAL waypoints (awaiting origin)')
        self.get_logger().info(
            f'Waypoint manager up. {wps_desc}, '
            f'accept={self._accept} m, timeout={self._timeout} s, '
            f'fence={self._fence}. IDLE — call ~/start_mission.')

    # ------------------------------------------------------------------ #
    def _on_position(self, msg: Point):
        self._pos = (msg.x, msg.y)
        self._last_pos_time = self.get_clock().now()

    def _on_origin(self, msg: NavSatFix):
        if self._origin is not None:
            return                      # origin is born once; ignore repeats
        self._origin = (msg.latitude, msg.longitude)
        if self._frame_kind == 'global':
            frame = LocalFrame(*self._origin)
            self._wps = [frame.to_local(lat, lon)
                         for lat, lon in self._raw_wps]
            self.get_logger().info(
                f'Global mission converted: {len(self._wps)} waypoints, '
                f'first at ({self._wps[0][0]:.1f} E, {self._wps[0][1]:.1f} N) '
                f'relative to origin.')

    def _on_start(self, request, response):
        # Frame readiness first (global missions may still await the
        # origin), then position readiness. Both fail closed.
        if self._wps is None:
            response.success = False
            response.message = 'Refused: global mission awaiting origin (no GPS yet).'
            return response
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