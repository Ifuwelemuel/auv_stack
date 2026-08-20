"""Line-of-sight guidance (Fossen): follow the LINE between waypoints,
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point

from auv_control.local_frame import LocalFrame


class LosGuidance(Node):
    def __init__(self):
        super().__init__('los_guidance')

        self.declare_parameter('lookahead_m', 5.0)
        self.declare_parameter('fix_timeout_s', 3.0)   # GPS is 1 Hz-ish; generous
        self.declare_parameter('rate_hz', 5.0)

        p = self.get_parameter
        self._lookahead = p('lookahead_m').value
        self._fix_timeout = p('fix_timeout_s').value

        self._frame = None          # LocalFrame, born at the FIRST good fix
        self._pos = None            # (east, north) current position
        self._last_fix_time = None
        self._target = None         # (east, north) current waypoint
        self._prev_wp = None        # segment start: previous waypoint or the
                                    # position where the current target was set

        self.create_subscription(NavSatFix, '/buoy/fix',
                                 self._on_fix, qos_profile_sensor_data)
        # Target in LOCAL metres (Point.x=east, y=north). Unit D publishes
        # this; until then: ros2 topic pub for bench tests.
        self.create_subscription(Point, '/guidance/target',
                                 self._on_target, 10)
        self._sp_pub = self.create_publisher(Float32, '/cmd/heading_setpoint', 10)
        self.create_timer(1.0 / p('rate_hz').value, self._on_timer)

        self.get_logger().info(
            f'LOS guidance up. lookahead={self._lookahead} m. '
            f'Waiting for first GPS fix (origin) and a target.')

    # ------------------------------------------------------------------ #
    def _on_fix(self, msg: NavSatFix):
        if msg.status.status < 0:      # NavSatStatus.STATUS_NO_FIX
            return                     # no fix = no data; silence, not guesses
        if self._frame is None:
            self._frame = LocalFrame(msg.latitude, msg.longitude)
            self.get_logger().info(
                f'Origin fixed at ({msg.latitude:.6f}, {msg.longitude:.6f}) '
                f'— local frame (map) is born here (ADR-028).')
        self._pos = self._frame.to_local(msg.latitude, msg.longitude)
        self._last_fix_time = self.get_clock().now()

    def _on_target(self, msg: Point):
        # Segment start = where we are when the target arrives (or the old
        # target if this is a waypoint advance). Unit D formalises this.
        self._prev_wp = self._target if self._target is not None else self._pos
        self._target = (msg.x, msg.y)
        self.get_logger().info(
            f'New target ({msg.x:.1f} E, {msg.y:.1f} N), '
            f'segment from {self._prev_wp}.')

    def _on_timer(self):
        if self._pos is None or self._target is None or self._prev_wp is None:
            return
        age = (self.get_clock().now() - self._last_fix_time).nanoseconds * 1e-9
        if age > self._fix_timeout:
            self.get_logger().warn('GPS stale — guidance muted.',
                                   throttle_duration_sec=5.0)
            return      # silence: heading controller holds last setpoint;
                        # Unit D's mission timeout is the deeper safety net.

        # --- LOS core (Fossen): project, lead, aim ----------------------
        px, py = self._prev_wp
        tx, ty = self._target
        vx, vy = tx - px, ty - py                  # segment vector
        seg_len = math.hypot(vx, vy)
        if seg_len < 1e-6:
            bearing = math.atan2(ty - self._pos[1], tx - self._pos[0])
        else:
            ux, uy = vx / seg_len, vy / seg_len    # unit along-track
            dx, dy = self._pos[0] - px, self._pos[1] - py
            along = dx * ux + dy * uy              # our projection onto the line
            # Aim point: lookahead ahead of the projection, clamped to the
            # segment end so we never aim past the waypoint.
            s = min(along + self._lookahead, seg_len)
            aim = (px + ux * s, py + uy * s)
            bearing = math.atan2(aim[1] - self._pos[1], aim[0] - self._pos[0])

        # ENU bearing: atan2(dN? no —) x=east,y=north: heading measured CCW
        # from EAST, matching yaw_from_quat's convention. One convention,
        # both controllers, zero offsets (ADR-028 notes this explicitly).
        self._sp_pub.publish(Float32(data=float(bearing)))


def main(args=None):
    rclpy.init(args=args)
    node = LosGuidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()