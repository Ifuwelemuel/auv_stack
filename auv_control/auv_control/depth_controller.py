"""Depth controller: closes the depth loop from Bar02 to pitch-fin command.

"""
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32


class DepthController(Node):
    def __init__(self):
        super().__init__('depth_controller')

        self.declare_parameter('kp', 0.6)      # rad fin per metre of error
        self.declare_parameter('ki', 0.05)     # ON from day one — trim exists
        self.declare_parameter('kd', 0.4)
        self.declare_parameter('i_limit', 0.25)
        self.declare_parameter('rate_hz', 10.0)          # matches /depth
        self.declare_parameter('depth_timeout_s', 0.7)
        # Safety ceiling: setpoints beyond this are CLAMPED and logged.
        # Bar02 is a 10 m sensor; vehicle tests far shallower.
        self.declare_parameter('max_depth_m', 2.0)

        p = self.get_parameter
        self._kp, self._ki, self._kd = p('kp').value, p('ki').value, p('kd').value
        self._i_lim = p('i_limit').value
        self._timeout = p('depth_timeout_s').value
        self._max_depth = p('max_depth_m').value

        self._depth = None          # None until first /depth: no guessing
        self._setpoint = None       # None until commanded: no default dive
        self._last_depth_time = None
        self._integral = 0.0
        self._prev_error = None
        self._dt = 1.0 / p('rate_hz').value

        self.create_subscription(Float32, '/depth', self._on_depth, 10)
        self.create_subscription(
            Float32, '/cmd/depth_setpoint', self._on_setpoint, 10)
        self._pitch_pub = self.create_publisher(Float32, '/cmd/pitch', 10)
        self.create_timer(self._dt, self._on_timer)

        self.get_logger().info(
            f'Depth controller up. kp={self._kp} ki={self._ki} kd={self._kd} '
            f'ceiling={self._max_depth} m. Waiting for /depth and setpoint.')

    def _on_depth(self, msg: Float32):
        self._depth = msg.data
        self._last_depth_time = self.get_clock().now()

    def _on_setpoint(self, msg: Float32):
        sp = msg.data
        if sp > self._max_depth:
            self.get_logger().warn(
                f'Setpoint {sp:.2f} m exceeds ceiling — clamped to '
                f'{self._max_depth:.2f} m.')
            sp = self._max_depth
        if sp < 0.0:
            sp = 0.0            # "above the surface" is not a depth
        self._setpoint = sp
        self._integral = 0.0     # new goal, clean slate (same as heading)
        self._prev_error = None

    def _on_timer(self):
        # Silence is the fail-safe: no measurement, no goal, or stale
        # sensor -> publish NOTHING; the composer zeroes stale pitch.
        if self._depth is None or self._setpoint is None:
            return
        age = (self.get_clock().now() - self._last_depth_time).nanoseconds * 1e-9
        if age > self._timeout:
            self.get_logger().warn('/depth stale — depth loop muted.',
                                   throttle_duration_sec=2.0)
            return

        error = self._setpoint - self._depth      # +ve = too shallow = dive

        self._integral += error * self._dt
        self._integral = max(-self._i_lim, min(self._i_lim, self._integral))
        deriv = 0.0 if self._prev_error is None else \
            (error - self._prev_error) / self._dt
        self._prev_error = error

        fin = self._kp * error + self._ki * self._integral + self._kd * deriv
        self._pitch_pub.publish(Float32(data=float(fin)))


def main(args=None):
    rclpy.init(args=args)
    node = DepthController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()