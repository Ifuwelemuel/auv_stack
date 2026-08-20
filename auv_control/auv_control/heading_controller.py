"""Heading controller: closes yaw loop from IMU to fins.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Float32, Header
from sensor_msgs.msg import Imu
from auv_interfaces.msg import ActuatorCommand


def yaw_from_quat(q) -> float:
    """ENU yaw from quaternion (REP-103). Standard ZYX extraction, yaw only."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def wrap(angle: float) -> float:
    """Map any angle to (-pi, pi]. THE heading-control idiom: guarantees the
    controller always turns the short way round."""
    return math.atan2(math.sin(angle), math.cos(angle))


class HeadingController(Node):
    def __init__(self):
        super().__init__('heading_controller')

        # Gains in parameters, never code: bench tuning is YAML edits +
    
        self.declare_parameter('kp', 0.8)     # rad fin per rad error
        self.declare_parameter('ki', 0.0)     # OFF until PD is tuned
        self.declare_parameter('kd', 0.3)
        self.declare_parameter('i_limit', 0.2)          # anti-windup clamp
        self.declare_parameter('cruise_thrust', 0.3)    # fins need flow
        # 10 Hz matches ArduSub's measured ATTITUDE stream rate: running the
        # loop faster than the measurement makes the D-term differentiate a
        # stair-step. Raise BOTH (SR params via QGC + this) together, later.
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('imu_timeout_s', 0.5)
        self.declare_parameter('imu_topic', '/mavros/mavros/data')

        p = self.get_parameter
        self._kp, self._ki, self._kd = p('kp').value, p('ki').value, p('kd').value
        self._i_lim = p('i_limit').value
        self._cruise = p('cruise_thrust').value
        self._imu_timeout = p('imu_timeout_s').value
        rate = p('rate_hz').value

        self._yaw = None          # None until first IMU: publish NOTHING before
        self._setpoint = None     # None until first setpoint: no default heading
        self._last_imu_time = None
        self._integral = 0.0
        self._prev_error = None
        self._dt = 1.0 / rate

        # Sensor QoS (best-effort): mavros publishes sensor-data QoS; a
        # reliable subscriber here would silently match nothing.
        self.create_subscription(
            Imu, p('imu_topic').value, self._on_imu, qos_profile_sensor_data)
        self.create_subscription(
            Float32, '/cmd/heading_setpoint', self._on_setpoint, 10)

        self._cmd_pub = self.create_publisher(ActuatorCommand, '/cmd/autonomy', 10)
        self._timer = self.create_timer(self._dt, self._on_timer)

        self.get_logger().info(
            f'Heading controller up. kp={self._kp} ki={self._ki} kd={self._kd} '
            f'cruise={self._cruise} imu={p("imu_topic").value}. '
            f'Waiting for IMU and setpoint.')

    def _on_imu(self, msg: Imu):
        self._yaw = yaw_from_quat(msg.orientation)
        self._last_imu_time = self.get_clock().now()

    def _on_setpoint(self, msg: Float32):
        self._setpoint = wrap(msg.data)
        self._integral = 0.0          # new goal, clean slate: stale integral
        self._prev_error = None       # from the old goal is pure windup

    def _on_timer(self):
        # Publish nothing until we have BOTH a measurement and a goal, and
        # nothing if the IMU goes stale — silence lets the mixer's staleness
        # rejection age us out. Silence IS the fail-safe here.
        if self._yaw is None or self._setpoint is None:
            return
        age = (self.get_clock().now() - self._last_imu_time).nanoseconds * 1e-9
        if age > self._imu_timeout:
            self.get_logger().warn('IMU stale — controller muted.',
                                   throttle_duration_sec=2.0)
            return

        error = wrap(self._setpoint - self._yaw)

        self._integral += error * self._dt
        self._integral = max(-self._i_lim, min(self._i_lim, self._integral))
        deriv = 0.0 if self._prev_error is None else \
            wrap(error - self._prev_error) / self._dt
        self._prev_error = error

        fin = self._kp * error + self._ki * self._integral + self._kd * deriv

        cmd = ActuatorCommand()
        cmd.header = Header()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.source = ActuatorCommand.SOURCE_AUTONOMY
        cmd.thruster = self._cruise
        cmd.fin_yaw = fin            # mixer clamps to its yaw limit
        cmd.fin_pitch = 0.0
        cmd.ballast_rate = 0.0
        self._cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = HeadingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()