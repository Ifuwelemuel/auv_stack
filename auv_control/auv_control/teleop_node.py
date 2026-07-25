
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Header
from auv_interfaces.msg import ActuatorCommand


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')

        # --- Button/axis mapping (SET THESE FROM YOUR /joy DISCOVERY) --------
        self.declare_parameter('deadman_button', 4)     # e.g. left bumper
        self.declare_parameter('estop_button', 1)       # e.g. B
        self.declare_parameter('clear_button', 3)       # e.g. Y
        self.declare_parameter('yaw_axis', 0)           # left stick horizontal
        self.declare_parameter('pitch_axis', 4)         # right stick vertical
        self.declare_parameter('thrust_axis', 5)        # right trigger
        self.declare_parameter('ballast_axis', 7)       # d-pad vertical

        # Scales convert normalised stick [-1,1] to actuator ranges.
        self.declare_parameter('yaw_scale', 0.5)        # rad at full stick
        self.declare_parameter('pitch_scale', 0.5)
        self.declare_parameter('thrust_scale', 1.0)
        self.declare_parameter('ballast_scale', 1.0)
        # Right trigger often rests at +1.0, goes to -1.0 when pressed.
        # This flag remaps it to a clean 0..1 forward range.
        self.declare_parameter('trigger_style_thrust', True)

        self._b_dead = self.get_parameter('deadman_button').value
        self._b_estop = self.get_parameter('estop_button').value
        self._b_clear = self.get_parameter('clear_button').value
        self._ax_yaw = self.get_parameter('yaw_axis').value
        self._ax_pitch = self.get_parameter('pitch_axis').value
        self._ax_thrust = self.get_parameter('thrust_axis').value
        self._ax_ballast = self.get_parameter('ballast_axis').value
        self._s_yaw = self.get_parameter('yaw_scale').value
        self._s_pitch = self.get_parameter('pitch_scale').value
        self._s_thrust = self.get_parameter('thrust_scale').value
        self._s_ballast = self.get_parameter('ballast_scale').value
        self._trigger_thrust = self.get_parameter('trigger_style_thrust').value

        # Edge detection for the estop/clear buttons: act on press, not hold.
        self._prev_estop = 0
        self._prev_clear = 0

        # --- Publishers ------------------------------------------------------
        self._cmd_pub = self.create_publisher(ActuatorCommand, '/cmd/teleop', 10)
        self._hb_pub = self.create_publisher(Header, '/heartbeat', 10)

        from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                               QoSReliabilityPolicy, QoSHistoryPolicy)
        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self._estop_pub = self.create_publisher(
            Bool, '/estop_request', latched_qos)

        # --- Subscriber to the joy driver -----------------------------------
        self.create_subscription(
            Joy, '/joy', self._on_joy, qos_profile_sensor_data)

        self.get_logger().info('Teleop up. HOLD deadman button to enable.')

    def _on_joy(self, msg: Joy):
        # Publish heartbeat EVERY joy message. While the controller streams and
        # this node lives, the supervisor sees a live human. Stop either and
        # the watchdog trips.
        hb = Header()
        hb.stamp = self.get_clock().now().to_msg()
        hb.frame_id = 'teleop'
        self._hb_pub.publish(hb)

        # --- Estop / clear buttons: edge-triggered --------------------------
        estop_now = self._button(msg, self._b_estop)
        clear_now = self._button(msg, self._b_clear)
        if estop_now and not self._prev_estop:
            self._estop_pub.publish(Bool(data=True))
            self.get_logger().warn('E-STOP requested from controller.')
        if clear_now and not self._prev_clear:
            self._estop_pub.publish(Bool(data=False))
            self.get_logger().info('E-STOP clear requested from controller.')
        self._prev_estop = estop_now
        self._prev_clear = clear_now

        # --- Deadman: non-zero commands ONLY while held ---------------------
        cmd = ActuatorCommand()
        cmd.header = Header()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.source = ActuatorCommand.SOURCE_TELEOP

        if self._button(msg, self._b_dead):
            cmd.fin_yaw = self._axis(msg, self._ax_yaw) * self._s_yaw
            cmd.fin_pitch = self._axis(msg, self._ax_pitch) * self._s_pitch
            cmd.ballast_rate = self._axis(msg, self._ax_ballast) * self._s_ballast
            raw_thrust = self._axis(msg, self._ax_thrust)
            if self._trigger_thrust:
                # Trigger rests at +1, full press at -1 -> map to 0..1 forward.
                cmd.thruster = (1.0 - raw_thrust) / 2.0 * self._s_thrust
            else:
                cmd.thruster = raw_thrust * self._s_thrust
        # else: all fields remain 0.0 — deadman released = neutral.

        self._cmd_pub.publish(cmd)

    @staticmethod
    def _axis(msg, i):
        return msg.axes[i] if 0 <= i < len(msg.axes) else 0.0

    @staticmethod
    def _button(msg, i):
        return msg.buttons[i] if 0 <= i < len(msg.buttons) else 0


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()