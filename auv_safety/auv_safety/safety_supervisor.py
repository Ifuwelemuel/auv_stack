"""Safety supervisor for the AUV.

Owns the vehicle's safe state and the e-stop latch. 
Responsibilities:
  1. Publish a latched /estop (transient-local) so any node, whenever it
     starts, learns the current stop state immediately.
  2. Publish a continuous safe-state ActuatorCommand on /cmd/safety that the
     mixer selects whenever stop is active or the watchdog has tripped.
  3. Run a watchdog: if the active commander stops sending heartbeats, latch
     stop automatically. Silence is treated as failure.
  4. Accept external stop requests on /estop_request (teleop button, later a
     hardware line) and latch/clear accordingly. A latched stop does NOT
     auto-clear on heartbeat return; clearing is always explicit.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Bool, Header
from auv_interfaces.msg import ActuatorCommand


class SafetySupervisor(Node):
    def __init__(self):
        super().__init__('safety_supervisor')

        # --- Parameters: declared, typed, overridable from YAML/launch -------
        # Never hardcode a safety threshold. On the bench we want a lax
        # timeout so debugging doesn't constantly trip the watchdog; in the
        # water we want it strict. Same code, different YAML.
        self.declare_parameter('watchdog_timeout_s', 0.5)
        self.declare_parameter('publish_rate_hz', 20.0)

        self._timeout_s = self.get_parameter('watchdog_timeout_s').value
        rate = self.get_parameter('publish_rate_hz').value

        # --- QoS profiles ----------------------------------------------------
        # Latched state: reliable + transient-local + keep-last-1. A late
        # subscriber (mixer starting after us) receives the current value on
        # connect. This is THE reason e-stop cannot be volatile.
        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        # Command QoS: reliable, keep-last-1. A dropped command is a fault; a
        # queue of stale commands is worse, so depth is 1 (freshest only).
        cmd_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        # --- Internal state --------------------------------------------------
        # Start LATCHED. The vehicle boots in the stopped state and must be
        # explicitly released. Fail-safe defaults: if anything goes wrong
        # before the operator is ready, we are already safe.
        self._estopped = True
        # Last heartbeat time. Initialise to now so we don't trip instantly
        # before the first heartbeat arrives during normal startup.
        self._last_heartbeat = self.get_clock().now()

        # --- Publishers ------------------------------------------------------
        self._estop_pub = self.create_publisher(Bool, '/estop', latched_qos)
        self._safety_cmd_pub = self.create_publisher(
            ActuatorCommand, '/cmd/safety', cmd_qos)

        # --- Subscribers -----------------------------------------------------
        # Heartbeat from the active commander. We only care about its arrival
        # time, so std_msgs/Header (which carries a stamp) is the lightest
        # honest type. Reliable depth-1: we want to know it arrived.
        self.create_subscription(
            Header, '/heartbeat', self._on_heartbeat, cmd_qos)
        # External stop request. Transient-local so a request latched before
        # this node (re)starts is still honoured.
        self.create_subscription(
            Bool, '/estop_request', self._on_estop_request, latched_qos)

        # --- Timer: the heartbeat of this node itself ------------------------
        # A single timer both runs the watchdog check and republishes state.
        # Timer (not sleep) so it respects simulated time under use_sim_time.
        self._timer = self.create_timer(1.0 / rate, self._on_timer)

        # Publish the initial latched stop immediately so the world knows the
        # vehicle boots safe, even before the first timer tick.
        self._publish_estop()
        self.get_logger().info(
            f'Safety supervisor up. Booted E-STOPPED. '
            f'watchdog={self._timeout_s}s rate={rate}Hz')

    # ------------------------------------------------------------------ #
    #  Callbacks                                                          #
    # ------------------------------------------------------------------ #
    def _on_heartbeat(self, msg: Header):
        """Record that the commander is alive. We use arrival time from our
        own clock rather than msg.stamp, to avoid trusting a remote clock
        that may be skewed — the watchdog measures OUR silence, locally."""
        self._last_heartbeat = self.get_clock().now()

    def _on_estop_request(self, msg: Bool):
        """External latch/clear. True = engage stop. False = request clear.
        Clearing is honoured only as an explicit operator action; the
        watchdog can engage stop but never clears it."""
        if msg.data:
            if not self._estopped:
                self.get_logger().warn('E-STOP engaged by external request.')
            self._estopped = True
        else:
            if self._estopped:
                self.get_logger().info('E-STOP cleared by external request.')
            self._estopped = False
        self._publish_estop()

    def _on_timer(self):
        """Runs at publish_rate_hz. Two jobs: check the watchdog, then
        publish current state. Ordering matters — check first so a trip is
        reflected in the same tick's output."""
        # Watchdog: how long since the last heartbeat?
        elapsed = (self.get_clock().now() - self._last_heartbeat).nanoseconds * 1e-9
        if elapsed > self._timeout_s and not self._estopped:
            self.get_logger().error(
                f'Watchdog TRIPPED: {elapsed:.2f}s since last heartbeat '
                f'(> {self._timeout_s}s). Engaging E-STOP.')
            self._estopped = True
            self._publish_estop()

        # Always republish the safe-state command. The mixer selects it or
        # ignores it based on /estop; our job is to keep offering it fresh so
        # it is never stale when the mixer needs it.
        self._publish_safety_command()

    # ------------------------------------------------------------------ #
    #  Publishing helpers                                                 #
    # ------------------------------------------------------------------ #
    def _publish_estop(self):
        self._estop_pub.publish(Bool(data=self._estopped))

    def _publish_safety_command(self):
        """The definition of 'safe' for this vehicle. Fail-to-surface."""
        cmd = ActuatorCommand()
        cmd.header = Header()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.thruster = 0.0        # no propulsion
        cmd.fin_pitch = 0.0       # neutral control surfaces
        cmd.fin_yaw = 0.0
        cmd.ballast_rate = -1.0   # drive ballast to EMPTY: positive buoyancy
        cmd.source = ActuatorCommand.SOURCE_SAFETY
        self._safety_cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SafetySupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # On shutdown, make a best effort to leave the latched stop TRUE so
        # anything still listening sees a stop as we exit.
        node._estopped = True
        node._publish_estop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()