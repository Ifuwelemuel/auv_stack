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
     hardware line). Requests can only ENGAGE the stop; a `false` on this
     topic is ignored. Clearing is exclusively via the clear_estop service,
     refused unless a live commander heartbeat is present (ADR-019).
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Bool, Header
from std_srvs.srv import Trigger
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
        # connect. This is THE reason the /estop STATE cannot be volatile.
        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        # Command QoS: reliable, VOLATILE, keep-last-1. A dropped command is a
        # fault; a queue of stale commands is worse, so depth is 1. Volatile
        # because commands and requests are EVENTS — they must never be
        # replayed to a late-joining subscriber.
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
        # FIX 1a: heartbeat time starts at EPOCH, not now(). Until a commander
        # has actually been heard, the vehicle is not clearable — clear_estop
        # will refuse. Boot-time grace windows are how fail-safes leak. Note
        # this cannot cause a watchdog trip-loop at boot: tripping requires
        # `not self._estopped`, and we boot stopped.
        self._last_heartbeat = rclpy.time.Time(clock_type=self.get_clock().clock_type)

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
        # External stop request — ENGAGE ONLY, volatile. A request is an
        # event, not state; volatile durability means a latched historical
        # message can never be replayed into this callback. (Transient-local
        # here is what let a stale `false` clear the boot e-stop — ADR-019.)
        self.create_subscription(
            Bool, '/estop_request', self._on_estop_request, cmd_qos)

        # Clearing is a SERVICE, not a topic: an edge that cannot be latched
        # or replayed, and returns an acknowledgement. Asymmetric authority —
        # anyone may stop the vehicle; only a deliberate, fresh, acknowledged
        # action may un-stop it.
        self.create_service(Trigger, '~/clear_estop', self._on_clear_estop)

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
        # FIX 1b: record NOW. (The epoch construction belongs in __init__
        # only — putting it here made every heartbeat read as 1970 and
        # clear_estop refuse forever.)
        self._last_heartbeat = self.get_clock().now()

    def _on_estop_request(self, msg: Bool):
        """Engage-only. `true` latches the stop from anywhere, any time —
        even a stale or replayed `true` is safe, because engaging an engaged
        stop is a no-op. `false` on this topic is a protocol violation from
        an outdated publisher and is ignored loudly: clearing has exactly one
        path, the clear_estop service (ADR-019)."""
        if msg.data:
            if not self._estopped:
                self.get_logger().warn('E-STOP engaged by external request.')
            self._estopped = True
            self._publish_estop()
        else:
            self.get_logger().warn(
                'Ignored `false` on /estop_request — clearing via topic is '
                'not permitted. Use the clear_estop service.')

    def _on_clear_estop(self, request, response):
        """The ONLY path out of the latched stop. Two conditions:
        1. It happened at all — a deliberate service call, never a replay.
        2. A commander is currently alive — clearing with a stale heartbeat
           would go live for up to timeout_s with nobody in command before
           the watchdog re-trips. An industrial drive will not reset while
           the fault condition persists; a missing commander IS our fault
           condition."""
        elapsed = (self.get_clock().now() - self._last_heartbeat).nanoseconds * 1e-9
        if elapsed > self._timeout_s:
            response.success = False
            response.message = (
                f'Refused: no live heartbeat ({elapsed:.2f}s > '
                f'{self._timeout_s}s). Start the commander first.')
            self.get_logger().warn(f'Clear refused: {response.message}')
            return response

        self._estopped = False
        self._publish_estop()
        response.success = True
        response.message = 'E-stop cleared.'
        self.get_logger().info('E-STOP cleared via service.')
        return response

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