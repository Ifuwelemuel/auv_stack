"""Bar02 depth sensor node.

"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Float32
from sensor_msgs.msg import FluidPressure
from std_srvs.srv import Trigger

from auv_control import ms5837   # vendored Blue Robotics driver (not on PyPI)

MBAR_TO_PA = 100.0
G = 9.80665


class DepthSensor(Node):
    def __init__(self):
        super().__init__('depth_sensor')

        self.declare_parameter('i2c_bus', 1)
        # Fresh water 997-1000, seawater ~1025. YAML per deployment site.
        self.declare_parameter('fluid_density', 997.0)
        self.declare_parameter('rate_hz', 10.0)
        # Consecutive-failure ceiling before the sensor is declared dead.
        self.declare_parameter('max_failures', 5)

        p = self.get_parameter
        self._density = p('fluid_density').value
        self._max_fail = p('max_failures').value
        rate = p('rate_hz').value

        self._sensor = ms5837.MS5837_02BA(bus=p('i2c_bus').value)
        if not self._sensor.init():
            # Fail LOUDLY at startup — a depth node that boots without its
            # sensor is a lie waiting for a dive.
            raise RuntimeError('Bar02 init failed — check wiring/power '
                               '(5V rail: level-converter version, ADR-025).')

        self._surface_pa = None       # set by tare; no depth until tared
        self._failures = 0
        self._alive = True

        self._depth_pub = self.create_publisher(Float32, '/depth', 10)
        self._press_pub = self.create_publisher(
            FluidPressure, '/pressure', qos_profile_sensor_data)
        self.create_service(Trigger, '~/tare', self._on_tare)
        self.create_timer(1.0 / rate, self._on_timer)

        self.get_logger().info(
            f'Bar02 up. density={self._density} kg/m3, {rate} Hz. '
            f'UNTARED — call ~/tare at the surface before diving.')

    # ------------------------------------------------------------------ #
    def _on_tare(self, request, response):
        """Capture current pressure as the surface reference. The pre-dive
        checklist owns calling this; the vehicle must be AT the surface."""
        if not self._alive or not self._sensor.read():
            response.success = False
            response.message = 'Tare failed: sensor not reading.'
            return response
        self._surface_pa = self._sensor.pressure() * MBAR_TO_PA
        response.success = True
        response.message = f'Tared at {self._surface_pa:.1f} Pa.'
        self.get_logger().info(response.message)
        return response

    def _on_timer(self):
        if not self._alive:
            return
        if not self._try_read():
            return

        pa = self._sensor.pressure() * MBAR_TO_PA

        msg = FluidPressure()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'bar02_link'
        msg.fluid_pressure = pa
        self._press_pub.publish(msg)

        # Depth only once tared: publishing a wrong-datum depth is worse
        # than publishing none (same principle as the untared -0.144 m).
        if self._surface_pa is not None:
            depth = (pa - self._surface_pa) / (self._density * G)
            self._depth_pub.publish(Float32(data=float(depth)))

    def _try_read(self) -> bool:
        """Retry-with-limit. Returns True if this cycle produced data."""
        try:
            if self._sensor.read():
                if self._failures:
                    self.get_logger().info(
                        f'Sensor recovered after {self._failures} failure(s).')
                self._failures = 0
                return True
            raise OSError('read() returned False')
        except OSError as e:
            self._failures += 1
            self.get_logger().warn(
                f'Bar02 read failed ({self._failures}/{self._max_fail}): {e}')
            if self._failures >= self._max_fail:
                self._alive = False
                self.get_logger().error(
                    'Bar02 DEAD: consecutive failure limit hit. Publishing '
                    'STOPPED (silence = fail-safe). Attempting re-init...')
                try:
                    if self._sensor.init():
                        self._alive = True
                        self._failures = 0
                        self.get_logger().warn(
                            'Bar02 re-initialised — datum unchanged, but '
                            'investigate before trusting a dive.')
                except OSError:
                    pass
            return False


def main(args=None):
    rclpy.init(args=args)
    node = DepthSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()