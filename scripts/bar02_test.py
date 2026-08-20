#!/usr/bin/env python3
"""Bar02 standalone smoke test. Prove the sensor before any ROS touches it."""
import time
import ms5837

s = ms5837.MS5837_02BA(bus=1)      # Bar02 = the 2-bar variant, bus 1
if not s.init():
    raise SystemExit("Bar02 init failed — sensor answered i2cdetect but not init; check wiring/voltage.")

print("Bar02 up. Ctrl-C to stop.")
while True:
    if s.read():
        print(f"pressure: {s.pressure():8.2f} mbar   "
              f"temp: {s.temperature():5.2f} C   "
              f"depth(fresh): {s.depth():+6.3f} m")
    else:
        print("read failed")
    time.sleep(0.2)