#!/bin/bash
# Deliberate purge-then-launch: the human typed "clean", that's the consent.
~/auv_ws/src/auv_stack/scripts/kill_stack.sh
exec ros2 launch auv_bringup bench.launch.py "$@"
