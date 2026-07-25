# Single authoritative actuator command for the vehicle.
#
# ONLY actuator_mixer publishes this message. Every other control authority
# (teleop, autonomy, safety) publishes its request on a separate topic and the
# mixer arbitrates. This is the command-multiplexer pattern; the wheeled robot
# equivalent is twist_mux.
#
# Conventions: REP-103. Body frame is FLU (x forward, y left, z up).
# Angles in radians, positive by right-hand rule about the body axis.

std_msgs/Header header

float32 thruster
float32 fin_pitch
float32 fin_yaw
float32 ballast_rate

uint8 source

uint8 SOURCE_SAFETY=0
uint8 SOURCE_TELEOP=1
uint8 SOURCE_AUTONOMY=2