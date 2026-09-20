# A correction to the stored gait, published on /zeus/command at ~250 Hz.
#
# The STM32 is already playing a reference trajectory; this is only the nudge
# on top of it. For every joint:
#
#     drive target = ref_angle + residual_rad
#
# Send all zeros with enable true and the robot walks the stored gait.
#
# UNITS: RADIANS on the output shaft, like every angle in NexusState. The USB
# protocol carries turns; zeus_link converts, so nothing on the ROS side ever
# has to. The STM32 rejects any |residual| above 0.1 turns = 0.6283 rad.
#
# Joint order is the J_* order in NexusState.msg.
#
# The board only moves while these keep arriving. Stop publishing and the last
# target is held for 200 ms, then the link fault idles every axis. After any
# fault, publish one message with enable false before enable true is accepted
# again - see the zeus_link README.

std_msgs/Header header

float32[8] residual_rad         # rad, added to NexusState.ref_angle
bool enable                     # false = stand down: idle the axes
