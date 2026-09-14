"""
QoS for the link topics. Import these; do not invent your own.

A subscriber whose QoS is incompatible with the publisher receives nothing and
reports nothing - the classic case is a RELIABLE subscriber on a BEST_EFFORT
publisher. Both ends importing the same profile makes that impossible.

  STATE_QOS    /zeus/state    best effort, keep last 1
               A 1 kHz measurement stream: the newest sample is the only one
               worth having, and a retransmitted old one is worse than none.

  COMMAND_QOS  /zeus/command  reliable, keep last 1
               At 250 Hz a lost command costs the STM32 a 4 ms segment it has
               to hold; keeping only the newest means a slow link never
               delivers a stale correction in place of a fresh one.
"""

from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

STATE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
)

COMMAND_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
)
