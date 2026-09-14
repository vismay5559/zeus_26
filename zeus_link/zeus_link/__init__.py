"""
zeus_link - the Raspberry Pi's end of the USB link to the STM32.

  nexus_proto   the bytes: packet layouts, CRC, joint map (copied from stm32_zeuss)
  nexus_link    a threaded reader that keeps up with 1 kHz (copied from stm32_zeuss)
  convert       packet <-> ROS message, and the policy observation block
  ports         finds the board among the Pi's serial devices
  link_node     the ROS node that owns the port
"""
