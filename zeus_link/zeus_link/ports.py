"""
Finding the STM32's USB link among the Pi's serial devices.

The board shows up as /dev/ttyACM<n>, but so does the Nucleo's ST-LINK if that
cable is plugged in too, and <n> depends on plug order. Opening the ST-LINK
port by mistake looks exactly like a dead link: the port opens fine and no
packet ever arrives.

So "auto" looks for the link by what it IS, not by where it happens to be:

  1. /dev/zeus_stm32, if the udev rule in zeus_bringup/config is installed
  2. the one serial device with the STM32 USB CDC IDs, 0483:5740
     (the ST-LINK's own port is 0483:374x, so it never matches)
"""

from __future__ import annotations

import os
from typing import Callable, Iterable, Optional

STM32_CDC_VID = 0x0483
STM32_CDC_PID = 0x5740
STABLE_NAME = "/dev/zeus_stm32"


class PortError(RuntimeError):
    pass


def _list_ports():
    from serial.tools import list_ports
    return list_ports.comports()


def find_port(requested: str = "auto",
              comports: Optional[Callable[[], Iterable]] = None,
              exists: Callable[[str], bool] = os.path.exists) -> str:
    """Resolve a port parameter to a device path. Anything but "auto" is used as given."""
    if requested and requested != "auto":
        return requested

    if exists(STABLE_NAME):
        return STABLE_NAME

    ports = list((comports or _list_ports)())
    matches = [p.device for p in ports
               if p.vid == STM32_CDC_VID and p.pid == STM32_CDC_PID]

    if len(matches) == 1:
        return matches[0]

    seen = ", ".join(
        f"{p.device} ({p.vid:04x}:{p.pid:04x})" if p.vid is not None else p.device
        for p in ports) or "none"

    if len(matches) > 1:
        raise PortError(
            f"{len(matches)} devices have the STM32 link's USB IDs ({', '.join(matches)}). "
            f"Pass the one you mean with port:=/dev/ttyACMn.")

    raise PortError(
        "no STM32 USB link found (looking for 0483:5740). Check the cable is in the "
        "board's USB USER port, not the ST-LINK port, and that the Appli is running. "
        f"Serial devices present: {seen}")
