#!/usr/bin/env python3
"""Diagnostic: does TARA (tare) zero the sensor over USB?

Reads baseline frames, sends TARA, waits, then reads again to see whether the
values collapse toward zero.

    python examples/tara_test.py /dev/ttyACM0
    python examples/tara_test.py COM7
"""

import sys
import time

from resensehex import ResenseHEX


def dump(hex_sensor: ResenseHEX, tag: str, n: int = 4) -> None:
    for _ in range(n):
        f = hex_sensor.read_frame()
        print(f"{tag:7} Fx={f.fx:8.3f} Fy={f.fy:8.3f} Fz={f.fz:8.3f}  "
              f"Mx={f.mx:8.3f} My={f.my:8.3f} Mz={f.mz:8.3f}  T={f.temperature:.2f}")


def main() -> None:
    device = sys.argv[1] if len(sys.argv) > 1 else (
        "COM3" if sys.platform.startswith("win") else "/dev/ttyACM0"
    )

    with ResenseHEX(device) as hex_sensor:
        hex_sensor.align()

        print("--- BEFORE tare ---")
        dump(hex_sensor, "before")

        print("--- sending TARA ---")
        hex_sensor.tare()
        print("TARA sent")

        # Drain ~1.5 s so the tare computation completes, then re-align.
        end = time.monotonic() + 1.5
        while time.monotonic() < end:
            try:
                hex_sensor.read_frame()
            except Exception:  # noqa: BLE001
                pass
        hex_sensor.align()

        print("--- AFTER tare ---")
        dump(hex_sensor, "after")


if __name__ == "__main__":
    main()
