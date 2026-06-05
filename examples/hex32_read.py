#!/usr/bin/env python3
"""Minimal streaming example: read the Resense HEX32 over USB and print frames.

    python examples/hex32_read.py /dev/ttyACM0        # Linux
    python examples/hex32_read.py COM7                 # Windows

For more options (trigger mode, CSV, rate report) use the installed CLI:
    hex32-read --quiet
"""

import sys

from resensehex import FrameCorruption, FrameTimeout, ResenseHEX


def main() -> None:
    device = sys.argv[1] if len(sys.argv) > 1 else (
        "COM3" if sys.platform.startswith("win") else "/dev/ttyACM0"
    )

    with ResenseHEX(device) as hex_sensor:
        print(f"Opened {device}. Aligning...")
        hex_sensor.align()
        try:
            while True:
                try:
                    f = hex_sensor.read_frame_and_timestamp()
                except (FrameTimeout, FrameCorruption) as exc:
                    print(f"skip: {exc}", file=sys.stderr)
                    continue
                print(f"Fx={f.fx:8.3f} Fy={f.fy:8.3f} Fz={f.fz:8.3f}  "
                      f"Mx={f.mx:8.3f} My={f.my:8.3f} Mz={f.mz:8.3f}  T={f.temperature:.2f}C")
        except KeyboardInterrupt:
            print("\nStopped.")


if __name__ == "__main__":
    main()
