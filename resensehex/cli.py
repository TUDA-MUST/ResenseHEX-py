"""Command-line reader for the Resense HEX32 (installed as ``hex32-read``)."""

from __future__ import annotations

import argparse
import csv as _csv
import sys
import time

from .core import DEFAULT_BAUD, FrameCorruption, FrameTimeout, ResenseHEX


def _default_device() -> str:
    return "COM3" if sys.platform.startswith("win") else "/dev/ttyACM0"


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="hex32-read",
        description="Read 7-float frames from a Resense HEX32 over USB and print/log them.",
    )
    p.add_argument("device", nargs="?", default=_default_device(),
                   help="serial device (Linux /dev/ttyACM0, Windows COM7)")
    p.add_argument("--mode", choices=["continuous", "trigger"], default="continuous")
    p.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    p.add_argument("--rate", type=float, default=0.0,
                   help="trigger mode only: target Hz (continuous runs at the sensor's rate)")
    p.add_argument("--count", type=int, default=-1, help="number of frames (-1 = forever)")
    p.add_argument("--csv", help="append frames to this CSV file")
    p.add_argument("--tare", action="store_true", help="send TARA before reading")
    p.add_argument("--quiet", "-q", action="store_true",
                   help="suppress per-frame lines, print achieved Hz once per second")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    trigger = args.mode == "trigger"

    hex_sensor = ResenseHEX(args.device, args.baud)
    try:
        hex_sensor.open()
    except Exception as exc:  # noqa: BLE001 - surface any open failure cleanly
        print(f"Failed to open {args.device}: {exc}", file=sys.stderr)
        return 1

    print(f"Opened {args.device} @ {args.baud} baud, mode={args.mode}")

    if args.tare:
        print("Taring (this takes a moment)...")
        print("Tare OK." if hex_sensor.tare_blocking() else "Tare failed (continuing).")

    if not trigger:
        print("Aligning to frame boundary...")
        if not hex_sensor.align():
            print("Align warning: no stable boundary (continuing unaligned)", file=sys.stderr)

    csv_writer = None
    csv_file = None
    if args.csv:
        new = False
        try:
            new = not open(args.csv).read(1)
        except OSError:
            new = True
        csv_file = open(args.csv, "a", newline="")
        csv_writer = _csv.writer(csv_file)
        if new:
            csv_writer.writerow(["host_s", "Fx", "Fy", "Fz", "Mx", "My", "Mz", "Temp"])

    pace = trigger and args.rate > 0.0
    if not trigger and args.rate > 0.0:
        print("note: --rate ignored in continuous mode (rate is set by the sensor)", file=sys.stderr)
    period = (1.0 / args.rate) if pace else 0.0

    n = 0
    fails = 0
    next_t = time.monotonic()
    win_start = time.monotonic()
    win_frames = 0

    try:
        while args.count < 0 or n < args.count:
            try:
                f = hex_sensor.trigger_and_read() if trigger else hex_sensor.read_frame_and_timestamp()
            except (FrameTimeout, FrameCorruption) as exc:
                print(f"read failed: {exc}", file=sys.stderr)
                fails += 1
                if fails > 50:
                    print("too many failures, aborting", file=sys.stderr)
                    break
                continue
            fails = 0
            n += 1
            win_frames += 1

            if not args.quiet:
                print(f"[{f.timestamp:.3f}s] Fx={f.fx:8.3f} Fy={f.fy:8.3f} Fz={f.fz:8.3f}  "
                      f"Mx={f.mx:8.3f} My={f.my:8.3f} Mz={f.mz:8.3f}  T={f.temperature:5.2f}C")

            if csv_writer is not None:
                csv_writer.writerow([f"{f.timestamp:.6f}", f.fx, f.fy, f.fz,
                                     f.mx, f.my, f.mz, f.temperature])

            now = time.monotonic()
            if now - win_start >= 1.0:
                hz = win_frames / (now - win_start)
                print(f"[rate] {hz:.1f} Hz ({win_frames} frames in "
                      f"{1000 * (now - win_start):.0f} ms)", file=sys.stderr)
                win_start = now
                win_frames = 0

            if period > 0.0:
                next_t += period
                sleep = next_t - time.monotonic()
                if sleep > 0:
                    time.sleep(sleep)
    except KeyboardInterrupt:
        pass
    finally:
        if csv_file is not None:
            csv_file.close()
        hex_sensor.close()

    print(f"\nRead {n} frames. Closing.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
