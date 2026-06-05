"""Core implementation of the Resense HEX32 USB interface.

The Resense evaluation box enumerates as a USB CDC-ACM serial device, so this
library opens it with pyserial (raw, 8-N-1, 2 Mbaud) and reads the 28-byte
binary frames the sensor emits. The API mirrors the C++/Arduino libraries.

Protocol (identical to the UART case, carried over USB)::

    Baud 2,000,000  8-N-1
    Frame = 28 bytes = 7x float32 little-endian:
      [0:4] fx  [4:8] fy  [8:12] fz  [12:16] mx  [16:20] my  [20:24] mz  [24:28] temperature
    Software trigger : b"SAMPLE\\r\\n"   (single measurement)
    Tare             : b"TARA\\r\\n"
"""

from __future__ import annotations

import math
import struct
import time
from dataclasses import dataclass
from typing import Optional, Union

import serial  # pyserial

DEFAULT_BAUD = 2_000_000
FRAME_SIZE = 28
_FRAME = struct.Struct("<7f")  # 7 little-endian float32
_SOFTWARE_TRIGGER_CMD = b"SAMPLE\r\n"
_TARE_CMD = b"TARA\r\n"
_MIN_TARE_READS = 1000


class ResenseHEXError(Exception):
    """Base class for all library errors."""


class FrameTimeout(ResenseHEXError, TimeoutError):
    """A full frame did not arrive within the read timeout."""


class FrameCorruption(ResenseHEXError):
    """A frame decoded to non-finite (NaN/Inf) values."""


@dataclass
class HexFrame:
    """One complete measurement frame.

    Forces in newtons, torques in millinewton-metres, temperature in degrees C.
    ``timestamp`` is a host monotonic time in seconds (``time.monotonic()``),
    filled by :meth:`ResenseHEX.read_frame_and_timestamp` and
    :meth:`ResenseHEX.trigger_and_read`.
    """

    fx: float = 0.0
    fy: float = 0.0
    fz: float = 0.0
    mx: float = 0.0
    my: float = 0.0
    mz: float = 0.0
    temperature: float = 0.0
    timestamp: float = 0.0


class ResenseHEX:
    """USB (CDC-ACM) interface to a Resense HEX 6-axis F/T sensor.

    Usable as a context manager::

        with ResenseHEX("/dev/ttyACM0") as hex:
            hex.align()
            while True:
                print(hex.read_frame_and_timestamp())
    """

    def __init__(
        self,
        device: str,
        baud: int = DEFAULT_BAUD,
        *,
        read_timeout: float = 0.3,
        tare_timeout: float = 20.0,
    ) -> None:
        self.device = device
        self.baud = baud
        self.read_timeout = read_timeout
        self.tare_timeout = tare_timeout

        # Validation limits (match the Arduino/C++ defaults).
        self.max_force = 5000.0     # N
        self.max_torque = 10.0      # mNm
        self.max_temperature = 150.0  # deg C

        self._serial: Optional[serial.Serial] = None

    # -- lifecycle -----------------------------------------------------------

    def open(self) -> None:
        """Open and configure the serial device (raw, 8-N-1, requested baud)."""
        if self.is_open:
            return
        self._serial = serial.Serial(
            port=self.device,
            baudrate=self.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.read_timeout,
            write_timeout=1.0,
        )

    def close(self) -> None:
        """Close the device (safe to call repeatedly)."""
        if self._serial is not None:
            self._serial.close()
            self._serial = None

    @property
    def is_open(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def __enter__(self) -> "ResenseHEX":
        self.open()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # -- continuous mode -----------------------------------------------------

    def read_frame(self) -> HexFrame:
        """Read and decode one 28-byte frame (continuous-stream mode).

        Frames carry no header byte; call :meth:`align` once after :meth:`open`
        for streaming use. Raises :class:`FrameTimeout` or
        :class:`FrameCorruption`.
        """
        return self._decode(self._read_exact(FRAME_SIZE))

    def read_frame_and_timestamp(self) -> HexFrame:
        """Like :meth:`read_frame` but stamps ``timestamp`` (host seconds)."""
        raw = self._read_exact(FRAME_SIZE)
        ts = time.monotonic()
        frame = self._decode(raw)
        frame.timestamp = ts
        return frame

    def align(self, *, check_frames: int = 4) -> bool:
        """Find the 28-byte frame boundary in a continuous stream.

        Reads a window of bytes and, for each of the 28 offsets, checks whether
        several consecutive frames decode to finite values with a stable, sane
        temperature. Discards bytes so the next :meth:`read_frame` starts on a
        boundary. Returns ``True`` if a stable boundary was found.
        """
        window_frames = check_frames + 2
        window = self._read_exact(FRAME_SIZE * window_frames, timeout=self.read_timeout * 4)

        for off in range(FRAME_SIZE):
            tmin, tmax = math.inf, -math.inf
            ok = True
            for k in range(check_frames):
                start = off + k * FRAME_SIZE
                vals = _FRAME.unpack(window[start:start + FRAME_SIZE])
                if any(math.isnan(v) or math.isinf(v) for v in vals):
                    ok = False
                    break
                temp = vals[6]
                if temp < -40.0 or temp > 150.0:
                    ok = False
                    break
                tmin, tmax = min(tmin, temp), max(tmax, temp)
            if not ok or (tmax - tmin) > 5.0:
                continue

            consumed = off + check_frames * FRAME_SIZE
            leftover = FRAME_SIZE * window_frames - consumed
            to_boundary = (FRAME_SIZE - (leftover % FRAME_SIZE)) % FRAME_SIZE
            if to_boundary:
                self._read_exact(to_boundary)
            return True
        return False

    # -- software-trigger mode ----------------------------------------------

    def software_trigger(self) -> None:
        """Send the software trigger command (``SAMPLE\\r\\n``)."""
        self._write(_SOFTWARE_TRIGGER_CMD)

    def trigger_and_read(self) -> HexFrame:
        """Flush input, trigger, stamp timestamp, read + validate one frame.

        Reliable single-shot path when the sensor is in software-trigger mode.
        """
        self.flush_input()
        self.software_trigger()
        ts = time.monotonic()
        frame = self._decode(self._read_exact(FRAME_SIZE))
        frame.timestamp = ts
        return frame

    # -- taring --------------------------------------------------------------

    def tare(self) -> None:
        """Send the tare command (``TARA\\r\\n``)."""
        self._write(_TARE_CMD)

    def tare_blocking(self) -> bool:
        """Tare and block until completed (>= 1000 frames or timeout)."""
        self.tare()
        deadline = time.monotonic() + self.tare_timeout
        reads = 0
        while reads < _MIN_TARE_READS:
            if time.monotonic() >= deadline:
                return False
            self.software_trigger()
            time.sleep(0.01)
            try:
                self._read_exact(FRAME_SIZE)
            except FrameTimeout:
                return False
            reads += 1
        return True

    def send_command(self, cmd: Union[str, bytes]) -> None:
        """Send an arbitrary command verbatim."""
        if isinstance(cmd, str):
            cmd = cmd.encode("ascii")
        self._write(cmd)

    def flush_input(self) -> None:
        """Discard all currently-pending input bytes."""
        self._require_open().reset_input_buffer()

    # -- validation ----------------------------------------------------------

    def validate_limits(self, frame: HexFrame) -> bool:
        """True if the frame is within the configured force/torque/temp limits."""
        return (
            abs(frame.fx) <= self.max_force
            and abs(frame.fy) <= self.max_force
            and abs(frame.fz) <= self.max_force
            and abs(frame.mx) <= self.max_torque
            and abs(frame.my) <= self.max_torque
            and abs(frame.mz) <= self.max_torque
            and abs(frame.temperature) <= self.max_temperature
        )

    # -- internals -----------------------------------------------------------

    def _require_open(self) -> serial.Serial:
        if not self.is_open:
            raise ResenseHEXError("device not open")
        assert self._serial is not None
        return self._serial

    def _read_exact(self, n: int, *, timeout: Optional[float] = None) -> bytes:
        ser = self._require_open()
        if timeout is None:
            timeout = self.read_timeout
        deadline = time.monotonic() + timeout
        buf = bytearray()
        while len(buf) < n:
            chunk = ser.read(n - len(buf))
            if chunk:
                buf.extend(chunk)
            elif time.monotonic() >= deadline:
                raise FrameTimeout(f"timeout reading {n} bytes (got {len(buf)})")
        return bytes(buf)

    def _write(self, data: bytes) -> None:
        ser = self._require_open()
        ser.write(data)
        ser.flush()

    @staticmethod
    def _decode(raw: bytes) -> HexFrame:
        fx, fy, fz, mx, my, mz, temp = _FRAME.unpack(raw)
        if any(math.isnan(v) or math.isinf(v) for v in (fx, fy, fz, mx, my, mz, temp)):
            raise FrameCorruption("frame decoded to NaN/Inf")
        return HexFrame(fx, fy, fz, mx, my, mz, temp)
