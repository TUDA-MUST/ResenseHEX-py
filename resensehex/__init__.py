"""ResenseHEX-py: read a Resense HEX32 6-axis F/T sensor over USB.

Cross-platform (Linux / macOS / Windows) Python library that mirrors the
C++/Arduino ResenseHEX API.
"""

from .core import (
    DEFAULT_BAUD,
    FRAME_SIZE,
    FrameCorruption,
    FrameTimeout,
    HexFrame,
    ResenseHEX,
    ResenseHEXError,
)

__version__ = "1.0.0"

__all__ = [
    "ResenseHEX",
    "HexFrame",
    "ResenseHEXError",
    "FrameTimeout",
    "FrameCorruption",
    "DEFAULT_BAUD",
    "FRAME_SIZE",
    "__version__",
]
