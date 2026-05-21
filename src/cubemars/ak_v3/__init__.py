"""CubeMars AK-series V3.0 (extended 29-bit CAN) driver package."""
import sys
import os as _os

_src = _os.path.dirname(_os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
if _src not in sys.path:
    sys.path.insert(0, _src)
