"""as5048a.py -- Driver for the AMS/OSRAM AS5048A magnetic rotary encoder.

14-bit absolute position over SPI (mode 1, up to 10 MHz).  Hardware
wiring is described by an ``AS5048AConfig`` dataclass; no constants in
control code.  Interface mirrors the LeRobot bus contract
(``connect``/``disconnect``/``is_connected``/context-manager/``read*``).

References
----------
* Datasheet "AS5048A/AS5048B Magnetic Rotary Encoder" v1-11, 2018-Jan-29
  - SPI command/response, register map, parity rules.
* TSSOP14 pinout: CSn=1, CLK=2, MISO=3, MOSI=4.

Register quick-reference (SPI, 14-bit address space)
----------------------------------------------------
    0x0000  NOP                 (read; returns 0)
    0x0001  Clear error flag    (read; returns parity/cmd/frame error)
    0x0003  Programming control (R/W -- only needed to burn OTP zero)
    0x0016  OTP zero pos hi     (R/W -- bits 13..6 of zero offset)
    0x0017  OTP zero pos lo     (R/W -- bits 5..0  of zero offset)
    0x3FFD  Diagnostics + AGC   (read)
    0x3FFE  Magnitude (CORDIC)  (read)
    0x3FFF  Angle (zero-corrected) (read)

SPI frame layout (16-bit, MSB first)
------------------------------------
    Command:   [PAR | RWn | addr<13:0>]
    Response:  [PAR | EF  | data<13:0>]    EF=1 -> error in *previous* host frame
    Data:      [PAR | R=0 | data<13:0>]    used for writes (R must be 0)

Parity is **even** computed over bits 0..14 (the parity bit itself is bit 15).
"""
from __future__ import annotations

import time

try:
    import spidev   # noqa: F401 -- imported lazily so import works on dev hosts
except ImportError:
    spidev = None   # type: ignore[assignment]

from encoder_config import AS5048AConfig


# --- Register addresses ---------------------------------------------------
REG_NOP             = 0x0000
REG_CLEAR_ERROR     = 0x0001
REG_PROG_CONTROL    = 0x0003
REG_OTP_ZERO_HI     = 0x0016
REG_OTP_ZERO_LO     = 0x0017
REG_DIAG_AGC        = 0x3FFD
REG_MAGNITUDE       = 0x3FFE
REG_ANGLE           = 0x3FFF

# --- Frame bit masks ------------------------------------------------------
_DATA_MASK   = 0x3FFF   # bits 13..0
_RW_READ     = 0x4000   # bit 14
_PARITY_BIT  = 0x8000   # bit 15
_ERR_FLAG    = 0x4000   # bit 14 in response = EF

# --- Resolution constants -------------------------------------------------
_FULL_SCALE  = 1 << 14            # 16384 counts / revolution
_LSB_DEG     = 360.0 / _FULL_SCALE        # 0.021972°
_LSB_RAD     = 6.283185307179586 / _FULL_SCALE


def _even_parity(word14: int) -> int:
    """Return 1 if the 15 low bits of ``word14`` contain an odd number of 1s.

    The datasheet specifies EVEN parity over the command/data field --
    i.e. the parity bit is chosen so that the total number of 1s in the
    full 16-bit frame is even.  Equivalently, parity = XOR of the 15
    payload bits.  Used only for building command frames (bits 0..14).
    """
    v = word14 & 0x7FFF
    v ^= v >> 8
    v ^= v >> 4
    v ^= v >> 2
    v ^= v >> 1
    return v & 1


def _parity_ok(word16: int) -> bool:
    """Return True if the full 16-bit word has even parity (valid frame).

    For a response frame to be valid per the datasheet, the total number
    of 1-bits across all 16 bits (including PAR) must be even.
    """
    v = word16 & 0xFFFF
    v ^= v >> 8
    v ^= v >> 4
    v ^= v >> 2
    v ^= v >> 1
    return (v & 1) == 0


class AS5048A:
    """LeRobot-style driver for one AS5048A on an SPI bus.

    All angle methods return values from the firmware **angle register
    (0x3FFF)** which already includes the OTP zero-position correction.

    Usage
    -----
        >>> from encoder_config import AS5048AConfig
        >>> from as5048a import AS5048A
        >>> with AS5048A(AS5048AConfig()) as enc:
        ...     print(enc.read_angle_deg())

    Thread safety
    -------------
    Not thread-safe.  One control loop per encoder.
    """

    def __init__(self, config: AS5048AConfig | None = None) -> None:
        self.config = config or AS5048AConfig()
        self._spi: "spidev.SpiDev | None" = None

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def connect(self) -> None:
        """Open /dev/spidev<bus>.<device> and configure clock/mode."""
        if spidev is None:
            raise RuntimeError(
                "spidev not installed.  On Jetson run: pip install spidev"
            )
        self._spi = spidev.SpiDev()
        self._spi.open(self.config.bus, self.config.device)
        self._spi.max_speed_hz = self.config.max_hz
        self._spi.mode = self.config.mode        # AS5048A is SPI mode 1
        self._spi.bits_per_word = 8
        # Clear any pending error from power-up.
        try:
            self.clear_error()
        except Exception:
            pass

    def disconnect(self) -> None:
        """Close the SPI handle."""
        if self._spi is not None:
            try:
                self._spi.close()
            except Exception:
                pass
            self._spi = None

    @property
    def is_connected(self) -> bool:
        return self._spi is not None

    def __enter__(self) -> "AS5048A":
        if not self.is_connected:
            self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.disconnect()

    # ------------------------------------------------------------------
    # Low-level SPI primitives
    # ------------------------------------------------------------------

    def _xfer16(self, word: int) -> int:
        """Transmit one 16-bit frame MSB-first and return the 16-bit reply."""
        if self._spi is None:
            raise RuntimeError("AS5048A.connect() not called")
        hi = (word >> 8) & 0xFF
        lo = word & 0xFF
        rx = self._spi.xfer2([hi, lo])
        return (rx[0] << 8) | rx[1]

    @staticmethod
    def _build_read_cmd(addr: int) -> int:
        """Build a READ command frame with even parity."""
        cmd = _RW_READ | (addr & _DATA_MASK)
        if _even_parity(cmd):
            cmd |= _PARITY_BIT
        return cmd

    @staticmethod
    def _build_write_cmd(addr: int) -> int:
        """Build a WRITE command frame (RWn=0) with even parity."""
        cmd = (addr & _DATA_MASK)   # RWn = 0
        if _even_parity(cmd):
            cmd |= _PARITY_BIT
        return cmd

    @staticmethod
    def _build_data(data: int) -> int:
        """Build a WRITE data frame (R=0) with even parity."""
        d = data & _DATA_MASK
        if _even_parity(d):
            d |= _PARITY_BIT
        return d

    # ------------------------------------------------------------------
    # Register access
    # ------------------------------------------------------------------

    def read_register(self, addr: int, *, check_parity: bool = True) -> int:
        """Read one 14-bit register.

        Two SPI frames are required: the first carries the READ command,
        the response in the *next* frame contains the data (see datasheet
        Figure 23).  We send a NOP as the trailing frame.

        If the error flag (EF) is set -- which can happen transiently due
        to SPI noise on jumper wires -- the error register is cleared and
        the read is retried once before raising.

        Raises:
            RuntimeError if parity mismatches or EF persists after retry.
        """
        for attempt in range(2):
            self._xfer16(self._build_read_cmd(addr))
            resp = self._xfer16(self._build_read_cmd(REG_NOP))   # NOP

            if check_parity and not _parity_ok(resp):
                raise RuntimeError(f"AS5048A parity error on register 0x{addr:04X}")
            if resp & _ERR_FLAG:
                # EF is a sticky flag; clear it and retry once.  On jumper-wire
                # setups SPI noise occasionally triggers a spurious parity error
                # in the chip, which sets EF without corrupting the data.
                err = self.clear_error()
                if attempt == 0:
                    continue   # retry the read
                raise RuntimeError(
                    f"AS5048A error flag set when reading 0x{addr:04X}; "
                    f"error register = 0x{err:04X} "
                    f"(parity={bool(err & 0b100)}, cmd_invalid={bool(err & 0b010)}, "
                    f"frame={bool(err & 0b001)})"
                )
            return resp & _DATA_MASK
        return resp & _DATA_MASK   # unreachable, satisfies type checker

    def write_register(self, addr: int, value: int) -> int:
        """Write a 14-bit value to a register and return the *old* contents."""
        self._xfer16(self._build_write_cmd(addr))
        old = self._xfer16(self._build_data(value))
        # A NOP flushes the verification frame so the next read sees the new value.
        self._xfer16(self._build_read_cmd(REG_NOP))
        return old & _DATA_MASK

    def clear_error(self) -> int:
        """Issue the CLEAR ERROR FLAG command; return the latched error bits."""
        self._xfer16(self._build_read_cmd(REG_CLEAR_ERROR))
        resp = self._xfer16(self._build_read_cmd(REG_NOP))
        return resp & _DATA_MASK

    # ------------------------------------------------------------------
    # High-level angle/diagnostics API
    # ------------------------------------------------------------------

    def read_angle_raw(self) -> int:
        """Return the raw 14-bit angle (0..16383), zero-position corrected."""
        return self.read_register(REG_ANGLE)

    def read_angle_deg(self) -> float:
        """Return the angle in degrees, [0.0, 360.0)."""
        return self.read_angle_raw() * _LSB_DEG

    def read_angle_rad(self) -> float:
        """Return the angle in radians, [0.0, 2*pi)."""
        return self.read_angle_raw() * _LSB_RAD

    def read_magnitude(self) -> int:
        """Return the CORDIC magnitude (proxy for magnet field strength)."""
        return self.read_register(REG_MAGNITUDE)

    def read_diagnostics(self) -> dict:
        """Return diagnostic flags + AGC value from register 0x3FFD.

        Returns dict with:
            comp_high : weak magnetic field (raise magnet closer)
            comp_low  : strong magnetic field (move magnet away)
            cof       : CORDIC overflow (angle invalid)
            ocf       : offset compensation finished (1 after power-up)
            agc       : 0..255, automatic gain control (low = strong field)
        """
        raw = self.read_register(REG_DIAG_AGC)
        return {
            "comp_high": bool(raw & (1 << 11)),
            "comp_low":  bool(raw & (1 << 10)),
            "cof":       bool(raw & (1 << 9)),
            "ocf":       bool(raw & (1 << 8)),
            "agc":       raw & 0xFF,
        }

    # ------------------------------------------------------------------
    # Zero-position helpers
    # ------------------------------------------------------------------

    def set_zero(self, *, burn_otp: bool = False) -> int:
        """Latch the current shaft angle as the new zero offset.

        Sequence per datasheet "Programming Sequence with Verification":
          1. Write 0 to 0x0016 / 0x0017 (clear current offset).
          2. Read raw angle.
          3. Write that angle back to 0x0016 / 0x0017.

        With ``burn_otp=False`` (default) the zero is only set in RAM and
        is lost on power cycle.  ``burn_otp=True`` permanently burns the
        OTP fuses -- THIS IS IRREVERSIBLE and can only be done once.

        Returns the raw 14-bit angle that was latched as zero.
        """
        # 1. Clear the current zero offset so we read the absolute mechanical angle.
        self.write_register(REG_OTP_ZERO_HI, 0)
        self.write_register(REG_OTP_ZERO_LO, 0)
        time.sleep(0.005)

        # 2. Read current absolute angle.
        angle = self.read_angle_raw()

        # 3. Write it back.  Bits 13..6 in HI register, bits 5..0 in LO register.
        self.write_register(REG_OTP_ZERO_HI, (angle >> 6) & 0xFF)
        self.write_register(REG_OTP_ZERO_LO, angle & 0x3F)
        time.sleep(0.005)

        if burn_otp:
            # Programming sequence per datasheet section "Application Information".
            self.write_register(REG_PROG_CONTROL, 0x01)   # Programming Enable
            self.write_register(REG_PROG_CONTROL, 0x08)   # Burn
            time.sleep(0.020)
            _ = self.read_angle_raw()                     # should now read 0
            self.write_register(REG_PROG_CONTROL, 0x40)   # Verify (re-load OTP)
            time.sleep(0.005)
        return angle
