"""encoder_config.py -- Declarative configuration for AS5048A encoders.

Same LeRobot-style dataclass pattern used by motor_config.py:
hardware wiring (SPI bus, CS line, clock speed) is captured in a plain
dataclass that can be constructed inline, loaded from YAML/JSON, or
passed between modules without touching driver code.

Wiring (AS5048A → Jetson Orin Nano 40-pin header)
-------------------------------------------------
    Encoder pin   Jetson pin   Jetson signal
    GND           25           GND
    3V3           17           3.3V
    MOSI          19           SPI MOSI  (spi1_mosi_pz5)
    MISO          21           SPI MISO  (spi1_miso_pz4)
    SCK           23           SPI SCK   (spi1_sck_pz3)
    CSN           24           SPI CS0   (spi1_cs0_pz6)

Linux bus mapping on this hardware
----------------------------------
Empirical loopback test (``tests/encoder/00_spi_connectivity.py``)
on a JP6 Orin Nano Devkit shows that the 40-pin header pins
19/21/23/24 are exposed as **``/dev/spidev0.0``** -- i.e. Linux
``spi0`` = Tegra ``spi@3210000``.  The other controller
``spi@3230000`` enumerates as ``/dev/spidev1.0`` but is NOT wired to
the header on this board, despite the Tegra pad names being
``spi1_*_pz*``.

This corresponds to ``AS5048AConfig(bus=0, device=0)``.

If you swap to a board where the header is wired to ``spi@3230000``
(common on AGX-class carriers), change ``bus=1``.  The loopback test
in stage 1 of ``00_spi_connectivity.py`` will tell you which bus the
header maps to.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class AS5048AConfig:
    """Configuration for one AS5048A encoder on an SPI bus.

    Attributes
    ----------
    bus       : SPI bus number.  /dev/spidevN.M -> N=bus.  On the Orin
                Nano Devkit (P3768) the 40-pin header maps to
                ``/dev/spidev0.0`` -> bus=0.
    device    : Chip-select index on that bus.  CS0 -> 0, CS1 -> 1.
                With the wiring above this is 0.
    max_hz    : SPI clock frequency.  Datasheet allows up to 10 MHz
                (Tclk_min = 100 ns).  1 MHz is conservative and rock-solid
                over jumper wires; raise to 5–10 MHz on a clean PCB.
    mode      : SPI mode.  Datasheet: MOSI sampled on falling CLK edge,
                MISO updated on rising CLK edge -> SPI mode 1
                (CPOL=0, CPHA=1).
    """
    bus:    int = 0
    device: int = 0
    max_hz: int = 1_000_000
    mode:   int = 1


# Bench default: single encoder on the Orin Nano 40-pin header.
# Verified by loopback test: pins 19/21/23/24 -> /dev/spidev0.0.
DEFAULT_ENCODER_CONFIG = AS5048AConfig(bus=0, device=0, max_hz=1_000_000, mode=1)
