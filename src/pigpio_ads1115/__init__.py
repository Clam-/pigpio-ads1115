import struct
from typing import Optional
import pigpio
import time

class ADS1115:
    # Pointer registers
    PTR_CONVERSION = 0x00
    PTR_CONFIG     = 0x01

    # Config masks
    OS_SINGLE       = 0x8000  # start single‐shot conversion
    MODE_SINGLE     = 0x0100
    MODE_CONTINUOUS = 0x0000

    # Channel / PGA / data rate / comparator
    MUX_AIN0    = 0x4000       # AIN0 vs GND
    PGA_4_096V  = 0x0200       # ±4.096V full scale
    DR_128SPS   = 0x0080       # 128 samples/sec
    COMP_DISABLE= 0x0003       # comparator off

    def __init__(
        self,
        pi: Optional[pigpio.pi] = None,
        bus: int = 0,
        address: int = 0x48,
        mode: str = "single",        # "single" or "continuous"
        data_rate: int = 128         # only 128 SPS supported in this module
    ):
        if not pi: pi = pigpio.pi()
        self.pi      = pi
        self.mode    = mode.lower()
        self.rate    = data_rate
        self.handle  = pi.i2c_open(bus, address)
        if self.handle < 0:
            raise RuntimeError(f"Cannot open I2C bus{bus}@0x{address:02X}")

        # Pre-compute the bits that never change
        self._base_cfg = (
            self.MUX_AIN0 |
            self.PGA_4_096V |
            self.COMP_DISABLE
        )

        if self.mode not in ("single", "continuous"):
            raise ValueError("mode must be 'single' or 'continuous'")

        if self.mode == "continuous":
            self._start_continuous()

    def _start_continuous(self):
        """Write config to launch continuous conversions."""
        cfg = (
            self._base_cfg |
            self.MODE_CONTINUOUS |
            self.DR_128SPS
        )
        # PTR_CONFIG + [MSB, LSB]
        self.pi.i2c_write_device(
            self.handle,
            [self.PTR_CONFIG, (cfg >> 8) & 0xFF, cfg & 0xFF]
        )
        # Wait one conversion period
        time.sleep(1 / self.rate + 0.001)

    def read(self) -> int:
        """
        Trigger or fetch one ADC conversion and return the signed 16‐bit raw value.
        """
        if self.mode == "single":
            return self._read_single()
        else:
            return self._read_continuous()

    @property
    def value(self) -> int:
        """
        Alias for read(): returns the signed 16‐bit ADC result.
        """
        return self.read()
    
    def _read_single(self) -> int:
        """Trigger one-shot conversion and read result."""
        cfg = (
            self._base_cfg |
            self.OS_SINGLE |
            self.MODE_SINGLE |
            self.DR_128SPS
        )
        self.pi.i2c_write_device(
            self.handle,
            [self.PTR_CONFIG, (cfg >> 8) & 0xFF, cfg & 0xFF ]
        )
        time.sleep(1 / self.rate + 0.001)
        self.pi.i2c_write_byte(self.handle, self.PTR_CONVERSION)
        return self._read_conversion()

    def _read_continuous(self) -> int:
        """Fetch the latest conversion in continuous mode."""
        # Point at the conversion register
        self.pi.i2c_write_byte(self.handle, self.PTR_CONVERSION)
        return self._read_conversion()

    def _read_conversion(self) -> int:
        count, data = self.pi.i2c_read_device(self.handle, 2)
        if count != 2:
            raise RuntimeError(f"Conversion read failed ({count} bytes)")

        # normalize to bytes
        if isinstance(data, str):
            buf = data.encode("latin1")
        else:
            buf = bytes(data)   # list[int] → bytes

        # unpack signed 16bit BE
        raw, = struct.unpack(">h", buf)
        return raw

    def raw_to_voltage(self, raw: int) -> float:
        """
        Convert the signed raw ADC reading to volts
        (LSB = 125µV at ±4.096V).
        """
        return raw * 0.000125

    def close(self):
        """Close the I2C handle."""
        self.pi.i2c_close(self.handle)
