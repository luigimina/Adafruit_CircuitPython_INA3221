# SPDX-FileCopyrightText: Copyright (c) 2024 Liz Clark for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""
`adafruit_ina3221`
================================================================================
CircuitPython driver for the INA3221 Triple 0-26 VDC, ±3.2 Amp Power Monitor

* Author(s): Liz Clark
Implementation Notes
--------------------
**Hardware:**
.. todo:: Add links to any specific hardware product page(s), or category page(s).
  Use unordered list & hyperlink rST inline format: "* `Link Text <url>`_"
**Software and Dependencies:**
* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads
* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
"""

import time
from adafruit_bus_device.i2c_device import I2CDevice

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_INA3221.git"

# Define constants
DEFAULT_ADDRESS = 0x40

MANUFACTURER_ID = 0x5449
DIE_ID = 0x3220

# Register Definitions
CONFIGURATION = 0x00
SHUNTVOLTAGE_CH1 = 0x01
BUSVOLTAGE_CH1 = 0x02
SHUNTVOLTAGE_CH2 = 0x03
BUSVOLTAGE_CH2 = 0x04
SHUNTVOLTAGE_CH3 = 0x05
BUSVOLTAGE_CH3 = 0x06
CRITICAL_ALERT_LIMIT_CH1 = 0x07
WARNING_ALERT_LIMIT_CH1 = 0x08
CRITICAL_ALERT_LIMIT_CH2 = 0x09
WARNING_ALERT_LIMIT_CH2 = 0x0A
CRITICAL_ALERT_LIMIT_CH3 = 0x0B
WARNING_ALERT_LIMIT_CH3 = 0x0C
SHUNTVOLTAGE_SUM = 0x0D
SHUNTVOLTAGE_SUM_LIMIT = 0x0E
MASK_ENABLE = 0x0F
POWERVALID_UPPERLIMIT = 0x10
POWERVALID_LOWERLIMIT = 0x11
MANUFACTURER_ID_REG = 0xFE
DIE_ID_REG = 0xFF

# Mask/Enable Register bit flags
CONV_READY = (1 << 0)
TIMECONT_ALERT = (1 << 1)
POWER_VALID = (1 << 2)
WARN_CH3 = (1 << 3)
WARN_CH2 = (1 << 4)
WARN_CH1 = (1 << 5)
SUMMATION = (1 << 6)
CRITICAL_CH3 = (1 << 7)
CRITICAL_CH2 = (1 << 8)
CRITICAL_CH1 = (1 << 9)

# Enumerations for averaging, conversion time, and mode
class AVG_MODE:
    AVG_1_SAMPLE = 0b000
    AVG_4_SAMPLES = 0b001
    AVG_16_SAMPLES = 0b010
    AVG_64_SAMPLES = 0b011
    AVG_128_SAMPLES = 0b100
    AVG_256_SAMPLES = 0b101
    AVG_512_SAMPLES = 0b110
    AVG_1024_SAMPLES = 0b111

class CONV_TIME:
    CONV_TIME_140US = 0b000
    CONV_TIME_204US = 0b001
    CONV_TIME_332US = 0b010
    CONV_TIME_588US = 0b011
    CONV_TIME_1MS = 0b100
    CONV_TIME_2MS = 0b101
    CONV_TIME_4MS = 0b110
    CONV_TIME_8MS = 0b111

class MODE:
    POWER_DOWN = 0b000
    SHUNT_TRIG = 0b001
    BUS_TRIG = 0b010
    SHUNT_BUS_TRIG = 0b011
    POWER_DOWN2 = 0b100
    SHUNT_CONT = 0b101
    BUS_CONT = 0b110
    SHUNT_BUS_CONT = 0b111

class INA3221Channel:
    """Represents a single channel of the INA3221."""

    def __init__(self, parent, channel):
        self._parent = parent
        self._channel = channel

    @property
    def bus_voltage(self):
        """Returns the bus voltage for this channel."""
        return self._parent._bus_voltage(self._channel)

    @property
    def shunt_voltage(self):
        """Returns the shunt voltage for this channel."""
        return self._parent._shunt_voltage(self._channel)

    @property
    def shunt_resistance(self):
        """Get or set the shunt resistance for this channel."""
        return self._parent._shunt_resistance[self._channel]

    @shunt_resistance.setter
    def shunt_resistance(self, value):
        self._parent._shunt_resistance[self._channel] = value

    @property
    def current_amps(self):
        """Returns the current in amperes for this channel."""
        shunt_voltage = self.shunt_voltage
        if shunt_voltage != shunt_voltage:  # Check for NaN
            return float('nan')
        return shunt_voltage / self.shunt_resistance

class INA3221:
    """Represents the INA3221 device with three channels."""

    def __init__(self, i2c, address=DEFAULT_ADDRESS):
        self.i2c_dev = I2CDevice(i2c, address)
        self._shunt_resistance = [0.05, 0.05, 0.05]  # Default shunt resistances
        self.reset()
        
        self.channels = [INA3221Channel(self, i) for i in range(3)]
        for i in range(3):
            self.enable_channel(i)
        self.mode = MODE.SHUNT_BUS_CONT
        print(f"mode is {self.mode}")
        self.shunt_voltage_conv_time = CONV_TIME.CONV_TIME_8MS
        self.bus_voltage_conv_time = CONV_TIME.CONV_TIME_8MS
        # Set the default sampling rate (averaging mode) to 16 samples
        self.averaging_mode = AVG_MODE.AVG_64_SAMPLES

    def __getitem__(self, channel):
        """Allows access to channels via index, e.g., ina[0].bus_voltage"""
        if channel < 0 or channel >= 3:
            raise IndexError("Channel must be 0, 1, or 2.")
        return self.channels[channel]

    @property
    def die_id(self):
        """Returns the Die ID of the INA3221."""
        return int.from_bytes(self._read_register(DIE_ID_REG, 2), 'big')

    @property
    def manufacturer_id(self):
        """Returns the Manufacturer ID of the INA3221."""
        return int.from_bytes(self._read_register(MANUFACTURER_ID_REG, 2), 'big')
    
    @property
    def mode(self):
        """Get or set the operating mode of the INA3221."""
        config = self._read_register(CONFIGURATION, 2)
        return config[1] & 0x07

    @mode.setter
    def mode(self, value):
        config = self._read_register(CONFIGURATION, 2)
        config = bytearray(config)
        config[1] = (config[1] & 0xF8) | value
        self._write_register(CONFIGURATION, config)

    @property
    def shunt_voltage_conv_time(self):
        """Get or set the shunt voltage conversion time."""
        config = self._read_register(CONFIGURATION, 2)
        return (config[1] >> 4) & 0x07

    @shunt_voltage_conv_time.setter
    def shunt_voltage_conv_time(self, conv_time):
        config = self._read_register(CONFIGURATION, 2)
        config = bytearray(config)
        config[1] = (config[1] & 0x8F) | (conv_time << 4)
        self._write_register(CONFIGURATION, config)

    @property
    def bus_voltage_conv_time(self):
        """Get or set the bus voltage conversion time."""
        config = self._read_register(CONFIGURATION, 2)
        return (config[0] >> 3) & 0x07  # Bits 12-14 are the bus voltage conversion time

    @bus_voltage_conv_time.setter
    def bus_voltage_conv_time(self, conv_time):
        if conv_time < 0 or conv_time > 7:
            raise ValueError("Conversion time must be between 0 and 7")

        config = self._read_register(CONFIGURATION, 2)
        config = bytearray(config)
        
        # Clear the old conversion time (bits 12-14)
        config[0] = config[0] & 0xC7  # Clear bits 12-14 (0b11000111)
        
        # Set the new conversion time
        config[0] = config[0] | (conv_time << 3)
        
        self._write_register(CONFIGURATION, config)

    @property
    def averaging_mode(self):
        """Get or set the averaging mode."""
        config = self._read_register(CONFIGURATION, 2)
        return (config[1] >> 1) & 0x07

    @averaging_mode.setter
    def averaging_mode(self, mode):
        config = self._read_register(CONFIGURATION, 2)
        config = bytearray(config)
        config[1] = (config[1] & 0xF1) | (mode << 1)
        self._write_register(CONFIGURATION, config)

    def _to_signed(self, val, bits):
        """Convert an unsigned integer to signed."""
        if val & (1 << (bits - 1)):
            val -= 1 << bits
        return val

    def _shunt_voltage(self, channel):
        """Internal method to get the shunt voltage for a given channel."""
        if channel > 2:
            return float('nan')  # Invalid channel
        reg_address = [SHUNTVOLTAGE_CH1, SHUNTVOLTAGE_CH2, SHUNTVOLTAGE_CH3][channel]
        result = self._read_register(reg_address, 2)
        # Manually convert to a signed integer
        raw_value = int.from_bytes(result, 'big')
        raw_value = self._to_signed(raw_value, 16)  # 16-bit signed value

        return (raw_value >> 3) * 40e-6  # Scale factor for shunt voltage

    def _bus_voltage(self, channel):
        """Internal method to get the bus voltage for a given channel."""
        if channel > 2:
            return float('nan')  # Invalid channel

        reg_address = [BUSVOLTAGE_CH1, BUSVOLTAGE_CH2, BUSVOLTAGE_CH3][channel]
        result = self._read_register(reg_address, 2)

        # Convert the result into an unsigned 16-bit integer
        raw_value = int.from_bytes(result, 'big')

        # Drop the bottom 3 bits and scale the value
        voltage = (raw_value >> 3) * 8e-3  # Scale factor for bus voltage

        return voltage

    def _current_amps(self, channel):
        """Internal method to get the current in amperes for a specific channel."""
        if channel >= 3:
            return float('nan')  # Invalid channel

        shunt_voltage = self._shunt_voltage(channel)
        if shunt_voltage != shunt_voltage:  # Check for NaN
            return float('nan')

        return shunt_voltage / self._shunt_resistance[channel]

    def reset(self):
        """Perform a soft reset."""
        config = self._read_register(CONFIGURATION, 2)
        config = bytearray(config)
        config[0] |= 0x80  # Set the reset bit
        return self._write_register(CONFIGURATION, config)

    def _write_register(self, reg, data):
        """Write to a specific register."""
        with self.i2c_dev:
            self.i2c_dev.write(bytes([reg]) + data)

    def _read_register(self, reg, length):
        """Read from a specific register."""
        result = bytearray(length)
        try:
            with self.i2c_dev:
                self.i2c_dev.write(bytes([reg]))
                self.i2c_dev.readinto(result)
        except OSError as e:
            print(f"I2C error: {e}")
            return None
        return result

    def enable_channel(self, channel):
        if channel > 2:
            raise ValueError("Invalid channel number. Must be 0, 1, or 2.")

        config = self._read_register(CONFIGURATION, 2)
        config_value = (config[0] << 8) | config[1]
        config_value |= (1 << (14 - channel))  # Set the bit for the specific channel
        high_byte = (config_value >> 8) & 0xFF
        low_byte = config_value & 0xFF
        self._write_register(CONFIGURATION, bytes([high_byte, low_byte]))

        # Confirm the change
        config_after = self._read_register(CONFIGURATION, 2)
        print(f"Channel {channel} enabled, configuration register: {config_after}")