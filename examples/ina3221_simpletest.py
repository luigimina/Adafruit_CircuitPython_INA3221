# SPDX-FileCopyrightText: Copyright (c) 2024 Liz Clark for Adafruit Industries
#
# SPDX-License-Identifier: MIT
import time
import board
import busio
from adafruit_ina3221 import INA3221

# Set up I2C and INA3221
i2c = busio.I2C(board.SCL, board.SDA)
ina = INA3221(i2c)

while True:
    # Loop through the 3 channels and read bus voltage and current
    for i in range(3):
        bus_voltage = ina[i].bus_voltage
        shunt_voltage = ina[i].shunt_voltage
        current = ina[i].current_amps * 1000

        print(f"Channel {i + 1}:")
        print(f"  Bus Voltage: {bus_voltage:.6f} V")
        print(f"  Shunt Voltage: {shunt_voltage:.6f} V")
        print(f"  Current: {current:.6f} mA")
        print("-" * 30)

    # Wait 2 seconds before reading again
    time.sleep(2)
