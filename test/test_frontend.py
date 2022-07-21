#!/usr/bin/env python3

#
# This file is part of Thunderscope-LiteX project.
#
# Copyright (c) 2022 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import sys
import time
import argparse

from litex import RemoteClient

# Frontend Constants -------------------------------------------------------------------------------

FRONTEND_CONTROL_LDO_EN      = (1 <<  0)
FRONTEND_CONTROL_COUPLING    = (1 <<  8)
FRONTEND_CONTROL_ATTENUATION = (1 << 16)

FRONTEND_STATUS_LDO_PWR_GOOD = (1 <<  0)

FRONTEND_AC_COUPLING = 0
FRONTEND_DC_COUPLING = 1

FRONTEND_1X_ATTENUATION  = 0
FRONTEND_10X_ATTENUATION = 1

# Frontend SPI Driver ------------------------------------------------------------------------------

SPI_CONTROL_START  = (1 << 0)
SPI_CONTROL_LENGTH = (1 << 8)
SPI_STATUS_DONE    = (1 << 0)

SPI_CS_PGA0 = 0
SPI_CS_PGA1 = 0
SPI_CS_PGA2 = 0
SPI_CS_PGA3 = 0

class FrontendSPIDriver:
    def __init__(self, bus):
        self.bus = bus

    def write(self, cs, data):
        assert len(data) <= 3
        # Convert data to bytes (if not already).
        data = data if isinstance(data, (bytes, bytearray)) else bytes(data)
        # Set Chip Select.
        self.bus.regs.frontend_spi_cs.write((1 << cs))
        # Prepare MOSI data.
        mosi_bits = len(data)*8
        mosi_data = int.from_bytes(data, byteorder="big")
        mosi_data <<= (24 - mosi_bits)
        self.bus.regs.frontend_spi_mosi.write(mosi_data)
        # Start SPI Xfer.
        self.bus.regs.frontend_spi_control.write(mosi_bits*SPI_CONTROL_LENGTH | SPI_CONTROL_START)
        # Wait SPI Xfer to be done.
        while not (self.bus.regs.frontend_spi_status.read() & SPI_STATUS_DONE):
            pass

# Frontend LDO Test --------------------------------------------------------------------------------

def frontend_ldo_test(host, port):
    bus = RemoteClient(host=host, port=port)
    bus.open()


    bus.regs.frontend_control.write(0)
    print(bus.regs.frontend_status.read())

    bus.regs.frontend_control.write(FRONTEND_CONTROL_LDO_EN)
    print(bus.regs.frontend_status.read())

    bus.close()

# Frontend Configure -------------------------------------------------------------------------------

def frontend_configure(host, port, channel, coupling, attenuation):
    bus = RemoteClient(host=host, port=port)
    bus.open()

    # LDO.
    def configure_ldo(enable):
        control_value  = bus.regs.frontend_control.read()
        control_value &= ~(     1 * FRONTEND_CONTROL_LDO_EN)
        control_value |=  (enable * FRONTEND_CONTROL_LDO_EN)
        bus.regs.frontend_control.write(control_value)

    configure_ldo(1)

    # Coupling.
    def configure_coupling(channel, coupling):
        value = {
            "AC" : FRONTEND_AC_COUPLING,
            "DC" : FRONTEND_DC_COUPLING,
        }[coupling.upper()]
        control_value  = bus.regs.frontend_control.read()
        control_value &= ~((    1 << channel) * FRONTEND_CONTROL_COUPLING)
        control_value |=  ((value << channel) * FRONTEND_CONTROL_COUPLING)
        bus.regs.frontend_control.write(control_value)

    configure_coupling(channel, coupling)

    # Attenuation.
    def configure_attenuation(channel, attenuation):
        value = {
            "1X"  :  FRONTEND_1X_ATTENUATION,
            "10X" : FRONTEND_10X_ATTENUATION,
        }[attenuation.upper()]
        print(value)
        control_value  = bus.regs.frontend_control.read()
        print(f"0b{control_value:032b}")
        control_value &= ~((    1 << channel) * FRONTEND_CONTROL_ATTENUATION)
        print(f"0b{control_value:032b}")
        control_value |=  ((value << channel) * FRONTEND_CONTROL_ATTENUATION)
        print(f"0b{control_value:032b}")
        bus.regs.frontend_control.write(control_value)

    configure_attenuation(channel, attenuation)

    print(f"0b{bus.regs.frontend_control.read():032b}")

    bus.close()

# Run ----------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Frontend test utility")
    parser.add_argument("--host",    default="localhost",   help="Host ip address")
    parser.add_argument("--port",    default="1234",        help="Host bind port.")

    parser.add_argument("--ldo-test", action="store_true", help="Test LDO.")

    parser.add_argument("--channel",     default="0",  help="Select Channel.")
    parser.add_argument("--coupling",    default="AC", help="Select Coupling: AC (default) or DC.")
    parser.add_argument("--attenuation", default="1X", help="Select Attenuation: 1X (default) or 10X.")
    args = parser.parse_args()

    host = args.host
    port = int(args.port, 0)

    channel = int(args.channel, 0)

    if args.ldo_test:
        frontend_ldo_test(host=host, port=port)

    frontend_configure(host=host, port=port,
        channel     = channel,
        coupling    = args.coupling,
        attenuation = args.attenuation,
    )

if __name__ == "__main__":
    main()
