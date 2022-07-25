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

sys.path.append("..")
from peripherals.spi import *

# Frontend Constants -------------------------------------------------------------------------------

FRONTEND_CONTROL_LDO_EN      = (1 <<  0)
FRONTEND_CONTROL_COUPLING    = (1 <<  8)
FRONTEND_CONTROL_ATTENUATION = (1 << 16)

FRONTEND_STATUS_LDO_PWR_GOOD = (1 <<  0)

FRONTEND_AC_COUPLING = 0
FRONTEND_DC_COUPLING = 1

FRONTEND_1X_ATTENUATION  = 0
FRONTEND_10X_ATTENUATION = 1

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

# PGA Configure ------------------------------------------------------------------------------------

class LMH6518Driver:
    def __init__(self, bus):
        self.spi = SPIDriver(bus=bus, name="frontend_spi")

    def set(self, channel, preamp_db=10, atten_db=10, bw_mhz=900):
        # Compute Configuration Fields.
        preamp_field =  {
            10 : 0b0,
            30 : 0b1,
        }[preamp_db]
        atten_field = {
            0  : 0b0000,
            2  : 0b0001,
            4  : 0b0010,
            6  : 0b0011,
            8  : 0b0100,
            10 : 0b0101,
            12 : 0b0110,
            14 : 0b0111,
            16 : 0b1000,
            18 : 0b1001,
            20 : 0b1010,
        }[atten_db]
        bw_field = {
             900 : 0b000,
              20 : 0b001,
             100 : 0b010,
             200 : 0b011,
             350 : 0b100,
             650 : 0b100,
             750 : 0b110,
        }[bw_mhz]

        # Prepare SPI Data.
        cmd_field = 0
        dat_field = 0
        dat_field |= (           1 << 10) # Aux Hi-Z.
        dat_field |= (bw_field     <<  6) # Filter.
        dat_field |= (preamp_field <<  4) # Preamp.
        dat_field |= (atten_field  <<  0) # Attenuation.
        spi_data  = []
        spi_data.append(cmd_field)
        spi_data.append((dat_field >> 8) & 0xff)
        spi_data.append((dat_field >> 0) & 0xff)
        self.spi.write(cs=channel, data=spi_data)

def pga_configure(host, port, channel, preamp_db, atten_db, bw_mhz):
    bus = RemoteClient(host=host, port=port)
    bus.open()

    # Programmable Gain Amplifier.
    pga = LMH6518Driver(bus)
    pga.set(channel, preamp_db, atten_db, bw_mhz)

    bus.close()

# Run ----------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Frontend test utility")
    parser.add_argument("--host",    default="localhost",   help="Host ip address")
    parser.add_argument("--port",    default="1234",        help="Host bind port.")

    parser.add_argument("--ldo-test", action="store_true", help="Test LDO.")

    parser.add_argument("--channel",     default="0",  help="Select Channel.")
    parser.add_argument("--coupling",    default="AC", help="Set Frontend Coupling: AC (default) or DC.")
    parser.add_argument("--attenuation", default="1X", help="Set Frontend Attenuation: 1X (default) or 10X.")

    parser.add_argument("--pga-preamp",  default="10",   help="Set PGA Preamp Gain (dB), 10 or 30.")
    parser.add_argument("--pga-atten",   default="10",   help="Set PGA Attenuation (dB), 0 to 20 (with 2 increment).")
    parser.add_argument("--pga-bw",      default="full", help="Set PGA Bandwidth (MHz), 20, 100, 200, full.")

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

    preamp_db = int(args.pga_preamp, 0)
    atten_db  = int(args.pga_atten,  0)
    if args.pga_bw == "full":
        bw_mhz = 900
    else:
        bw_mhz = int(args.pga_bw, 0)
    pga_configure(host=host, port=port,
        channel   = channel,
        preamp_db = preamp_db,
        atten_db  = atten_db,
        bw_mhz    = bw_mhz,
    )

if __name__ == "__main__":
    main()
