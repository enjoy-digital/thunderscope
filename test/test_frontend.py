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
from peripherals.lmh6518 import *

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

def pga_configure(host, port, channel, preamp_db, atten_db, bw_mhz):
    bus = RemoteClient(host=host, port=port)
    bus.open()

    # Programmable Gain Amplifier.
    pga = LMH6518Driver(bus=bus, name="frontend_spi")
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
