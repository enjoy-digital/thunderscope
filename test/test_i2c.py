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
from peripherals.i2c import *
from peripherals.mcp4728 import *

# I2C Scan -----------------------------------------------------------------------------------------

def i2c_scan(host, port):
    bus = RemoteClient(host=host, port=port)
    bus.open()

    i2c = I2CDriver(bus=bus, name="i2c")

    print("      0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f", end="");
    sys.stdout.flush()
    for addr in range(0, 0x80):
        if (addr % 0x10) == 0:
            print(f"\n0x{addr:02x}", end="")
        if i2c.poll(addr):
            print(f" {addr:02x}", end="")
        else:
            print(f" --", end="")
        sys.stdout.flush()
    print("")

    bus.close()

# MCP4728 Test -------------------------------------------------------------------------------------

def mcp4728_test(host, port):
    bus = RemoteClient(host=host, port=port)
    bus.open()

    mcp4728 = MCP4728Driver(bus=bus, name="i2c", addr=MCP4728_I2C_ADDR)
    mcp4728.rst()

    print("MCP4728 test...")
    for i in range(4):
        print(f"{i}/4")
        print("Setting channels to 0.")
        for n in range(4):
            mcp4728.set_ch(n=n, value=0)
        print("Setting channels to 2**12-1.")
        for n in range(4):
            mcp4728.set_ch(n=n, value=2**12-1)

    bus.close()

# Run ----------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="I2C test utility")
    parser.add_argument("--host",    default="localhost",   help="Host ip address")
    parser.add_argument("--port",    default="1234",        help="Host bind port.")

    parser.add_argument("--scan",         action="store_true", help="Scan I2C Bus.")
    parser.add_argument("--mcp4728-test", action="store_true", help="Test MCP4728 I2C Access/Config.")
    args = parser.parse_args()

    host = args.host
    port = int(args.port, 0)

    if args.scan:
        i2c_scan(host=host, port=port)

    if args.mcp4728_test:
        mcp4728_test(host=host, port=port)

if __name__ == "__main__":
    main()
