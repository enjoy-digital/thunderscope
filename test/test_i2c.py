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

# Run ----------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="I2C test utility")
    parser.add_argument("--host",    default="localhost",   help="Host ip address")
    parser.add_argument("--port",    default="1234",        help="Host bind port.")

    parser.add_argument("--scan",         action="store_true", help="Scan I2C Bus.")
    args = parser.parse_args()

    host = args.host
    port = int(args.port, 0)

    if args.scan:
        i2c_scan(host=host, port=port)

if __name__ == "__main__":
    main()
