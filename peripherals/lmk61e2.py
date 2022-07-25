#
# This file is part of Thunderscope-LiteX project.
#
# Copyright (c) 2022 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import sys

from .i2c import *

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
#                               D E S C R I P T I O N                                              #
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #

# N/A.

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
#                               D E F I N I T I O N S                                              #
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #

LMK61E2_I2C_ADDR = 0x58
LMK61E2_I2C_CONF = {
    # From configuration tool (Reg : Value).
    0x00 : 0x10,
    0x01 : 0x0b,
    0x02 : 0x33,
    0x08 : 0xb0,
    0x09 : 0x01,
    0x10 : 0x00,
    0x11 : 0x80,
    0x15 : 0x01,
    0x16 : 0x00,
    0x17 : 0x05,
    0x19 : 0x00,
    0x1a : 0x32,
    0x1b : 0x00,
    0x1c : 0x00,
    0x1d : 0x00,
    0x1e : 0x00,
    0x1f : 0x00,
    0x20 : 0x01,
    0x21 : 0x0c,
    0x22 : 0x28,
    0x23 : 0x03,
    0x24 : 0x08,
    0x25 : 0x00,
    0x26 : 0x00,
    0x27 : 0x00,
    0x2f : 0x00,
    0x30 : 0x00,
    0x31 : 0x10,
    0x32 : 0x00,
    0x33 : 0x00,
    0x34 : 0x00,
    0x35 : 0x00,
    0x38 : 0x00,
    0x48 : 0x02,
}

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
#                                  G A T E W A R E                                                 #
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #

# N/A (Directly reusing LiteX's bitbang I2C core).

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
#                                  S O F T W A R E                                                 #
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #

class LMK61E2Driver:
    def __init__(self, bus, name, addr):
        self.i2c  = I2CDriver(bus=bus, name=name)
        self.addr = addr

    def init(self, config, debug=False):
        for reg, value in config.items():
            if debug:
                print(f"0x{reg:02x}", end="")
            ack = 0
            while (not ack):
                if debug:
                    print(".", end="")
                sys.stdout.flush()
                self.i2c.start_cond()
                ack =  self.i2c.write(I2C_W_ADDR(self.addr))
                ack &= self.i2c.write(reg)
                ack &= self.i2c.write(value)
                self.i2c.stop_cond()
            if debug:
                print("")
