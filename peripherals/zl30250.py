#
# This file is part of Thunderscope-LiteX project.
#
# Copyright (c) 2022 Florent Kermarrec <florent@enjoy-digital.fr>
# Copyright (c) 2023 Aleksa Bjelogrlic <aleksa@eevengers.com>
# Copyright (c) 2023 John Simons <john@totalitee.nl>

# SPDX-License-Identifier: BSD-2-Clause

import sys
import time

from .i2c import *

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
#                               D E S C R I P T I O N                                              #
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #

# N/A.

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
#                               D E F I N I T I O N S                                              #
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #

ZL30250_I2C_ADDR = 0x6C
ZL30250_I2C_WRITE_COMMAND = 0x02
ZL30250_I2C_READ_COMMAND = 0x03
ZL30250_I2C_CONF = {
    # From configuration tool (Reg : Value).
    0X0009 : 0x02,
    0X0621 : 0x08,
    0X0631 : 0x40,
    0X0100 : 0x06,
    0X0101 : 0x20,
    0X0102 : 0x02,
    0X0103 : 0x80,
    0X010A : 0x20,
    0X010B : 0x03,
    0X0114 : 0x0D,
    0X0120 : 0x06,
    0X0125 : 0xC0,
    0X0126 : 0x60,
    0X0127 : 0x7F,
    0X0129 : 0x04,
    0X012A : 0xB3,
    0X012B : 0xC0,
    0X012C : 0x80,
    0X001C : 0x10,
    0X001D : 0x80,
    0X0340 : 0x03,
    0X0201 : 0x41,
    0X0221 : 0x35,
    0X0222 : 0x40,
    0X000C : 0x02,
    0X000B : 0x01,
    0X000D : 0x05, #Need 10ms beforehand, will be fine through jtag
}

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
#                                  G A T E W A R E                                                 #
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #

# N/A (Directly reusing LiteX's bitbang I2C core).

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
#                                  S O F T W A R E                                                 #
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #

class ZL30250Driver:
    def __init__(self, bus, name, addr):
        self.i2c  = I2CDriver(bus=bus, name=name)
        self.addr = addr

    def init(self, config, debug=False):
        for reg, value in config.items():
            if (reg == 0x000D):
                time.sleep(0.1)
            if debug:
                print(f"0x{reg:02x}", end="")
            ack = 0
            while (not ack):
                if debug:
                    print(".", end="")
                sys.stdout.flush()
                self.i2c.start_cond()
                ack =  self.i2c.write(I2C_W_ADDR(self.addr))
                ack =  self.i2c.write(ZL30250_I2C_WRITE_COMMAND)
                ack &= self.i2c.write(reg >> 8)
                ack &= self.i2c.write(reg & 0xFF)
                ack &= self.i2c.write(value)
                self.i2c.stop_cond()
            if debug:
                print("")
                
    def read(self, reg, debug=True):
        ack = 0
        while (not ack):

            sys.stdout.flush()
            self.i2c.start_cond()
            ack =  self.i2c.write(I2C_R_ADDR(self.addr))
            ack &=  self.i2c.write(ZL30250_I2C_READ_COMMAND)
            ack &= self.i2c.write((reg >> 8) & 0x0f) 
            ack &= self.i2c.write((reg >> 0) & 0xff)
            value = self.i2c.read(1)
            if debug:
                print(f"0x{reg:02x} 0x{value:02x}", end="")
            self.i2c.stop_cond()