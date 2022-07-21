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

# I2C Constants ------------------------------------------------------------------------------------

I2C_SCL    = 0x01
I2C_SDAOE  = 0x02
I2C_SDAOUT = 0x04
I2C_SDAIN  = 0x01

I2C_DELAY = 1
I2C_WRITE = 0
I2C_READ  = 1

def I2C_W_ADDR(addr):
    return (addr & 0x7f) << 1

def I2C_R_ADDR(addr):
    return ((addr & 0x7f) << 1) | 1

MCP4728_I2C_ADDR = 0x61
LMK61E2_I2C_ADDR = 0x58

# BitBangI2C ---------------------------------------------------------------------------------------

class BitBangI2C:
    def __init__(self, regs):
        self.regs = regs

        self.started = 0

        self.regs.i2c_w.write(I2C_SCL)
        # Check that the I2C bus is ready.
        while(not (self.regs.i2c_r.read() & I2C_SDAIN)):
            time.sleep(1e-3)

    # I2C bit-banging functions from http://en.wikipedia.org/wiki/I2c.
    def read_bit(self):
        # Let the Slave drive data.
        self.regs.i2c_w.write(0)
        self.regs.i2c_w.write(I2C_SCL)
        bit = (self.regs.i2c_r.read() & I2C_SDAIN)
        self.regs.i2c_w.write(0)
        return bit

    def write_bit(self, bit):
        if bit:
            self.regs.i2c_w.write(I2C_SDAOE| I2C_SDAOUT)
        else:
            self.regs.i2c_w.write(I2C_SDAOE)
        # Clock stretching.
        self.regs.i2c_w.write(self.regs.i2c_w.read() | I2C_SCL);
        self.regs.i2c_w.write(self.regs.i2c_w.read() & ~I2C_SCL);

    def start_cond(self):
        if self.started:
            # Set SDA to 1.
            self.regs.i2c_w.write(I2C_SDAOE| I2C_SDAOUT);
            self.regs.i2c_w.write(self.regs.i2c_w.read() | I2C_SCL);
        # SCL is high, set SDA from 1 to 0.
        self.regs.i2c_w.write(I2C_SDAOE| I2C_SCL)
        self.regs.i2c_w.write(I2C_SDAOE)
        self.started = 1

    def stop_cond(self):
        # Set SDA to 0.
        self.regs.i2c_w.write(I2C_SDAOE)
        # Clock stretching.
        self.regs.i2c_w.write(I2C_SDAOE| I2C_SCL)
        # SCL is high, set SDA from 0 to 1.
        self.regs.i2c_w.write(I2C_SCL)
        self.started = 0

    def write(self, byte):
        for bit in range(8):
            self.write_bit(byte & 0x80)
            byte <<= 1
        ack = not self.read_bit()
        return ack

    def read(self, ack):
        byte = 0
        for bit in range(8):
            byte <<= 1
            byte |= self.read_bit()
        self.write(not ack)
        return byte

    def poll(self, addr):
        self.start_cond()
        ack  = self.write(I2C_W_ADDR(addr))
        ack |= self.write(I2C_R_ADDR(addr))
        self.stop_cond()
        return ack

# I2C Scan -----------------------------------------------------------------------------------------

def i2c_scan(host, port):
    bus = RemoteClient(host=host, port=port)
    bus.open()

    i2c = BitBangI2C(bus.regs)

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

    class MCP4728:
        def __init__(self, addr):
            self.addr = addr
            self.i2c  = BitBangI2C(bus.regs)

        def reset(self):
            # General Call Reset.
            self.i2c.start_cond()
            self.i2c.write(I2C_W_ADDR(self.addr))
            self.i2c.write(0x00)
            self.i2c.write(0x06)
            self.i2c.stop_cond()

        def set_ch(self, n, value):
            value &= 0xfff
            self.i2c.start_cond()
            self.i2c.write(I2C_W_ADDR(self.addr))
            self.i2c.write(0x40 + (n << 1))
            self.i2c.write((value >> 8) & 0x0f)
            self.i2c.write((value >> 0) & 0xff)
            self.i2c.stop_cond()


    mcp4728 = MCP4728(addr=MCP4728_I2C_ADDR)
    mcp4728.reset()

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
