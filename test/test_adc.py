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

from test_i2c import *

# ADC Constants ------------------------------------------------------------------------------------

ADC_CONTROL_LDO_EN   = (1 <<  0)
ADC_CONTROL_PLL_EN   = (1 <<  1)
ADC_CONTROL_RST      = (1 <<  2)
ADC_CONTROL_PWR_DOWN = (1 <<  3)

ADC_STATUS_LDO_PWR_GOOD = (1 <<  0)

# ADC SPI Driver -----------------------------------------------------------------------------------

SPI_CONTROL_START  = (1 << 0)
SPI_CONTROL_LENGTH = (1 << 8)
SPI_STATUS_DONE    = (1 << 0)

class ADCSPIDriver:
    def __init__(self, bus):
        self.bus = bus

    def write(self, cs, data):
        assert len(data) <= 3
        # Convert data to bytes (if not already).
        data = data if isinstance(data, (bytes, bytearray)) else bytes(data)
        # Set Chip Select.
        self.bus.regs.adc_spi_cs.write((1 << cs))
        # Prepare MOSI data.
        mosi_bits = len(data)*8
        mosi_data = int.from_bytes(data, byteorder="big")
        mosi_data <<= (24 - mosi_bits)
        self.bus.regs.adc_spi_mosi.write(mosi_data)
        # Start SPI Xfer.
        self.bus.regs.adc_spi_control.write(mosi_bits*SPI_CONTROL_LENGTH | SPI_CONTROL_START)
        # Wait SPI Xfer to be done.
        while not (self.bus.regs.adc_spi_status.read() & SPI_STATUS_DONE):
            pass

# ADC Configure ------------------------------------------------------------------------------------

def adc_configure(host, port):
    bus = RemoteClient(host=host, port=port)
    bus.open()

    # LDO.
    def configure_ldo(enable):
        print("Configure LDO EN...")
        control_value  = bus.regs.adc_control.read()
        control_value &= ~(     1 * ADC_CONTROL_LDO_EN)
        control_value |=  (enable * ADC_CONTROL_LDO_EN)
        bus.regs.adc_control.write(control_value)

    configure_ldo(1)

    # PLL.
    def configure_pll(enable):
        print("Configure PLL EN...")
        control_value  = bus.regs.adc_control.read()
        control_value &= ~(     1 * ADC_CONTROL_PLL_EN)
        control_value |=  (enable * ADC_CONTROL_PLL_EN)
        bus.regs.adc_control.write(control_value)

    configure_pll(1)

    # PLL I2C.
    class LMK61E2:
        # From configuration tool (Reg : Value).
        conf = {
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
        def __init__(self, addr):
            self.addr = addr
            self.i2c  = BitBangI2C(bus.regs)

        def init(self):
            print("Configure PLL I2C...")
            for reg, value in self.conf.items():
                print(reg)
                self.i2c.start_cond()
                self.i2c.write(I2C_W_ADDR(self.addr))
                self.i2c.write(reg)
                self.i2c.write(value)
                self.i2c.stop_cond()

    lmk61e2 = LMK61E2(addr=LMK61E2_I2C_ADDR)
    #lmk61e2.init()

    # RST.
    def configure_rst(enable):
        print("Configure RST...")
        control_value  = bus.regs.adc_control.read()
        control_value &= ~(     1 * ADC_CONTROL_RST)
        control_value |=  (enable * ADC_CONTROL_RST)
        bus.regs.adc_control.write(control_value)

    configure_rst(0)

    # PWR_DOWN.
    def configure_pwr_down(enable):
        print("Configure PWR_DOWN...")
        control_value  = bus.regs.adc_control.read()
        control_value &= ~(     1 * ADC_CONTROL_PWR_DOWN)
        control_value |=  (enable * ADC_CONTROL_PWR_DOWN)
        bus.regs.adc_control.write(control_value)

    configure_pwr_down(0)


    # HAD1511.
    print("Configure HAD1511...")
    import sys
    sys.path.append("..")
    from peripherals.had1511_adc import HAD1511ADCDriver

    n = 0
    spi = ADCSPIDriver(bus)
    adc = HAD1511ADCDriver(bus, spi, n=0)
    adc.reset()
    adc.data_mode(n={1: [n], 2: [0, 1], 4: [0, 1]}[4])
    adc.enable_ramp_pattern()
    for i in range(8):
        print(adc.get_samplerate(duration=0.5))

    bus.close()

# Run ----------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Frontend test utility")
    parser.add_argument("--host",    default="localhost",   help="Host ip address")
    parser.add_argument("--port",    default="1234",        help="Host bind port.")

    args = parser.parse_args()

    host = args.host
    port = int(args.port, 0)

    adc_configure(host=host, port=port)

if __name__ == "__main__":
    main()
