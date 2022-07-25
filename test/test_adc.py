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

import sys
sys.path.append("..")
from peripherals.spi import *
from peripherals.i2c import *
from peripherals.lmk61e2 import *
from peripherals.trigger import *
from peripherals.had1511_adc import *

from test_i2c import *

# ADC Constants ------------------------------------------------------------------------------------

ADC_CONTROL_LDO_EN   = (1 <<  0)
ADC_CONTROL_PLL_EN   = (1 <<  1)
ADC_CONTROL_RST      = (1 <<  2)
ADC_CONTROL_PWR_DOWN = (1 <<  3)

ADC_STATUS_LDO_PWR_GOOD = (1 <<  0)

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

    lmk61e2 = LMK61E2Driver(bus=bus, name="i2c", addr=LMK61E2_I2C_ADDR)
    lmk61e2.init(config=LMK61E2_I2C_CONF)

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

    n = 0
    spi = SPIDriver(bus=bus, name="adc_spi")
    adc = HAD1511ADCDriver(bus, spi, n=0)
    adc.reset()
    adc.data_mode(n={1: [n], 2: [0, 1], 4: [0, 1]}[4])
    adc.enable_ramp_pattern()
    #adc.enable_sync_pattern()
    for i in range(8):
        print(adc.get_samplerate(duration=0.5))

    # Delay calibration.
    bus.regs.adc_had1511_control.write(HAD1511_CORE_CONTROL_DELAY_RST)
    bitslip_count_last = bus.regs.adc_had1511_bitslip_count.read()
    for d in range(32):
        bus.regs.adc_had1511_control.write(HAD1511_CORE_CONTROL_DELAY_INC)
        time.sleep(0.2)
        bitslip_count = bus.regs.adc_had1511_bitslip_count.read()
        bitslip_diff  = (bitslip_count - bitslip_count_last) # FIXME: Handle rollover.
        bitslip_count_last = bitslip_count
        print(f"Delay {d} / BitSlip Errors: {bitslip_diff}")

    bus.regs.adc_had1511_control.write(HAD1511_CORE_CONTROL_DELAY_RST)

    trigger = TriggerDriver(bus)
    trigger.reset()
    trigger.enable()

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
