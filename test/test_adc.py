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
from peripherals.lmh6518 import *
from peripherals.lmk61e2 import *
from peripherals.trigger import *
from peripherals.had1511_adc import *

from test_i2c import *

# Frontend Constants -------------------------------------------------------------------------------

FRONTEND_CONTROL_LDO_EN      = (1 <<  0)
FRONTEND_CONTROL_COUPLING    = (1 <<  8)
FRONTEND_CONTROL_ATTENUATION = (1 << 16)

FRONTEND_STATUS_LDO_PWR_GOOD = (1 <<  0)

FRONTEND_AC_COUPLING = 0
FRONTEND_DC_COUPLING = 1

FRONTEND_1X_ATTENUATION  = 0
FRONTEND_10X_ATTENUATION = 1

# ADC Constants ------------------------------------------------------------------------------------

ADC_CONTROL_LDO_EN   = (1 <<  0)
ADC_CONTROL_PLL_EN   = (1 <<  1)
ADC_CONTROL_RST      = (1 <<  2)
ADC_CONTROL_PWR_DOWN = (1 <<  3)

ADC_STATUS_LDO_PWR_GOOD = (1 <<  0)

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

    adc_configure(host=host, port=port)

if __name__ == "__main__":
    main()
