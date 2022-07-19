#!/usr/bin/env python3

#
# This file is part of Thunderscope-LiteX project.
#
# Copyright (c) 2022 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import os

from migen import *

from litex.build.generic_platform import *
from litex.build.xilinx import XilinxPlatform, VivadoProgrammer
from litex.build.openocd import OpenOCD

from litex.soc.interconnect.csr import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.interconnect import stream

from litex.soc.cores.clock import *
from litex.soc.cores.led import LedChaser
from litex.soc.cores.xadc import XADC
from litex.soc.cores.dna  import DNA
from litex.soc.cores.bitbang import I2CMaster
from litex.soc.cores.gpio import GPIOOut
from litex.soc.cores.spi import SPIMaster

from litepcie.phy.s7pciephy import S7PCIEPHY
from litepcie.software import generate_litepcie_software

from peripherals.had1511_adc import HAD1511ADC
from peripherals.trigger import Trigger

# IOs ----------------------------------------------------------------------------------------------

_io = [
    # Leds.
    # -----
    ("user_led_n", 0, Pins("U21"), IOStandard("LVCMOS33")), # Green.
    ("user_led_n", 1, Pins("R17"), IOStandard("LVCMOS33")), # Red.
    ("user_led_n", 2, Pins("Y22"), IOStandard("LVCMOS33")), # Green.
    ("user_led_n", 3, Pins("T21"), IOStandard("LVCMOS33")), # Red.

    # PCIe / Gen2 X4.
    # ---------------
    ("pcie_x4", 0,
        Subsignal("rst_n", Pins("V14"), IOStandard("LVCMOS33"), Misc("PULLUP=TRUE")),
        Subsignal("clk_p", Pins("F10")),
        Subsignal("clk_n", Pins("E10")),
        Subsignal("rx_p",  Pins("D9 B10 D11 D8")),
        Subsignal("rx_n",  Pins("C9 A10 C11 A8")),
        Subsignal("tx_p",  Pins("D7 B6 D5 B4")),
        Subsignal("tx_n",  Pins("C7 A6 C5 A4")),
    ),

    # Frontend.
    # ---------

    # Coupling.
    ("fe_coupling", 0, Pins("H22 L19 H19 N20"), IOStandard("LVCMOS33")),

    # Attenuation.
    ("fe_attenuation", 0, Pins("H20 K19 J19 M21"), IOStandard("LVCMOS33")),

    # Programmable Gain Amplifier.
    ("fe_pga", 0,
        Subsignal("clk",  Pins("K21")),
        Subsignal("cs_n", Pins("G20 K18 L20 L21")),
        Subsignal("mosi", Pins("K22")),
        IOStandard("LVCMOS33"),
    ),

    # Misc / TBD.
    ("probe_comp", 0, Pins("N1"), IOStandard("LVCMOS33")),
    ("fe_pg",      0, Pins("J5"), IOStandard("LVCMOS33")),
    ("osc_oe",     0, Pins("N3"), IOStandard("LVCMOS33")),
    ("acq_en",     0, Pins("M4"), IOStandard("LVCMOS33")),
    ("fe_en",      0, Pins("K6"), IOStandard("LVCMOS33")),


    # I2C bus.
    # --------
    # - Trim DAC (MCP4728 @ 0x61).
    # - PLL      (LMK61E2 @ 0x58).
    ("i2c", 0,
        Subsignal("sda", Pins("J14")),
        Subsignal("scl", Pins("H14")),
        IOStandard("LVCMOS33"),
    ),

    # ADC / HMCAD1511.
    # ----------------

    # Control.
    ("adc_ctrl", 0,
        Subsignal("pd",    Pins("L4")),
        Subsignal("pg",    Pins("R6")),
        Subsignal("rst_n", Pins("M6")),
        Subsignal("cs",    Pins("J4")),
        Subsignal("sclk",  Pins("J6")),
        Subsignal("sdata", Pins("L5")),
        IOStandard("LVCMOS33"),
    ),

    # Datapath.
    ("adc_data", 0,
        Subsignal("lclk_p", Pins("R2")), # Bitclock.
        Subsignal("lclk_n", Pins("R1")),
        Subsignal("fclk_p", Pins("U2")), # Frameclock.
        Subsignal("fclk_n", Pins("U1")),
        Subsignal("d_p", Pins("U4 V3 U7 V8 R5 T4 U6 R7")), # Data.
        Subsignal("d_n", Pins("V4 V2 V6 V7 T5 T3 U5 T7")),
        IOStandard("LVDS_25"),
        Misc("DIFF_TERM=TRUE"),
    ),
]

# Platform -----------------------------------------------------------------------------------------

class Platform(XilinxPlatform):
    def __init__(self, toolchain="vivado"):
        XilinxPlatform.__init__(self, "xc7a100tfgg484-2", _io, toolchain=toolchain)

        self.toolchain.bitstream_commands = [
            "set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]",
            "set_property BITSTREAM.CONFIG.CONFIGRATE 16 [current_design]",
            "set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]",
            "set_property CFGBVS VCCO [current_design]",
            "set_property CONFIG_VOLTAGE 3.3 [current_design]",
        ]

        self.toolchain.additional_commands = [
            # Non-Multiboot SPI-Flash bitstream generation.
            "write_cfgmem -force -format bin -interface spix4 -size 16 -loadbit \"up 0x0 {build_name}.bit\" -file {build_name}.bin",
        ]

    def create_programmer(self, name='openocd'):
        if name == 'openocd':
            return OpenOCD("openocd_xc7_ft232.cfg", "bscan_spi_xc7a35t.bit")
        elif name == 'vivado':
            # TODO: some board versions may have s25fl128s
            return VivadoProgrammer(flash_part='s25fl256sxxxxxx0-spi-x1_x2_x4')

    def do_finalize(self, fragment):
        XilinxPlatform.do_finalize(self, fragment)

# CRG ----------------------------------------------------------------------------------------------

class CRG(Module):
    def __init__(self, platform, sys_clk_freq):
        self.rst = Signal()
        self.clock_domains.cd_sys = ClockDomain()

        # CFGM Clk ~65MHz.
        cfgm_clk      = Signal()
        cfgm_clk_freq = int(65e6)
        self.specials += Instance("STARTUPE2",
            i_CLK       = 0,
            i_GSR       = 0,
            i_GTS       = 0,
            i_KEYCLEARB = 1,
            i_PACK      = 0,
            i_USRCCLKO  = cfgm_clk,
            i_USRCCLKTS = 0,
            i_USRDONEO  = 1,
            i_USRDONETS = 1,
            o_CFGMCLK   = cfgm_clk
        )

        # PLL
        self.submodules.pll = pll = S7PLL(speedgrade=-1)
        self.comb += pll.reset.eq(self.rst)
        pll.register_clkin(cfgm_clk, cfgm_clk_freq)
        pll.create_clkout(self.cd_sys, sys_clk_freq)
        platform.add_false_path_constraints(self.cd_sys.clk, pll.clkin) # Ignore sys_clk to pll.clkin path created by SoC's rst.

# BaseSoC -----------------------------------------------------------------------------------------

class BaseSoC(SoCMini):
    def __init__(self, sys_clk_freq=int(125e6), with_pcie=True, with_frontend=True, with_adc=False, with_jtagbone=True):
        platform = Platform()

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = CRG(platform, sys_clk_freq)

        # SoCMini ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform, sys_clk_freq,
            ident         = "LitePCIe SoC on ThunderScope",
            ident_version = True,
        )

        # JTAGBone ---------------------------------------------------------------------------------
        if with_jtagbone:
            self.add_jtagbone()

        # XADC -------------------------------------------------------------------------------------
        self.submodules.xadc = XADC()

        # DNA --------------------------------------------------------------------------------------
        self.submodules.dna = DNA()
        self.dna.add_timing_constraints(platform, sys_clk_freq, self.crg.cd_sys.clk)

        # Leds -------------------------------------------------------------------------------------
        self.submodules.leds = LedChaser(
            pads         = platform.request_all("user_led_n"),
            sys_clk_freq = sys_clk_freq,
            polarity     = 1,
        )
        self.leds.add_pwm(default_width=128, default_period=1024) # Default to 1/8 to reduce brightness.

        # PCIe -------------------------------------------------------------------------------------
        if with_pcie:
            self.submodules.pcie_phy = S7PCIEPHY(platform, platform.request("pcie_x4"),
                data_width = 128,
                bar0_size  = 0x20000
            )
            self.add_pcie(phy=self.pcie_phy, ndmas=1, dma_buffering_depth=8192)
            # FIXME: Apply it to all targets (integrate it in LitePCIe?).
            platform.add_period_constraint(self.crg.cd_sys.clk, 1e9/sys_clk_freq)
            platform.toolchain.pre_placement_commands.add("set_clock_groups -group [get_clocks {sys_clk}] -group [get_clocks userclk2] -asynchronous", sys_clk=self.crg.cd_sys.clk)
            platform.toolchain.pre_placement_commands.add("set_clock_groups -group [get_clocks {sys_clk}] -group [get_clocks clk_125mhz] -asynchronous", sys_clk=self.crg.cd_sys.clk)
            platform.toolchain.pre_placement_commands.add("set_clock_groups -group [get_clocks {sys_clk}] -group [get_clocks clk_250mhz] -asynchronous", sys_clk=self.crg.cd_sys.clk)
            platform.toolchain.pre_placement_commands.add("set_clock_groups -group [get_clocks clk_125mhz] -group [get_clocks clk_250mhz] -asynchronous")

            # ICAP (For FPGA reload over PCIe).
            from litex.soc.cores.icap import ICAP
            self.submodules.icap = ICAP()
            self.icap.add_reload()
            self.icap.add_timing_constraints(platform, sys_clk_freq, self.crg.cd_sys.clk)


        # Frontend / ADC ---------------------------------------------------------------------------

        # I2C Bus:
        # - Trim DAC (MCP4728 @ 0x61).
        # - PLL      (LMK61E2 @ 0x58).
        self.submodules.i2c = I2CMaster(platform.request("i2c"))

        # Frontend.
        if with_frontend:
            # AC/DC Coupling (Discrete IOs).
            self.submodules.fe_coupling = GPIOOut(
                pads  = platform.request("fe_coupling"),
                reset = 0b0000,
            )

            # Attenuation (Discrete IOs).
            self.submodules.fe_attenuation = GPIOOut(
                pads  = platform.request("fe_attenuation"),
                reset = 0b0000,
            )

            # Programmable Gain Amplifier (LMH6518/SPI).
            fe_pga_pads = self.platform.request("fe_pga")
            fe_pga_pads.miso = Signal()
            self.submodules.spi = SPIMaster(
                pads         = fe_pga_pads,
                data_width   = 24,
                sys_clk_freq = self.sys_clk_freq,
                spi_clk_freq = 1e6
            )

        # ADC.
        if with_adc:

            # Frontend.
            # TODO.

            # SPI.
            # TODO.

            # Trigger.
            self.submodules.trigger = Trigger()

            # ADC.
            self.submodules.adc = HAD1511ADC(self.platform.request("adc_data"), sys_clk_freq, polarity=1)
            self.submodules.gate = stream.Gate([("data", 64)], sink_ready_when_disabled=True)
            self.submodules.conv = stream.Converter(64, self.pcie_phy.data_width)
            self.comb += self.gate.enable.eq(self.trigger.enable)

            # Datapath ---------------------------------------------------------------------------------
            self.submodules += stream.Pipeline(
                self.adc,
                self.gate,
                self.conv,
                self.pcie_dma0.sink,
            )

# Build --------------------------------------------------------------------------------------------

def main():
    from litex.soc.integration.soc import LiteXSoCArgumentParser
    parser = LiteXSoCArgumentParser(description="LitePCIe SoC on ThunderScope")
    target_group = parser.add_argument_group(title="Target options")
    target_group.add_argument("--build",     action="store_true", help="Build bitstream.")
    target_group.add_argument("--load",      action="store_true", help="Load bitstream.")
    target_group.add_argument("--flash",     action="store_true", help="Flash bitstream.")
    target_group.add_argument("--driver",    action="store_true", help="Generate PCIe driver.")
    args = parser.parse_args()

    soc = BaseSoC()

    builder  = Builder(soc, csr_csv="test/csr.csv")
    builder.build(run=args.build)

    if args.driver:
        generate_litepcie_software(soc, os.path.join(builder.output_dir, "driver"))

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

if __name__ == "__main__":
    main()
