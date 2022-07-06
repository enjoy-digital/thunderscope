#!/usr/bin/env python3

#
# This file is part of Thunderscope-LiteX project.
#
# Copyright (c) 2022 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

# Build/Use ----------------------------------------------------------------------------------------
# ./thunderscope.py --driver --build --load

#.Build the kernel and load it:
# cd build/<platform>/driver/kernel
# make
# sudo ./init.sh
#
# Test userspace utilities:
# cd build/<platform>/driver/user
# make
# ./litepcie_util info
# ./litepcie_util scratch_test
# ./litepcie_util dma_test

import os

from migen import *

from litex.build.generic_platform import *
from litex.build.xilinx import XilinxPlatform, VivadoProgrammer
from litex.build.openocd import OpenOCD

from litex.soc.interconnect.csr import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from litex.soc.cores.clock import *
from litex.soc.cores.led import LedChaser
from litex.soc.cores.xadc import XADC
from litex.soc.cores.dna  import DNA

from litedram.modules import MT41K512M16
from litedram.phy import s7ddrphy

from litepcie.phy.s7pciephy import S7PCIEPHY
from litepcie.software import generate_litepcie_software

# IOs ----------------------------------------------------------------------------------------------

_io = [
    # Leds.
    ("user_led", 0, Pins("P6"), IOStandard("LVCMOS33")),
    ("user_led", 1, Pins("N6"), IOStandard("LVCMOS33")),

    # PCIe / Gen2 X4.
    ("pcie_x4", 0,
        Subsignal("rst_n", Pins("L2"), IOStandard("LVCMOS33"), Misc("PULLUP=TRUE")),
        Subsignal("clk_p", Pins("B6")),
        Subsignal("clk_n", Pins("B5")),
        Subsignal("rx_p",  Pins("E4 A4 C4 G4")),
        Subsignal("rx_n",  Pins("E3 A3 C3 G3")),
        Subsignal("tx_p",  Pins("H2 F2 D2 B2")),
        Subsignal("tx_n",  Pins("H1 F1 D1 B1")),
    ),

    # PGA.
    ("pga", 0,
        Subsignal("sdio", Pins("L3")),
        Subsignal("sclk", Pins("K2")),
        Subsignal("cs",   Pins("M1 M5 R3 P5")),
        IOStandard("LVCMOS33"),
    ),

    # Atten.
    ("atten", 0, Pins("P4 M2 P1 K1"), IOStandard("LVCMOS33")),

    # DC Coupling.
    ("dc_cpl", 0, Pins("P3 N2 T2 K3"), IOStandard("LVCMOS33")),

    # I2C.
    ("i2c", 0,
        Subsignal("sda", Pins("N4")),
        Subsignal("scl", Pins("K5")),
        IOStandard("LVCMOS33"),
    ),

    # Frontend.
    ("probe_comp", 0, Pins("N1"), IOStandard("LVCMOS33")),
    ("fe_pg",      0, Pins("J5"), IOStandard("LVCMOS33")),
    ("osc_oe",     0, Pins("N3"), IOStandard("LVCMOS33")),
    ("acq_en",     0, Pins("M4"), IOStandard("LVCMOS33")),
    ("fe_en",      0, Pins("K6"), IOStandard("LVCMOS33")),

    # ADC / HMCAD1511.
    ("adc_ctrl", 0,
        Subsignal("pd",    Pins("L4")),
        Subsignal("pg",    Pins("R6")),
        Subsignal("rst_n", Pins("M6")),
        Subsignal("cs",    Pins("J4")),
        Subsignal("sclk",  Pins("J6")),
        Subsignal("sdata", Pins("L5")),
        IOStandard("LVCMOS33"),
    ),
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
        XilinxPlatform.__init__(self, "xc7a35tcsg325-2", _io, toolchain=toolchain)

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

class BaseSoC(SoCCore):
    def __init__(self, sys_clk_freq=int(125e6)):
        platform = Platform()

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = CRG(platform, sys_clk_freq)

        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, sys_clk_freq,
            uart_name = "stub",
            cpu_type  = None,
            ident     = "LitePCIe SoC on ThunderScope"
        )

        # XADC -------------------------------------------------------------------------------------
        self.submodules.xadc = XADC()

        # DNA --------------------------------------------------------------------------------------
        self.submodules.dna = DNA()
        self.dna.add_timing_constraints(platform, sys_clk_freq, self.crg.cd_sys.clk)

        # PCIe -------------------------------------------------------------------------------------
        self.submodules.pcie_phy = S7PCIEPHY(platform, platform.request("pcie_x4"),
            data_width = 128,
            bar0_size  = 0x20000)
        self.add_pcie(phy=self.pcie_phy, ndmas=4, dma_buffering_depth=1024, max_pending_requests=4)
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

        # Leds -------------------------------------------------------------------------------------
        self.submodules.leds = LedChaser(
         pads         = platform.request_all("user_led"),
         sys_clk_freq = sys_clk_freq)

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

    builder  = Builder(soc)
    builder.build(run=args.build)

    if args.driver:
        generate_litepcie_software(soc, os.path.join(builder.output_dir, "driver"))

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

if __name__ == "__main__":
    main()
