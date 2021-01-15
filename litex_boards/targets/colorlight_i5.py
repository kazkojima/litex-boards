#!/usr/bin/env python3

#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2020 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

# Build/Use ----------------------------------------------------------------------------------------
#
# 1) SoC with regular UART and optional Ethernet connected to the CPU:
# Connect a USB/UART to J19: TX of the FPGA is DATA_LED-, RX of the FPGA is KEY+.
# ./colorlight_i5.py --revision=7.0 (--with-ethernet to add Ethernet capability)
# ./colorlight_i5.py --load
# You should see the LiteX BIOS and be able to interact with it.
#

import os
import argparse
import sys

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.build.io import DDROutput

from litex_boards.platforms import colorlight_i5
from litex.build.tools import write_to_file

from litex.build.lattice.trellis import trellis_args, trellis_argdict

from litex.soc.cores.clock import *
from litex.soc.cores.spi_flash import SpiFlash
from litex.soc.cores.spi import SPIMaster
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from litex.soc.cores.led import LedChaser

from litedram.modules import M12L64322A
from litedram.phy import GENSDRPHY, HalfRateGENSDRPHY

from liteeth.phy.ecp5rgmii import LiteEthPHYRGMII

# CRG ----------------------------------------------------------------------------------------------

class _CRG(Module):
    def __init__(self, platform, sys_clk_freq, use_internal_osc=False, with_usb_pll=False, sdram_rate="1:1"):
        self.rst = Signal()
        self.clock_domains.cd_sys    = ClockDomain()
        if sdram_rate == "1:2":
            self.clock_domains.cd_sys2x    = ClockDomain()
            self.clock_domains.cd_sys2x_ps = ClockDomain(reset_less=True)
        else:
            self.clock_domains.cd_sys_ps = ClockDomain(reset_less=True)

        # # #

        # Clk / Rst
        if not use_internal_osc:
            clk = platform.request("clk25")
            clk_freq = 25e6
        else:
            clk = Signal()
            div = 5
            self.specials += Instance("OSCG",
                                p_DIV = div,
                                o_OSC = clk)
            clk_freq = 310e6/div

        rst_n = platform.request("cpu_reset_n")

        # PLL
        self.submodules.pll = pll = ECP5PLL()
        self.comb += pll.reset.eq(~rst_n | self.rst)
        pll.register_clkin(clk, clk_freq)
        pll.create_clkout(self.cd_sys,    sys_clk_freq)
        if sdram_rate == "1:2":
            pll.create_clkout(self.cd_sys2x,    2*sys_clk_freq)
            pll.create_clkout(self.cd_sys2x_ps, 2*sys_clk_freq, phase=180) # Idealy 90° but needs to be increased.
        else:
           pll.create_clkout(self.cd_sys_ps, sys_clk_freq, phase=180) # Idealy 90° but needs to be increased.

        # USB PLL
        if with_usb_pll:
            self.submodules.usb_pll = usb_pll = ECP5PLL()
            self.comb += usb_pll.reset.eq(~rst_n | self.rst)
            usb_pll.register_clkin(clk, clk_freq)
            self.clock_domains.cd_usb_12 = ClockDomain()
            self.clock_domains.cd_usb_48 = ClockDomain()
            usb_pll.create_clkout(self.cd_usb_12, 12e6, margin=0)
            usb_pll.create_clkout(self.cd_usb_48, 48e6, margin=0)

        # SDRAM clock
        sdram_clk = ClockSignal("sys2x_ps" if sdram_rate == "1:2" else "sys_ps")
        self.specials += DDROutput(1, 0, platform.request("sdram_clock"), sdram_clk)

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    mem_map = {**SoCCore.mem_map, **{"spiflash": 0xd0000000}}
    def __init__(self, board="i5", revision="7.0", sys_clk_freq=60e6, with_ethernet=False, with_etherbone=False, eth_phy=0, use_internal_osc=False, sdram_rate="1:1", **kwargs):
        board = board.lower()
        assert board in ["i5"]
        if board == "i5":
            platform = colorlight_i5.Platform(revision=revision)

        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, int(sys_clk_freq),
            ident          = "LiteX SoC on Colorlight " + board.upper(),
            ident_version  = True,
            **kwargs)

        # CRG --------------------------------------------------------------------------------------
        with_usb_pll = kwargs.get("uart_name", None) == "usb_acm"
        self.submodules.crg = _CRG(platform, sys_clk_freq, use_internal_osc=use_internal_osc, with_usb_pll=with_usb_pll, sdram_rate=sdram_rate)

        # Leds -------------------------------------------------------------------------------------
        ledn = platform.request_all("user_led_n")
        self.submodules.leds = LedChaser(pads=ledn, sys_clk_freq=sys_clk_freq)
        self.add_csr("leds")

        # SPI Flash --------------------------------------------------------------------------------
        self.add_spi_flash(mode="1x", dummy_cycles=8)
        self.add_constant("SPIFLASH_PAGE_SIZE", 256)
        self.add_constant("SPIFLASH_SECTOR_SIZE", 4096)

        # SDR SDRAM --------------------------------------------------------------------------------
        if not self.integrated_main_ram_size:
            sdrphy_cls = HalfRateGENSDRPHY if sdram_rate == "1:2" else GENSDRPHY
            self.submodules.sdrphy = sdrphy_cls(platform.request("sdram"))
            # if board == "i5" and revision == "7.0":
            sdram_cls  = M12L64322A # compat with EM638325-6H
            self.add_sdram("sdram",
                phy                     = self.sdrphy,
                module                  = sdram_cls(sys_clk_freq, sdram_rate),
                origin                  = self.mem_map["main_ram"],
                size                    = kwargs.get("max_sdram_size", 0x40000000),
                l2_cache_size           = kwargs.get("l2_size", 8192),
                l2_cache_min_data_width = kwargs.get("min_l2_data_width", 128),
                l2_cache_reverse        = True
            )

        # Ethernet / Etherbone ---------------------------------------------------------------------
        if with_ethernet or with_etherbone:
            self.submodules.ethphy = LiteEthPHYRGMII(
                clock_pads = self.platform.request("eth_clocks", eth_phy),
                pads       = self.platform.request("eth", eth_phy))
            self.add_csr("ethphy")
            self.add_constant("TARGET_BIOS_INIT", 1)
            if with_ethernet:
                self.add_ethernet(phy=self.ethphy)
            if with_etherbone:
                self.add_etherbone(phy=self.ethphy)

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteX SoC on Colorlight i5")
    parser.add_argument("--build",            action="store_true",      help="Build bitstream")
    parser.add_argument("--load",             action="store_true",      help="Load bitstream")
    parser.add_argument("--board",            default="i5",         help="Board type: i5 (default)")
    parser.add_argument("--revision",         default="7.0", type=str,  help="Board revision: 7.0 (default)")
    parser.add_argument("--sys-clk-freq",     default=60e6,             help="System clock frequency (default: 60MHz)")
    parser.add_argument("--with-ethernet",    action="store_true",      help="Enable Ethernet support")
    parser.add_argument("--with-etherbone",   action="store_true",      help="Enable Etherbone support")
    parser.add_argument("--with-spi-sdcard",  action="store_true",	help="Enable SPI-mode SDCard support")
    parser.add_argument("--with-sdcard",      action="store_true",	help="Enable SDCard support")
    parser.add_argument("--with-spi",	      action="store_true",      help="SPI support")
    parser.add_argument("--eth-phy",          default=0, type=int,      help="Ethernet PHY: 0 (default) or 1")
    parser.add_argument("--use-internal-osc", action="store_true",      help="Use internal oscillator")
    parser.add_argument("--sdram-rate",       default="1:1",            help="SDRAM Rate: 1:1 Full Rate (default), 1:2 Half Rate")
    parser.add_argument("--l2-size",          default=8192, type=int,   help="L2 cache size")
    builder_args(parser)
    soc_core_args(parser)
    trellis_args(parser)
    args = parser.parse_args()

    assert not (args.with_ethernet and args.with_etherbone)
    soc = BaseSoC(board=args.board, revision=args.revision,
        sys_clk_freq     = int(float(args.sys_clk_freq)),
        with_ethernet    = args.with_ethernet,
        with_etherbone   = args.with_etherbone,
        eth_phy          = args.eth_phy,
        use_internal_osc = args.use_internal_osc,
        sdram_rate       = args.sdram_rate,
        l2_size		 = args.l2_size,
        **soc_core_argdict(args)
    )
    assert not (args.with_spi_sdcard and args.with_sdcard)
    soc.platform.add_extension(colorlight_i5._sdcard_pmod_io)
    if args.with_spi_sdcard:
        soc.add_spi_sdcard()
    if args.with_sdcard:
        soc.add_sdcard()

    soc.platform.add_extension(colorlight_i5._spi_pmod_io)
    if args.with_spi:
        spi_pads = soc.platform.request("spi")
        soc.submodules.spi = SPIMaster(spi_pads, 32, soc.clk_freq, 1000000, mode="aligned")
        soc.add_csr("spi")

    builder = Builder(soc, **builder_argdict(args))
    if args.with_ethernet:
        os.makedirs(os.path.join(builder.software_dir, "include/generated"),
                    exist_ok=True)
        write_to_file(
            os.path.join(builder.software_dir, "include/generated", "target.h"),
            "// Colorlight i5 needs this to disable TX data to clock delay.\n"
	    "#define TARGET_BIOS_INIT_FUNC() mdio_write(0, 0x1c, 0x8c00)")

    builder.build(**trellis_argdict(args), run=args.build)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(os.path.join(builder.gateware_dir, soc.build_name + ".svf"))

if __name__ == "__main__":
    main()
