#!/usr/bin/env python3

#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2019 Arnaud Durand <arnaud.durand@unifr.ch>
# SPDX-License-Identifier: BSD-2-Clause

import os
import argparse

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.build.io import DDROutput

from litex_boards.platforms import ecp5_evn

from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
#from litex.soc.cores.led import LedChaser

from litedram import modules as litedram_modules
from litedram.phy import GENSDRPHY, HalfRateGENSDRPHY

from liteeth.phy.ecp5sgmii import LiteEthPHYSGMII

# CRG ----------------------------------------------------------------------------------------------

class _CRG(Module):
    def __init__(self, platform, sys_clk_freq, x5_clk_freq, sdram_rate="1:1"):
        self.rst = Signal()
        self.clock_domains.cd_sys = ClockDomain()
        if sdram_rate == "1:2":
            self.clock_domains.cd_sys2x    = ClockDomain()
            self.clock_domains.cd_sys2x_ps = ClockDomain(reset_less=True)
        else:
            self.clock_domains.cd_sys_ps = ClockDomain(reset_less=True)

        # # #

        # clk / rst
        clk = clk12 = platform.request("clk12")
        rst_n = platform.request("rst_n")
        if x5_clk_freq is not None:
            clk = clk50 = platform.request("ext_clk50")
            self.comb += platform.request("ext_clk50_en").eq(1)
            platform.add_period_constraint(clk50, 1e9/x5_clk_freq)

        # pll
        self.submodules.pll = pll = ECP5PLL()
        self.comb += pll.reset.eq(~rst_n | self.rst)
        pll.register_clkin(clk, x5_clk_freq or 12e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)
        if sdram_rate == "1:2":
            pll.create_clkout(self.cd_sys2x,    2*sys_clk_freq)
            pll.create_clkout(self.cd_sys2x_ps, 2*sys_clk_freq, phase=180) # Idealy 90° but needs to be increased.
        else:
           pll.create_clkout(self.cd_sys_ps, sys_clk_freq, phase=90)

        # SDRAM clock
        sdram_clk = ClockSignal("sys2x_ps" if sdram_rate == "1:2" else "sys_ps")
        self.specials += DDROutput(1, 0, platform.request("sdram_clock"), sdram_clk)

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    def __init__(self, sys_clk_freq=int(50e6), x5_clk_freq=None, toolchain="trellis", sdram_module_cls="AS4C32M16", sdram_rate="1:1", with_ethernet=False, with_etherbone=False, local_ip="", remote_ip="", **kwargs):
        platform = ecp5_evn.Platform(toolchain=toolchain)

        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, sys_clk_freq,
            ident          = "LiteX SoC on ECP5 Evaluation Board",
            ident_version  = True,
            **kwargs)

        # CRG --------------------------------------------------------------------------------------
        crg = _CRG(platform, sys_clk_freq, x5_clk_freq, sdram_rate=sdram_rate)
        self.submodules.crg = crg

        # SDR SDRAM --------------------------------------------------------------------------------
        if not self.integrated_main_ram_size:
            sdrphy_cls = HalfRateGENSDRPHY if sdram_rate == "1:2" else GENSDRPHY
            self.submodules.sdrphy = sdrphy_cls(platform.request("sdram"), sys_clk_freq)
            self.add_sdram("sdram",
                phy                     = self.sdrphy,
                module                  = getattr(litedram_modules, sdram_module_cls)(sys_clk_freq, sdram_rate),
                origin                  = self.mem_map["main_ram"],
                size                    = kwargs.get("max_sdram_size", 0x40000000),
                l2_cache_size           = kwargs.get("l2_size", 8192),
                l2_cache_min_data_width = kwargs.get("min_l2_data_width", 128),
                l2_cache_reverse        = True
            )

        # Ethernet / Etherbone ---------------------------------------------------------------------
        if with_ethernet or with_etherbone:
            self.submodules.ethphy = ethphy = LiteEthPHYSGMII(platform)
            self.add_csr("ethphy")
            if with_ethernet:
                self.add_ethernet(phy=self.ethphy)
            if with_etherbone:
                self.add_etherbone(phy=self.ethphy)

        if local_ip:
            local_ip = local_ip.split(".")
            self.add_constant("LOCALIP1", int(local_ip[0]))
            self.add_constant("LOCALIP2", int(local_ip[1]))
            self.add_constant("LOCALIP3", int(local_ip[2]))
            self.add_constant("LOCALIP4", int(local_ip[3]))

        if remote_ip:
            remote_ip = remote_ip.split(".")
            self.add_constant("REMOTEIP1", int(remote_ip[0]))
            self.add_constant("REMOTEIP2", int(remote_ip[1]))
            self.add_constant("REMOTEIP3", int(remote_ip[2]))
            self.add_constant("REMOTEIP4", int(remote_ip[3]))

        # Leds -------------------------------------------------------------------------------------
        #self.submodules.leds = LedChaser(
        #    pads         = platform.request_all("user_led"),
        #    sys_clk_freq = sys_clk_freq)
        #self.add_csr("leds")

        # Bridge
        self.add_uartbone(name="uart_bridge")

        # Analyzer ---------------------------------------------------------------------------------
        from litescope import LiteScopeAnalyzer
        self.submodules.analyzer = LiteScopeAnalyzer([
            #ethphy.pcs.init.tx_rst,
            #ethphy.pcs.init.rx_rst,
            #ethphy.pcs.init.pcs_rst,
            #ethphy.pcs.init.ready,
            #ethphy.pcs.init.tx_lol,
            #ethphy.pcs.init.rx_lol,
            #ethphy.pcs.init.rx_los,
            #ethphy.pcs.rx_enable,
            #ethphy.pcs.rx_ready,
            #ethphy.pcs.rx_idle,
            #ethphy.pcs.rx_align,
            #ethphy.pcs.rx_data,
            #ethphy.pcs.rx_k,
            #ethphy.pcs.tx_enable,
            #ethphy.pcs.tx_ready,
            #ethphy.pcs.tx_idle,
            #ethphy.pcs.tx_data,
            #ethphy.pcs.tx_k,
            #ethphy.pcs.source,
            #ethphy.pcs.sink,
            #ethphy.fsm,
            #ethphy.abi.i,
            #ethphy.abi.o,
            #ethphy.ack.i,
            #ethphy.ack.o,
            #ethphy.ci.i,
            #ethphy.ci.o,
            ethphy.tx.fsm,
            ethphy.tx.sink,
            #ethphy.rx.fsm,
            ethphy.rx.source,
            #ethphy.rx.sop,
            #ethphy.rx.soe,
            #ethphy.button,
            #ethphy.tx.config_stb,
            #ethphy.tx.config_reg,
            #ethphy.rx.seen_config_reg,
            #ethphy.rx.seen_valid_ci,
            #ethphy.rx.config_reg,
            #ethphy.link_partner_adv_ability,
            #ethphy.sgmii_remover.source,
            #ethphy.sgmii_remover.sink,
            #ethphy.sgmii_inserter.fsm,
            #ethphy.sgmii_inserter.source,
            #ethphy.sgmii_inserter.sink,
        ], clock_domain = "eth_rx", depth=512)
        self.add_csr("analyzer")


# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteX SoC on ECP5 Evaluation Board")
    parser.add_argument("--build",        action="store_true", help="Build bitstream")
    parser.add_argument("--load",         action="store_true", help="Load bitstream")
    parser.add_argument("--toolchain",    default="trellis",   help="FPGA toolchain: trellis (default) or diamond")
    parser.add_argument("--sys-clk-freq", default=60e6,        help="System clock frequency (default: 60MHz)")
    parser.add_argument("--x5-clk-freq",  type=int,            help="Use X5 oscillator as system clock at the specified frequency")
    parser.add_argument("--sdram-module",    default="AS4C32M16", help="SDRAM module: AS4C32M16(default) or AS4C16M16")
    parser.add_argument("--sdram-rate",      default="1:1",         help="SDRAM Rate: 1:1 Full Rate (default), 1:2 Half Rate")
    ethopts = parser.add_mutually_exclusive_group()
    ethopts.add_argument("--with-ethernet",   action="store_true",      help="Enable Ethernet support")
    ethopts.add_argument("--with-etherbone",  action="store_true",      help="Enable Etherbone support")
    parser.add_argument("--remote-ip",        default="192.168.1.100",  help="Remote IP address of TFTP server")
    parser.add_argument("--local-ip",         default="192.168.1.50",   help="Local IP address")
    builder_args(parser)
    soc_core_args(parser)
    args = parser.parse_args()

    soc = BaseSoC(toolchain=args.toolchain,
        sys_clk_freq     = int(float(args.sys_clk_freq)),
        x5_clk_freq      = args.x5_clk_freq,
        sdram_module_cls = args.sdram_module,
        sdram_rate       = args.sdram_rate,
        with_ethernet    = args.with_ethernet,
        with_etherbone   = args.with_etherbone,
        local_ip         = args.local_ip,
        remote_ip        = args.remote_ip,
        **soc_core_argdict(args))
    builder = Builder(soc, **builder_argdict(args))
    builder.build(run=args.build)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(os.path.join(builder.gateware_dir, soc.build_name + ".svf"))

if __name__ == "__main__":
    main()
