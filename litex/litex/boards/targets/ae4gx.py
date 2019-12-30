#!/usr/bin/env python3

import argparse

from migen import *

from litex.boards.platforms import ae4gx

from litex.soc.integration.soc_sdram import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from litedram.modules import IS42S16160
from litedram.phy import GENSDRPHY

# CRG ----------------------------------------------------------------------------------------------
#                  _________
#                 |         |
#    clk100 --|-->| ALT PLL |-----> sys_clk (200MHz)
#             |   |_________|
#             |
#             |-------------------> por_clk (100MHz)
#
#----------------------------------------------------------------------------------------------------

class _CRG(Module):
    def __init__(self, platform):
        self.clock_domains.cd_sys = ClockDomain()
        self.clock_domains.cd_por = ClockDomain(reset_less=True)

        # clock source request
        clk100 = platform.request("clk100")

        ###
        self.cd_sys.clk.attr.add("keep")
        self.cd_por.clk.attr.add("keep")

        # power on rst
        rst_n = Signal()
        self.sync.por += rst_n.eq(1)
        self.comb += [
            self.cd_sys.clk.eq(clk100),
            self.cd_por.clk.eq(clk100),
        ]

        self.sync += [
            self.cd_sys.rst.eq(~rst_n),
        ]

        # sys_clk pll setting (target 50MHz)
        # self.specials += \
            # Instance("ALTPLL",
                # p_BANDWIDTH_TYPE="AUTO",
                # p_CLK0_DIVIDE_BY=2,
                # p_CLK0_DUTY_CYCLE=50e0,
                # p_CLK0_MULTIPLY_BY=1,
                # p_CLK0_PHASE_SHIFT="0",
                # p_COMPENSATE_CLOCK="CLK0",
                # p_INCLK0_INPUT_FREQUENCY=20000e0,
                # p_OPERATION_MODE="NORMAL",
                # i_INCLK=clk100,
                # o_CLK=self.cd_sys.clk,
                # i_ARESET=~rst_n,
                # i_CLKENA=0x3f,
                # i_EXTCLKENA=0xf,
                # i_FBIN=1,
                # i_PFDENA=1,
                # i_PLLENA=1,
            # )

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    def __init__(self, sys_clk_freq=int(100e6), **kwargs):
        assert sys_clk_freq == int(100e6)
        platform = ae4gx.Platform()
        SoCCore.__init__(self, platform, clk_freq=sys_clk_freq,
                         with_uart=True,
                         integrated_rom_size=0x20000,  # 128KB
                         integrated_sram_size=0x10000, # 64KB
                         integrated_main_ram_size=0,
                         **kwargs)

        self.submodules.crg = _CRG(platform)

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteX SoC on A-E4GX V3.0")
    builder_args(parser)
    soc_sdram_args(parser)
    args = parser.parse_args()

    soc = BaseSoC(**soc_sdram_argdict(args))
    builder = Builder(soc, **builder_argdict(args))
    builder.build()


if __name__ == "__main__":
    main()
