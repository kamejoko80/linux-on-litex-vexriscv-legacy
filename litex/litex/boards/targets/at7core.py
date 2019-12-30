#!/usr/bin/env python3

import argparse

from migen import *

from litex.boards.platforms import at7core

from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

# CRG ----------------------------------------------------------------------------------------------

class _CRG(Module):
    def __init__(self, platform, sys_clk_freq):
        clk50 = platform.request("clk50")

        self.clock_domains.cd_sys = ClockDomain()
        self.clock_domains.cd_clkout = ClockDomain()
        self.reset = Signal()

        # POR reset logic- POR generated from sys clk, POR logic feeds sys clk
        # reset.
        self.clock_domains.cd_por = ClockDomain()
        reset_delay = Signal(12, reset=4095)
        self.comb += [
            self.cd_sys.clk.eq(self.cd_clkout.clk),
            self.cd_por.clk.eq(self.cd_clkout.clk),
        ]

        self.sync += [
            self.cd_sys.rst.eq(reset_delay != 0)
        ]

        self.sync.por += \
            If(reset_delay != 0,
                reset_delay.eq(reset_delay - 1)
            )

        self.specials += AsyncResetSynchronizer(self.cd_por, self.reset)

        self.submodules.pll = pll = S7PLL(speedgrade=-1)
        pll.register_clkin(clk50, 50e6)
        pll.create_clkout(self.cd_clkout, sys_clk_freq)

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    def __init__(self, sys_clk_freq=int(200e6), **kwargs):
        platform = at7core.Platform()
        SoCCore.__init__(self, platform, clk_freq=sys_clk_freq,
                         with_uart=True,
                         integrated_rom_size=0xA000,   #
                         integrated_sram_size=0x2000,  #
                         integrated_main_ram_size=0,
                         **kwargs)

        self.submodules.crg = _CRG(platform, sys_clk_freq)


# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteX SoC on Artix-7 XC7A100T Core Board")
    builder_args(parser)
    soc_sdram_args(parser)
    args = parser.parse_args()

    soc = BaseSoC(**soc_sdram_argdict(args))
    builder = Builder(soc, **builder_argdict(args))
    builder.build()

if __name__ == "__main__":
    main()
