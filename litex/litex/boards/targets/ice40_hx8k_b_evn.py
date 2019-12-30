#!/usr/bin/env python3

import argparse

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.boards.platforms import ice40_hx8k_b_evn

from litex.soc.cores.spi_flash import SpiFlash
from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

# CRG ----------------------------------------------------------------------------------------------

class _CRG(Module):
    def __init__(self, platform):
        clk12 = platform.request("clk12")

        self.clock_domains.cd_sys = ClockDomain()
        self.reset = Signal()

        # Fout = Fin x (DIVF + 1) / (2^DIVQ x (DIVR + 1))
        self.specials += \
            Instance("SB_PLL40_CORE",
                p_FEEDBACK_PATH="SIMPLE",
                p_PLLOUT_SELECT="GENCLK",
                p_DIVR=0,    # 0
                p_DIVF=7,    # 7
                p_DIVQ=1,    # 1
                p_FILTER_RANGE=0b010,
                i_RESETB=1,
                i_BYPASS=0,
                i_REFERENCECLK=clk12,
                o_PLLOUTCORE=self.cd_sys.clk, # 48MHz
            )

        # POR reset logic- POR generated from sys clk, POR logic feeds sys clk
        # reset.
        self.clock_domains.cd_por = ClockDomain()
        reset_delay = Signal(12, reset=4095)
        self.comb += [
            self.cd_por.clk.eq(self.cd_sys.clk),
        ]

        self.sync += [
            self.cd_sys.rst.eq(reset_delay != 0)
        ]

        self.sync.por += \
            If(reset_delay != 0,
                reset_delay.eq(reset_delay - 1)
            )
        self.specials += AsyncResetSynchronizer(self.cd_por, self.reset)

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    def __init__(self, device="ice40-hx8k", toolchain="icestorm", **kwargs):
        platform = ice40_hx8k_b_evn.Platform(device=device, toolchain=toolchain)
        sys_clk_freq = int(48e6)
        SoCCore.__init__(self, platform, clk_freq=sys_clk_freq,
                         with_uart=True,
                         integrated_rom_size=0,
                         integrated_sram_size=4096,
                         integrated_main_ram_size=0,
                         **kwargs)

        self.submodules.crg = _CRG(platform)

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteX SoC on ICE40-HX8K")
    parser.add_argument("--gateware-toolchain", dest="toolchain", default="icestorm",
        help='gateware toolchain to use, icestorm')
    parser.add_argument("--device", dest="device", default="ice40-hx8k",
        help='FPGA device, ICE40-HX8K can be populated with ice40-hx8k')
    builder_args(parser)
    args = parser.parse_args()

    soc = BaseSoC(device=args.device, toolchain=args.toolchain, **soc_sdram_argdict(args))
    builder = Builder(soc, **builder_argdict(args))
    builder.build()

if __name__ == "__main__":
    main()
