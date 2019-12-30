#!/usr/bin/env python3

import os

from migen import *

from litex.soc.interconnect import wishbone
from litex.soc.integration.soc_core import mem_decoder

from litex.soc.cores.spi_flash import SpiFlash

from periphs.misc import *

# SoCPicorv32 -----------------------------------------------------------------------------------------

def SoCPicorv32(soc_cls, **kwargs):
    class _SoCLinux(soc_cls):
        soc_cls.csr_map.update({
            "ctrl":       0,
            "uart":       2,
            "timer0":     3,
        })
        soc_cls.interrupt_map.update({
            "uart":       3,
            "timer0":     4,
        })
        soc_cls.mem_map = {
            "rom":          0x00000000,
            "sram":         0x10000000,
            "emulator_ram": 0x20000000,
            "ethmac":       0x30000000,
            "spiflash":     0x50000000,
            "main_ram":     0xc0000000,
            "csr":          0xf0000000,
        }

        def __init__(self, **kwargs):
            soc_cls.__init__(self, cpu_type="picorv32", cpu_variant="minimal", **kwargs)

            # Integrate int module
            self.submodules.gpio_isr = GpioISR(self.platform.request('key', 0), rissing_edge_detect=False)
            self.add_csr("gpio_isr", 10, allow_user_defined=True)
            self.add_interrupt("gpio_isr", 5, allow_user_defined=True)

    return _SoCLinux(**kwargs)
