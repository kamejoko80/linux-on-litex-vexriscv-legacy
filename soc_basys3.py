#!/usr/bin/env python3

import os

from migen import *

from litex.soc.interconnect import wishbone
from litex.soc.integration.soc_core import mem_decoder

from litex.soc.cores.spi_flash import SpiFlash

from periphs.misc import *
from periphs.accel import *

# SoCAE4GX -----------------------------------------------------------------------------------------

def SoCBASYS3(soc_cls, **kwargs):
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
            "csr":          0xf0000000,
        }

        def __init__(self, **kwargs):
            soc_cls.__init__(self, cpu_type="vexriscv", cpu_variant="lite", **kwargs)

            # Integrate SPI master
            self.submodules.spi_master = spi_master = SpiMaster(self.platform.request("spi", 0))
            self.add_csr("spi_master", 10, allow_user_defined=True)
            self.add_interrupt("spi_master", 6, allow_user_defined=True)
            self.register_mem("spi_master", 0x30000000, spi_master.bus, 32)
            spi_master.add_source(self.platform)

            # Custom accel simulator IP core
            self.submodules.accel = accel = AccelCore(freq=200000000, baud=115200, pads=self.platform.request("spi_slave", 0))
            self.add_csr("accel", 11, allow_user_defined=True)
            self.add_interrupt("accel", 7, allow_user_defined=True)            
            
    return _SoCLinux(**kwargs)
