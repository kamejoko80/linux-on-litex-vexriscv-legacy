#!/usr/bin/env python3

import os

from migen import *

from litex.soc.interconnect import wishbone
from litex.soc.integration.soc_core import mem_decoder

from litex.soc.cores.spi_flash import SpiFlashSingle

from periphs.misc import *

# SoCCustom -----------------------------------------------------------------------------------------

def SoCICE40UP(soc_cls, **kwargs):
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
            "spiflash":     0x00000000,
            "rom":          0x00020000,
            "sram":         0x10000000,
            "csr":          0xf0000000,
        }

        def __init__(self, **kwargs):      
            soc_cls.__init__(self, cpu_type="vexriscv", cpu_reset_address=self.mem_map["rom"], cpu_variant="lite", **kwargs)

            # get platform object
            platform = self.platform 
            cpu_reset_address = self.mem_map["spiflash"] + platform.gateware_size 

            # SPI flash peripheral
            self.submodules.spiflash = SpiFlashSingle(platform.request("spiflash"),
                                                      dummy=platform.spiflash_read_dummy_bits,
                                                      div=platform.spiflash_clock_div,
                                                      endianness=self.cpu.endianness)
            self.add_constant("SPIFLASH_PAGE_SIZE", platform.spiflash_page_size)
            self.add_constant("SPIFLASH_SECTOR_SIZE", platform.spiflash_sector_size)
            self.add_csr("spiflash", 5, allow_user_defined=True)
            self.register_mem("spiflash", self.mem_map["spiflash"], self.spiflash.bus, size=platform.spiflash_total_size)

            bios_size = 0x10000
            self.add_constant("ROM_DISABLE", 1)
            self.add_memory_region("rom", cpu_reset_address, bios_size)
            self.flash_boot_address = self.mem_map["spiflash"] + platform.gateware_size+bios_size

            # We don't have a DRAM, so use the remaining SPI flash for user
            # program.
            self.add_memory_region("user_flash",
                self.flash_boot_address,
                # Leave a grace area- possible one-by-off bug in add_memory_region?
                # Possible fix: addr < origin + length - 1
                platform.spiflash_total_size - (self.flash_boot_address - self.mem_map["spiflash"]) - 0x100)

    return _SoCLinux(**kwargs)
