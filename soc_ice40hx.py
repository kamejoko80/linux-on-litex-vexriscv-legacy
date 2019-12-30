#!/usr/bin/env python3

import os

from migen import *

from litex.soc.interconnect import wishbone
from litex.soc.integration.soc_core import mem_decoder

from litex.soc.cores.spi_flash import SpiFlashSingle

from periphs.misc import *
from periphs.accel import *

# SoCCustom -----------------------------------------------------------------------------------------

def SoCICE40HX(soc_cls, **kwargs):
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
            "rom":          0x00030000,
            "sram":         0x10000000,
            "csr":          0xf0000000,
        }

        def __init__(self, **kwargs):
            soc_cls.__init__(self, cpu_type="vexriscv", cpu_reset_address=self.mem_map["rom"], cpu_variant="min", **kwargs)

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

            # Integrate SPI master
            self.submodules.spi_master = spi_master = SpiMaster(self.platform.request("spi", 0))
            self.add_csr("spi_master", 10, allow_user_defined=True)
            #self.add_interrupt("spi_master", 6, allow_user_defined=True)
            self.register_mem("spi_master", 0x30000000, spi_master.bus, 32)
            spi_master.add_source(self.platform)

            # Custom accel simulator IP core
            self.submodules.accel = accel = AccelCore(freq=48000000, baud=115200, pads=self.platform.request("spi_slave", 0))
            self.add_csr("accel", 11, allow_user_defined=True)
            self.add_interrupt("accel", 7, allow_user_defined=True)

            # Integrate SPI SDC master
            #self.submodules.spi_sdc = spi_sdc = SpiMaster(self.platform.request("spi_sdc", 0))
            #self.add_csr("spi_sdc", 12, allow_user_defined=True)
            #self.add_interrupt("spi_sdc", 8, allow_user_defined=True)
            #self.register_mem("spi_sdc", 0x40000000, spi_sdc.bus, 32)
            #spi_sdc.add_source(self.platform)

            # Integrate int module
            #self.submodules.gpio_isr = GpioISR(self.platform.request("gpio_irq", 0), rissing_edge_detect=False)
            #self.add_csr("gpio_isr", 13, allow_user_defined=True)
            #self.add_interrupt("gpio_isr", 9, allow_user_defined=True)

    return _SoCLinux(**kwargs)
