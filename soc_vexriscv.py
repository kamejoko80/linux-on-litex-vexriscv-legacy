#!/usr/bin/env python3

import os

from migen import *

from litex.soc.interconnect import wishbone
from litex.soc.integration.soc_core import mem_decoder

from litex.soc.cores.spi_flash import SpiFlash

from periphs.misc import *
from periphs.accel import *

# SoCVexRiscv -----------------------------------------------------------------------------------------

def SoCVexRiscv(soc_cls, **kwargs):
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

            # Integrate int module
            # self.submodules.gpio_isr = GpioISR(self.platform.request('key', 0), rissing_edge_detect=False)
            # self.add_csr("gpio_isr", 10, allow_user_defined=True)
            # self.add_interrupt("gpio_isr", 5, allow_user_defined=True)

            # Integrate Adder8
            # self.submodules.adder8 = Adder8()
            # self.add_csr("adder8", 11, allow_user_defined=True)

            # Integrate my uart
            # self.submodules.my_uart = my_uart = MyUart(self.platform.request("MyUart", 0), self.platform.request("led0", 0))
            # my_uart.add_source(self.platform)
            # self.add_csr("my_uart", 12, allow_user_defined=True)

            # Integrate simple wishbone gpio
            # self.submodules.wb_gpio = wb_gpio = WbGpio(self.platform.request("led0", 0))
            # self.register_mem("wb_gpio", 0x30000000, wb_gpio.bus, 1000)

            # Integrate CAN
            # self.submodules.can_ctrl = can_ctrl = SJA1000(self.platform.request("canif", 0))
            # self.add_csr("can_ctrl", 13, allow_user_defined=True)
            # self.add_interrupt("can_ctrl", 6, allow_user_defined=True)
            # self.register_mem("can_ctrl", 0x30000000, can_ctrl.bus, 1000)
            # can_ctrl.add_source(self.platform)

            # Integrate wishbone to avalon bridge
            # self.submodules.w2a_bridge = w2a_bridge = W2ABridge()
            # self.register_mem("w2a_bridge", 0x30000000, w2a_bridge.bus, 1000)
            # w2a_bridge.add_source(self.platform)

            # Integrate SPI master
            self.submodules.spi_master = spi_master = SpiMaster(self.platform.request("spi", 0))
            self.add_csr("spi_master", 10, allow_user_defined=True)
            self.add_interrupt("spi_master", 6, allow_user_defined=True)
            self.register_mem("spi_master", 0x30000000, spi_master.bus, 32)
            spi_master.add_source(self.platform)

    return _SoCLinux(**kwargs)
