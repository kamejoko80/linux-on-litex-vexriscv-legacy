#!/usr/bin/env python3

import argparse

from migen import *
from migen.genlib.io import CRG
from migen.genlib.misc import timeline

from litex.build.generic_platform import *
from litex.build.sim import SimPlatform
from litex.build.sim.config import SimConfig

from litex.soc.interconnect.csr import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.interconnect import stream
from litex.soc.interconnect import wishbone
from litex.soc.cores import uart

from periphs.misc import *

class SimPins(Pins):
    def __init__(self, n=1):
        Pins.__init__(self, "s "*n)


_io = [
    ("sys_clk", 0, SimPins(1)),
    ("sys_rst", 0, SimPins(1)),
    ("serial", 0,
        Subsignal("source_valid", SimPins()),
        Subsignal("source_ready", SimPins()),
        Subsignal("source_data", SimPins(8)),

        Subsignal("sink_valid", SimPins()),
        Subsignal("sink_ready", SimPins()),
        Subsignal("sink_data", SimPins(8)),
    ),
    ("canif", 0,
        Subsignal("tx", SimPins()),
        Subsignal("rx", SimPins()),
        Subsignal("boo", SimPins()),
        Subsignal("irq", SimPins()),
        Subsignal("clkout", SimPins())
    ),
    ("spi", 0,
        Subsignal("sclk", SimPins()),
        Subsignal("miso", SimPins()),
        Subsignal("mosi", SimPins()),
        Subsignal("csn", SimPins()),
        Subsignal("irq", SimPins()),
    ),    
]

class Platform(SimPlatform):
    default_clk_name = "sys_clk"
    default_clk_period = 1000 # ~ 1MHz

    def __init__(self):
        SimPlatform.__init__(self, "SIM", _io)

    def do_finalize(self, fragment):
        pass


class Supervisor(Module, AutoCSR):
    def __init__(self):
        self._finish  = CSR()  # controlled from CPU
        self.finish = Signal() # controlled from logic
        self.sync += If(self._finish.re | self.finish, Finish())


class LinuxSoC(SoCCore):
    SoCCore.csr_map.update({
        "ctrl":       0,
        "uart":       2,
        "timer0":     3,
    })
    SoCCore.interrupt_map.update({
        "uart":       0,
        "timer0":     1,
    })
    SoCCore.mem_map = {
        "rom":          0x00000000,
        "sram":         0x10000000,
        "emulator_ram": 0x20000000,
        "main_ram":     0xC0000000,
        "csr":          0xf0000000,
    }

    def __init__(self, **kwargs):
        platform = Platform()
        sys_clk_freq = int(1e6)
        SoCCore.__init__(self, platform, clk_freq=sys_clk_freq,
            cpu_type="vexriscv", cpu_variant="linux",
            with_uart=False,
            integrated_rom_size=0xC000,
            integrated_main_ram_size=0x02000000, # 32MB
            **kwargs)

        # supervisor
        self.submodules.supervisor = Supervisor()
        self.add_csr("supervisor")

        # crg
        self.submodules.crg = CRG(platform.request("sys_clk"))

        # machine mode emulator ram
        emulator_rom = get_mem_data("emulator/emulator.bin", "little")
        self.submodules.emulator_ram = wishbone.SRAM(0x4000, init=emulator_rom)
        self.register_mem("emulator_ram", self.mem_map["emulator_ram"], self.emulator_ram.bus, 0x4000)
        self.add_constant("ROM_BOOT_ADDRESS",self.mem_map["emulator_ram"])

        # serial
        self.submodules.uart_phy = uart.RS232PHYModel(platform.request("serial"))
        self.submodules.uart = uart.UART(self.uart_phy)
        self.add_csr("uart", allow_user_defined=True)
        self.add_interrupt("uart", allow_user_defined=True)

        # Integrate Adder8
        self.submodules.adder8 = Adder8()
        self.add_csr("adder8", 10, allow_user_defined=True)

        # Integrate CAN
        # self.submodules.can_ctrl = can_ctrl = SJA1000(platform.request("canif", 0))
        # self.add_csr("can_ctrl", 11, allow_user_defined=True)
        # self.add_interrupt("can_ctrl", 6, allow_user_defined=True)
        # self.register_mem("can_ctrl", 0x30000000, can_ctrl.bus, 512)
        # can_ctrl.add_source(platform)
        # platform.add_verilog_include_path("periphs/verilog/can")

        # Integrate SPI master
        self.submodules.spi_master = spi_master = SpiMaster(self.platform.request("spi", 0))
        self.add_csr("spi_master", 11, allow_user_defined=True)
        self.add_interrupt("spi_master", 6, allow_user_defined=True)
        self.register_mem("spi_master", 0x30000000, spi_master.bus, 32)
        spi_master.add_source(self.platform)
        platform.add_verilog_include_path("periphs/verilog/spi")

def main():
    parser = argparse.ArgumentParser(description="Linux on LiteX-VexRiscv Simulation")
    parser.add_argument("--trace", action="store_true", help="enable VCD tracing")
    args = parser.parse_args()

    sim_config = SimConfig(default_clk="sys_clk")
    sim_config.add_module("serial2console", "serial")

    soc = LinuxSoC()
    builder = Builder(soc, output_dir="build", csr_csv="csr.csv")
    builder.build(sim_config=sim_config, trace=args.trace)


if __name__ == "__main__":
    main()
