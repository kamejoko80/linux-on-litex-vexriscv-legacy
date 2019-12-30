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
            integrated_main_ram_init=get_mem_data({
                "buildroot/Image":         "0x00000000",
                "buildroot/rootfs.cpio":   "0x00800000",
                "buildroot/rv32.dtb":      "0x01000000"
                }, "little"),
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


def main():
    parser = argparse.ArgumentParser(description="Linux on LiteX-VexRiscv Simulation")
    parser.add_argument("--trace", action="store_true", help="enable VCD tracing")
    args = parser.parse_args()

    sim_config = SimConfig(default_clk="sys_clk")
    sim_config.add_module("serial2console", "serial")

    print("Compile board device tree...")
    os.system("dtc -O dtb -o buildroot/rv32.dtb buildroot/board/litex_vexriscv/litex_vexriscv.dts")

    soc = LinuxSoC()
    builder = Builder(soc, output_dir="build", csr_csv="csr.csv")
    builder.build(sim_config=sim_config, trace=args.trace)


if __name__ == "__main__":
    main()
