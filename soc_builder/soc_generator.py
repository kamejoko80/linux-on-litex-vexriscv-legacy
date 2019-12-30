#!/usr/bin/env python3
import os
import sys
import math
import struct

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.build.generic_platform import *
from litex.build.xilinx import XilinxPlatform

from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *

from litex.soc.integration.builder import *
from litex.soc.interconnect import csr_bus
from litex.soc.cores.uart import *

sys.path.append('../')

from periphs.misc import *
from periphs.accel import *

def get_common_ios():
    return [
        # clk / rst
        ("clk", 0, Pins(1)),
        ("rst", 0, Pins(1)),

        # serial
        ("serial", 0,
            Subsignal("tx", Pins(1)),
            Subsignal("rx", Pins(1))
        ),

        ("gpio_irq", 0, Pins("1")),
        ("user_led", 15, Pins("1")),

        # SPI master
        ("spi", 0,
            Subsignal("sclk", Pins(1)),
            Subsignal("miso", Pins(1)),
            Subsignal("mosi", Pins(1)),
            Subsignal("csn",  Pins(1)),
            Subsignal("irq",  Pins(1)),
        ),

        # SPI slave, accel simulator
        ("spi_slave", 0,
            # SPI slave part
            Subsignal("sck",   Pins(1)),
            Subsignal("miso",  Pins(1)),
            Subsignal("mosi",  Pins(1)),
            Subsignal("csn",   Pins(1)),
            Subsignal("int1",  Pins(1)),
            Subsignal("int2",  Pins(1)),
            #Subsignal("irq",  Pins(1)),
            Subsignal("led0",  Pins(1)),
            Subsignal("led1",  Pins(1)),
            Subsignal("led2",  Pins(1)),
            Subsignal("led3",  Pins(1)),
            Subsignal("led4",  Pins(1)),
            Subsignal("led5",  Pins(1)),
            Subsignal("led6",  Pins(1)),
            Subsignal("led15", Pins(1)),
            # UART part
            Subsignal("tx",    Pins(1)),
            Subsignal("rx",    Pins(1)),
        ),

        # MailBox sender interface
        ("mbx_snd", 0,
            Subsignal("dout_r", Pins("0 1 2 3 4 5 6 7")),
            Subsignal("dout_re", Pins(1)),
            Subsignal("int_r", Pins(1)),
            Subsignal("int_re",  Pins(1)),
        ),

        # MailBox sender interface
        ("mbx_rcv", 0,
            Subsignal("din_status", Pins("0 1 2 3 4 5 6 7")),
            Subsignal("len_status", Pins("0 1 2 3 4 5 6 7")),
            Subsignal("rd_r", Pins(1)),
            Subsignal("rd_re", Pins(1)),
            Subsignal("int",  Pins(1)),
        ),
    ]

class Platform(XilinxPlatform):
    def __init__(self):
        XilinxPlatform.__init__(self, "xc7a35tcpg236-1", io=[], toolchain="vivado")

class CRG(Module):
    def __init__(self, platform, soc_config):
        clk = platform.request("clk")
        rst = platform.request("rst")

        self.clock_domains.cd_sys = ClockDomain()
        self.cd_sys.clk.attr.add("keep")
        self.cd_sys.rst.attr.add("keep")

        self.comb += [
            self.cd_sys.clk.eq(clk),
        ]

        self.sync += [
            self.cd_sys.rst.eq(rst),
        ]

class BaseSoC(SoCCore):
    csr_map = {
        "ctrl":   0,
        "uart":   2,
        "timer0": 3,
    }
    interrupt_map = {
        "uart":   3,
        "timer0": 4,
    }
    mem_map = {
        "rom":    0x00000000,
        "sram":   0x10000000,
        "csr":    0xf0000000,
    }
    csr_map.update(SoCCore.csr_map)
    interrupt_map.update(SoCCore.interrupt_map)

    def __init__(self, platform, soc_config, **kwargs):
        platform.add_extension(get_common_ios())
        sys_clk_freq = soc_config["sys_clk_freq"]
        SoCCore.__init__(self, platform, sys_clk_freq,
                         with_uart=True,
                         integrated_main_ram_size=0,
                         **kwargs)
        # crg
        self.submodules.crg = CRG(platform, soc_config)

        if soc_config["platform_name"] in ["accel_sim_release"]:
            # Integrate SPI master
            self.submodules.spi_master = spi_master = SpiMaster(self.platform.request("spi", 0))
            self.add_csr("spi_master", 10, allow_user_defined=True)
            self.add_interrupt("spi_master", 6, allow_user_defined=True)
            self.register_mem("spi_master", 0x30000000, spi_master.bus, 32)
            spi_master.add_source(self.platform)

            # Custom accel simulator IP core
            self.submodules.accel = accel = AccelCore(freq=sys_clk_freq, baud=115200, pads=self.platform.request("spi_slave", 0))
            self.add_csr("accel", 11, allow_user_defined=True)
            self.add_interrupt("accel", 7, allow_user_defined=True)

            if soc_config["mbx_sender"] in ["yes"]:
                # Integrate mailbox sender
                self.submodules.mbx_snd = mbx_snd = MailBoxSenderInf(self.platform.request("mbx_snd", 0))
                self.add_csr("mbx_snd", 12, allow_user_defined=True)

            if soc_config["mbx_receiver"] in ["yes"]:
                # Integrate mailbox receiver
                self.submodules.mbx_rcv = mbx_rcv = MailBoxReceiverInf(self.platform.request("mbx_rcv", 0))
                self.add_csr("mbx_rcv", 13, allow_user_defined=True)
                self.add_interrupt("mbx_rcv", 8, allow_user_defined=True)

            # Integrate GPIO LED
            self.submodules.gpio_led = gpio_led = GpioLED(self.platform.request("user_led", 15))
            self.add_csr("gpio_led", 14, allow_user_defined=True)

        if soc_config["platform_name"] in ["accel_sim"]:
            # Integrate SPI master
            self.submodules.spi_master = spi_master = SpiMaster(self.platform.request("spi", 0))
            self.add_csr("spi_master", 10, allow_user_defined=True)
            self.add_interrupt("spi_master", 6, allow_user_defined=True)
            self.register_mem("spi_master", 0x30000000, spi_master.bus, 32)
            spi_master.add_source(self.platform)

            # Custom accel simulator IP core
            self.submodules.accel = accel = AccelCore(freq=sys_clk_freq, baud=115200, pads=self.platform.request("spi_slave", 0))
            self.add_csr("accel", 11, allow_user_defined=True)
            self.add_interrupt("accel", 7, allow_user_defined=True)

            if soc_config["mbx_sender"] in ["yes"]:
                # Integrate mailbox sender
                self.submodules.mbx_snd = mbx_snd = MailBoxSenderInf(self.platform.request("mbx_snd", 0))
                self.add_csr("mbx_snd", 12, allow_user_defined=True)

            if soc_config["mbx_receiver"] in ["yes"]:
                # Integrate mailbox receiver
                self.submodules.mbx_rcv = mbx_rcv = MailBoxReceiverInf(self.platform.request("mbx_rcv", 0))
                self.add_csr("mbx_rcv", 13, allow_user_defined=True)
                self.add_interrupt("mbx_rcv", 8, allow_user_defined=True)

        if soc_config["platform_name"] in ["accel_test"]:
            # Integrate SPI master
            self.submodules.spi_master = spi_master = SpiMaster(self.platform.request("spi", 0))
            self.add_csr("spi_master", 10, allow_user_defined=True)
            self.add_interrupt("spi_master", 6, allow_user_defined=True)
            self.register_mem("spi_master", 0x30000000, spi_master.bus, 32)
            spi_master.add_source(self.platform)

            # Integrate int module
            self.submodules.gpio_isr = GpioISR(self.platform.request("gpio_irq", 0), rissing_edge_detect=False)
            self.add_csr("gpio_isr", 11, allow_user_defined=True)
            self.add_interrupt("gpio_isr", 7, allow_user_defined=True)

            if soc_config["mbx_sender"] in ["yes"]:
                # Integrate mailbox sender
                self.submodules.mbx_snd = mbx_snd = MailBoxSenderInf(self.platform.request("mbx_snd", 0))
                self.add_csr("mbx_snd", 12, allow_user_defined=True)

            if soc_config["mbx_receiver"] in ["yes"]:
                # Integrate mailbox receiver
                self.submodules.mbx_rcv = mbx_rcv = MailBoxReceiverInf(self.platform.request("mbx_rcv", 0))
                self.add_csr("mbx_rcv", 13, allow_user_defined=True)
                self.add_interrupt("mbx_rcv", 8, allow_user_defined=True)

def main():
    # get config
    if len(sys.argv) < 2:
        print("missing config file")
        exit(1)
    exec(open(sys.argv[1]).read(), globals())

    # generate core
    platform = Platform()
    platform.name = soc_config["platform_name"]

    soc = BaseSoC(platform, soc_config,
                  ident=soc_config["soc_ident"],
                  integrated_rom_size=soc_config["rom_size"],
                  integrated_sram_size=soc_config["sram_size"],
                  cpu_type=soc_config["cpu"],
                  cpu_variant=soc_config["cpu_variant"]
                  )

    output_dir = "build/" + soc_config["platform_name"]
    build_name = soc_config["platform_name"] + "_core"

    builder = Builder(soc, output_dir=output_dir , compile_gateware=False)
    vns = builder.build(build_name=build_name, regular_comb=False)

    # prepare core (could be improved)
    def replace_in_file(filename, _from, _to):
        # Read in the file
        with open(filename, "r") as file :
            filedata = file.read()

        # Replace the target string
        filedata = filedata.replace(_from, _to)

        # Write the file out again
        with open(filename, 'w') as file:
            file.write(filedata)

    init_filename = "mem.init"
    mem_1_init_filename = "mem_1.init"
    mem_2_init_filename = "mem_2.init"

    os.system("mv " + output_dir + "/gateware/mem.init " + output_dir + "/gateware/" + build_name + ".init".format(init_filename))
    os.system("mv " + output_dir + "/gateware/mem_1.init " + output_dir + "/gateware/" + build_name + "_mem_1" + ".init".format(mem_1_init_filename))
    os.system("mv " + output_dir + "/gateware/mem_2.init " + output_dir + "/gateware/" + build_name + "_mem_2" + ".init".format(mem_2_init_filename))

    replace_in_file(output_dir + "/gateware/" + build_name + ".v", init_filename, build_name + ".init")
    replace_in_file(output_dir + "/gateware/" + build_name + ".v", mem_1_init_filename, build_name + "_mem_1" + ".init")
    replace_in_file(output_dir + "/gateware/" + build_name + ".v", mem_2_init_filename, build_name + "_mem_2" + ".init")

if __name__ == "__main__":
    main()
