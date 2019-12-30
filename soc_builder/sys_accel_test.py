#!/usr/bin/env python3
import os
import sys
import math
import struct
import argparse

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from litex.boards.platforms import basys3

sys.path.append('../')

from periphs.misc import *

class System(Module):
    def __init__(self, platform):
        serial      = platform.request("serial")
        serial_test = platform.request("serial_test")
        clk100      = platform.request("clk")
        spi0        = platform.request("spi")
        spi_slave0  = platform.request("spi_slave")

        self.clock_domains.cd_clkout = ClockDomain()
        self.clock_domains.cd_sys = ClockDomain()
        self.clock_domains.cd_por = ClockDomain()

        # POR implementation
        self.reset_delay = Signal(max=4095, reset=4095)
        self.reset = Signal()

        self.comb += [
            self.cd_por.clk.eq(clk100),
            self.cd_sys.clk.eq(self.cd_clkout.clk),
        ]

        self.sync.por += [
            If(self.reset_delay != 0,
                self.reset_delay.eq(self.reset_delay - 1)
            ),
            self.reset.eq(self.reset_delay != 0),
            self.cd_sys.rst.eq(self.reset_delay != 0),
        ]

        self.submodules.pll = pll = S7PLL(speedgrade=-1)
        pll.register_clkin(clk100, 100e6)
        pll.create_clkout(self.cd_clkout, 250e6) # CPU clock max = 250MHz

        # Important, this ensures the system works properly
        platform.add_period_constraint(clk100, 10)
        platform.add_period_constraint(self.cd_sys.clk, 4)
        platform.add_period_constraint(self.cd_clkout.clk, 4)

        # SPI test bus internal signals
        self.spi_test_clk  = Signal()
        self.spi_test_miso = Signal()
        self.spi_test_mosi = Signal()
        self.spi_test_csn  = Signal()
        self.accel_int1    = Signal()

        # Mailbox implementation
        self.submodules.mbx1 = mbx1 = MailBox(fifo_depth=32)
        self.submodules.mbx2 = mbx2 = MailBox(fifo_depth=32)

        # Accel simulator core
        self.specials += Instance("accel_sim_core",
            i_clk                 = self.cd_clkout.clk,
            i_rst                 = self.reset,
            i_serial_rx           = serial.rx,
            o_serial_tx           = serial.tx,

            # SPI master
            o_spi0_sclk           = spi0.sclk,
            i_spi0_miso           = spi0.miso,
            o_spi0_mosi           = spi0.mosi,
            o_spi0_csn            = spi0.csn,
           #i_spi0_irq            = spi0.irq,

            # SPI slave, accel
            i_spi_slave0_sck      = self.spi_test_clk,
           io_spi_slave0_miso     = self.spi_test_miso,
            i_spi_slave0_mosi     = self.spi_test_mosi,
            i_spi_slave0_csn      = self.spi_test_csn,
            o_spi_slave0_int1     = self.accel_int1,
           #o_spi_slave0_int2

            # Debug LEDs
            o_spi_slave0_led0     = spi_slave0.led0,
            o_spi_slave0_led1     = spi_slave0.led1,
            o_spi_slave0_led2     = spi_slave0.led2,
            o_spi_slave0_led3     = spi_slave0.led3,
            o_spi_slave0_led4     = spi_slave0.led4,
            o_spi_slave0_led5     = spi_slave0.led5,
            o_spi_slave0_led6     = spi_slave0.led6,
	
            # Accel uart
            #i_spi_slave0_tx      = spi_slave0.tx,
            #i_spi_slave0_rx      = spi_slave0.rx,

            # Mailbox sender
            o_mbx_snd0_dout_r     = mbx1.dout_r,
            o_mbx_snd0_dout_re    = mbx1.dout_re,
            o_mbx_snd0_int_r      = mbx1.int_r,
            o_mbx_snd0_int_re     = mbx1.int_re,

            # Mailbox receiver
            i_mbx_rcv0_din_status = mbx2.din_status,
            i_mbx_rcv0_len_status = mbx2.len_status,
            o_mbx_rcv0_rd_r       = mbx2.rd_r,
            o_mbx_rcv0_rd_re      = mbx2.rd_re,
            i_mbx_rcv0_int        = mbx2.int,
        )

        # Accel test core
        self.specials += Instance("accel_test_core",
            i_clk                 = self.cd_clkout.clk,
            i_rst                 = self.reset,
            i_serial_rx           = serial_test.rx,
            o_serial_tx           = serial_test.tx,

            # SPI master
            o_spi0_sclk           = self.spi_test_clk,
            i_spi0_miso           = self.spi_test_miso,
            o_spi0_mosi           = self.spi_test_mosi,
            o_spi0_csn            = self.spi_test_csn,
           #i_spi0_irq
            i_gpio_irq0           = self.accel_int1,

            # Mailbox sender
            o_mbx_snd0_dout_r     = mbx2.dout_r,
            o_mbx_snd0_dout_re    = mbx2.dout_re,
            o_mbx_snd0_int_r      = mbx2.int_r,
            o_mbx_snd0_int_re     = mbx2.int_re,

            # Mailbox receiver
            i_mbx_rcv0_din_status = mbx1.din_status,
            i_mbx_rcv0_len_status = mbx1.len_status,
            o_mbx_rcv0_rd_r       = mbx1.rd_r,
            o_mbx_rcv0_rd_re      = mbx1.rd_re,
            i_mbx_rcv0_int        = mbx1.int,
        )

        # Accel simulator core
        platform.add_source(os.path.join("build/accel_sim/gateware", "accel_sim_core.v"))
        platform.add_source(os.path.join("build/accel_sim/gateware", "accel_sim_core.init"))
        platform.add_source(os.path.join("build/accel_sim/gateware", "accel_sim_core_mem_1.init"))
        platform.add_source(os.path.join("build/accel_sim/gateware", "accel_sim_core_mem_2.init"))

        # Accel test core
        platform.add_source(os.path.join("build/accel_test/gateware", "accel_test_core.v"))
        platform.add_source(os.path.join("build/accel_test/gateware", "accel_test_core.init"))
        platform.add_source(os.path.join("build/accel_test/gateware", "accel_test_core_mem_1.init"))
        platform.add_source(os.path.join("build/accel_test/gateware", "accel_test_core_mem_2.init"))

        # Vexriscv, SPI master core
        platform.add_source(os.path.join("../litex/litex/soc/cores/cpu/vexriscv/verilog", "VexRiscv_Min.v"))
        platform.add_source(os.path.join("../periphs/verilog/spi", "spi_defines.v"))
        platform.add_source(os.path.join("../periphs/verilog/spi", "spi_clgen.v"))
        platform.add_source(os.path.join("../periphs/verilog/spi", "spi_shift.v"))
        platform.add_source(os.path.join("../periphs/verilog/spi", "spi_top.v"))
        platform.add_source(os.path.join("../periphs/verilog/spi", "timescale.v"))

def main():
    description = "LiteX-VexRiscv SoC Builder\n\n"
    parser = argparse.ArgumentParser(description=description, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("--build", action="store_true", help="build bitstream")
    parser.add_argument("--load", action="store_true", help="load bitstream (to SRAM)")
    parser.add_argument("--flash", action="store_true", help="flash bitstream/images (to SPI Flash)")
    args = parser.parse_args()

    if args.build:
        platform = basys3.Platform()
        dut = System(platform)
        platform.build(dut, build_dir="build/sys_accel_test/gateware")

    if args.load:
        from litex.build.xilinx import VivadoProgrammer
        prog = VivadoProgrammer()
        prog.load_bitstream("build/sys_accel_test/gateware/top.bit")

    if args.flash:
        from litex.build.openocd import OpenOCD
        prog = OpenOCD("../prog/openocd_xilinx.cfg",
            flash_proxy_basename="../prog/bscan_spi_xc7a35t.bit")
        prog.set_flash_proxy_dir(".")
        prog.flash(0, "build/sys_accel_test/gateware/top.bin")

if __name__ == "__main__":
    main()       