#
# Copyright 2019-2020 Henry Dang <henrydang@fossil.com>
#
# This file is part of accel simulator project
#
# Accel simulator is free hw/sw: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# Accel simulator is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#

import os
import subprocess

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

import qmatech
from accel import *

class ADXL362(Module):
    def __init__(self, platform):
        pads  = platform.request("spi", 0)
        # clock source request
        clk50 = platform.request("clk50")

        self.clock_domains.cd_sys = ClockDomain()
        self.clock_domains.cd_por = ClockDomain(reset_less=True)        
        
        ###
        self.cd_sys.clk.attr.add("keep")
        self.cd_por.clk.attr.add("keep")

        # power on rst
        rst_n = Signal()
        self.sync.por += rst_n.eq(1)
        self.comb += [
            self.cd_por.clk.eq(clk50),
            self.cd_sys.rst.eq(~rst_n),
        ]

        # sys_clk pll setting (target 100MHz)
        #self.specials += \
        #    Instance("ALTPLL",
        #        p_BANDWIDTH_TYPE="AUTO",
        #        p_CLK0_DIVIDE_BY=1,
        #        p_CLK0_DUTY_CYCLE=50e0,
        #        p_CLK0_MULTIPLY_BY=1,
        #        p_CLK0_PHASE_SHIFT="0",
        #        p_COMPENSATE_CLOCK="CLK0",
        #        p_INCLK0_INPUT_FREQUENCY=20000e0,
        #        p_OPERATION_MODE="NORMAL",
        #        i_INCLK=clk50,
        #        o_CLK=self.cd_sys.clk,
        #        i_ARESET=~rst_n,
        #        i_CLKENA=0x3f,
        #        i_EXTCLKENA=0xf,
        #        i_FBIN=1,
        #        i_PFDENA=1,
        #        i_PLLENA=1,
        #    )

        self.comb += [
            self.cd_sys.clk.eq(clk50),
        ]
        
        self.rx_valid = Signal()
        self.tx_valid = Signal()
        self.rx_data  = Signal(8)
        self.tx_data  = Signal(8)

        self.specials += [
            Instance("SPI_Slave",
                o_o_RX_DV    = self.rx_valid,
                o_o_RX_Byte  = self.rx_data,
                i_i_TX_DV    = self.tx_valid,
                i_i_TX_Byte  = self.tx_data,
                i_i_Rst_L    = ~ResetSignal(),
                i_i_Clk      = ClockSignal(),
                i_i_SPI_Clk  = pads.sclk, 
                o_o_SPI_MISO = pads.miso,
                i_i_SPI_MOSI = pads.mosi,
                i_i_SPI_CS_n = pads.csn,
                )
        ]

        self.sync += [
            self.tx_data.eq(0xAA),
        ]

platform = qmatech.Platform()
dut = ADXL362(platform)
platform.add_source(os.path.join("./", "SPI_Slave.v"))
platform.build(dut)
from litex.build.altera import USBBlaster
prog = USBBlaster()
prog.load_bitstream("build/top.sof")
