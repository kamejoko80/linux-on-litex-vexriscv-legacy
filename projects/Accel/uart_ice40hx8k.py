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

import ice40hx8k
from accel import *

class ICE_UART(Module):
    def __init__(self, platform):
        led0  = platform.request("user_led", 0)
        led1  = platform.request("user_led", 1)
        led2  = platform.request("user_led", 2)
        led3  = platform.request("user_led", 3)
        led4  = platform.request("user_led", 4)
        led5  = platform.request("user_led", 5)
        led6  = platform.request("user_led", 6)
        led7  = platform.request("user_led", 7)
        pads  = platform.request("serial", 0)

        clk12 = platform.request("clk12")
        self.clock_domains.cd_sys = ClockDomain()
        self.reset = Signal()

        # Fout = Fin x (DIVF + 1) / (2^DIVQ x (DIVR + 1))
        self.specials += \
            Instance("SB_PLL40_CORE",
                p_FEEDBACK_PATH="SIMPLE",
                p_PLLOUT_SELECT="GENCLK",
                p_DIVR=0,    # 0
                p_DIVF=19,   # 19
                p_DIVQ=1,    # 1
                p_FILTER_RANGE=0b010,
                i_RESETB=1,
                i_BYPASS=0,
                i_REFERENCECLK=clk12,
                o_PLLOUTCORE=self.cd_sys.clk, # 120MHz
            )

        # POR reset logic- POR generated from sys clk, POR logic feeds sys clk
        # reset.
        self.clock_domains.cd_por = ClockDomain()
        reset_delay = Signal(12, reset=4095)
        self.comb += [
            self.cd_por.clk.eq(self.cd_sys.clk),
            self.cd_sys.rst.eq(reset_delay != 0)
        ]

        self.sync.por += \
            If(reset_delay != 0,
                reset_delay.eq(reset_delay - 1)
            )

        self.specials += AsyncResetSynchronizer(self.cd_por, self.reset)

        # Integrate accel core
        uart = UART(freq=120000000, baud=115200)
        self.submodules += uart

        self.comb += [
            uart.rx.eq(pads.rx),
            pads.tx.eq(uart.tx),
            led0.eq(uart.dout[0]),
            led1.eq(uart.dout[1]),
            led2.eq(uart.dout[2]),
            led3.eq(uart.dout[3]),
            led4.eq(uart.dout[4]),
            led5.eq(uart.dout[5]),
            led6.eq(uart.dout[6]),
            led7.eq(uart.dout[7]),
        ]

        # Echo implementation
        self.sync +=[
            If(uart.readable & uart.writable,
              uart.din.eq(uart.dout),
              uart.tx_start.eq(1),
            )
        ]

platform = ice40hx8k.Platform()
dut = ICE_UART(platform)
platform.build(dut)
subprocess.call(["iceprog", "-o", "0", "build/top.bin"])
