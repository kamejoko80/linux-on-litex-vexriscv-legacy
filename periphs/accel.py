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
import sys

from random import randrange

from migen import *
from migen.fhdl import verilog
from migen.fhdl.specials import Tristate
from migen.genlib.misc import WaitTimer
from migen.genlib.fifo import SyncFIFOBuffered

from litex.soc.interconnect.csr import *
from litex.soc.interconnect.csr_eventmanager import *

sys.path.append('../')

from periphs.misc import *

class RegisterArray(Module):
    def __init__(self):
        self.addr   = Signal(8)
        self.dw     = Signal(8)
        self.dr     = Signal(8)
        self.r      = Signal()
        self.w      = Signal()

        # Register set
        self.DEVID_AD       = Signal(8, reset=0xAD)      # DEVID_AD       [R]
        self.DEVID_MST      = Signal(8, reset=0x1D)      # DEVID_MST      [R]
        self.PARTID         = Signal(8, reset=0xF2)      # PARTID         [R]
        self.REVID          = Signal(8, reset=0x01)      # REVID          [R]

        self.XDATA          = Signal(8, reset=0x00)      # XDATA          [R] internally writable
        self.YDATA          = Signal(8, reset=0x00)      # YDATA          [R] internally writable
        self.ZDATA          = Signal(8, reset=0x00)      # ZDATA          [R] internally writable
        self.STATUS         = Signal(8, reset=0x40)      # STATUS         [R] internally writable
        self.FIFO_ENTRIES_L = Signal(8, reset=0x00)      # FIFO_ENTRIES_L [R] internally writable
        self.FIFO_ENTRIES_H = Signal(8, reset=0x00)      # FIFO_ENTRIES_H [R] internally writable
        self.XDATA_L        = Signal(8, reset=0x00)      # XDATA_L        [R] internally writable
        self.XDATA_H        = Signal(8, reset=0x00)      # XDATA_H        [R] internally writable
        self.YDATA_L        = Signal(8, reset=0x00)      # YDATA_L        [R] internally writable
        self.YDATA_H        = Signal(8, reset=0x00)      # YDATA_H        [R] internally writable
        self.ZDATA_L        = Signal(8, reset=0x00)      # ZDATA_L        [R] internally writable
        self.ZDATA_H        = Signal(8, reset=0x00)      # ZDATA_H        [R] internally writable
        self.TEMP_L         = Signal(8, reset=0x00)      # TEMP_L         [R] internally writable
        self.TEMP_H         = Signal(8, reset=0x00)      # TEMP_H         [R] internally writable

        self.SOFT_RESET     = Signal(8, reset=0x00)      # SOFT_RESET     [W]
        self.THRESH_ACT_L   = Signal(8, reset=0x00)      # THRESH_ACT_L   [RW]
        self.THRESH_ACT_H   = Signal(8, reset=0x00)      # THRESH_ACT_H   [RW]
        self.TIME_ACT       = Signal(8, reset=0x00)      # TIME_ACT       [RW]
        self.THRESH_INACT_L = Signal(8, reset=0x00)      # THRESH_INACT_L [RW]
        self.THRESH_INACT_H = Signal(8, reset=0x00)      # THRESH_INACT_H [RW]
        self.TIME_INACT_L   = Signal(8, reset=0x00)      # TIME_INACT_L   [RW]
        self.TIME_INACT_H   = Signal(8, reset=0x00)      # TIME_INACT_H   [RW]
        self.ACT_INACT_CTL  = Signal(8, reset=0x00)      # ACT_INACT_CTL  [RW]
        self.FIFO_CONTROL   = Signal(8, reset=0x00)      # FIFO_CONTROL   [RW]
        self.FIFO_SAMPLES   = Signal(8, reset=0x80)      # FIFO_SAMPLES   [RW]
        self.INTMAP1        = Signal(8, reset=0x00)      # INTMAP1        [RW]
        self.INTMAP2        = Signal(8, reset=0x00)      # INTMAP2        [RW]
        self.FILTER_CTL     = Signal(8, reset=0x13)      # FILTER_CTL     [RW]
        self.POWER_CTL      = Signal(8, reset=0x00)      # POWER_CTL      [RW]
       #self.SELF_TEST      = Signal(8, reset=0x00)      # SELF_TEST      [RW]

        self.sync += [
            If(self.r,
                Case(self.addr, {
                    0:  self.dr.eq(self.DEVID_AD),       # DEVID_AD       [R]  [0x00]
                    1:  self.dr.eq(self.DEVID_MST),      # DEVID_MST      [R]  [0x01]
                    2:  self.dr.eq(self.PARTID),         # PARTID         [R]  [0x02]
                    3:  self.dr.eq(self.REVID),          # REVID          [R]  [0x03]

                    8:  self.dr.eq(self.XDATA),          # XDATA          [R]  [0x08]
                    9:  self.dr.eq(self.YDATA),          # YDATA          [R]  [0x09]
                    10: self.dr.eq(self.ZDATA),          # ZDATA          [R]  [0x0A]
                    11:(self.dr.eq(self.STATUS),         # STATUS         [R]  [0x0B]
                        self.STATUS[4].eq(0),            # Clear ACT flag
                        self.STATUS[5].eq(0),            # Clear INACT flag
                        ),
                    12: self.dr.eq(self.FIFO_ENTRIES_L), # FIFO_ENTRIES_L [R]  [0x0C]
                    13: self.dr.eq(self.FIFO_ENTRIES_H), # FIFO_ENTRIES_H [R]  [0x0D]
                    14: self.dr.eq(self.XDATA_L),        # XDATA_L        [R]  [0x0E]
                    15: self.dr.eq(self.XDATA_H),        # XDATA_H        [R]  [0x0F]
                    16: self.dr.eq(self.YDATA_L),        # YDATA_L        [R]  [0x10]
                    17: self.dr.eq(self.YDATA_H),        # YDATA_H        [R]  [0x11]
                    18: self.dr.eq(self.ZDATA_L),        # ZDATA_L        [R]  [0x12]
                    19: self.dr.eq(self.ZDATA_H),        # ZDATA_H        [R]  [0x13]
                    20: self.dr.eq(self.TEMP_L),         # TEMP_L         [R]  [0x14]
                    21: self.dr.eq(self.TEMP_H),         # TEMP_H         [R]  [0x15]

                   #31: self.dr.eq(self.SOFT_RESET),     # SOFT_RESET     [W]  [0x1F]
                    32: self.dr.eq(self.THRESH_ACT_L),   # THRESH_ACT_L   [RW] [0x20]
                    33: self.dr.eq(self.THRESH_ACT_H),   # THRESH_ACT_H   [RW] [0x21]
                    34: self.dr.eq(self.TIME_ACT),       # TIME_ACT       [RW] [0x22]
                    35: self.dr.eq(self.THRESH_INACT_L), # THRESH_INACT_L [RW] [0x23]
                    36: self.dr.eq(self.THRESH_INACT_H), # THRESH_INACT_H [RW] [0x24]
                    37: self.dr.eq(self.TIME_INACT_L),   # TIME_INACT_L   [RW] [0x25]
                    38: self.dr.eq(self.TIME_INACT_H),   # TIME_INACT_H   [RW] [0x26]
                    39: self.dr.eq(self.ACT_INACT_CTL),  # ACT_INACT_CTL  [RW] [0x27]
                    40: self.dr.eq(self.FIFO_CONTROL),   # FIFO_CONTROL   [RW] [0x28]
                    41: self.dr.eq(self.FIFO_SAMPLES),   # FIFO_SAMPLES   [RW] [0x29]
                    42: self.dr.eq(self.INTMAP1),        # INTMAP1        [RW] [0x2A]
                    43: self.dr.eq(self.INTMAP2),        # INTMAP2        [RW] [0x2B]
                    44: self.dr.eq(self.FILTER_CTL),     # FILTER_CTL     [RW] [0x2C]
                    45: self.dr.eq(self.POWER_CTL),      # POWER_CTL      [RW] [0x2D]
                   #46: self.dr.eq(self.SELF_TEST),      # SELF_TEST      [RW] [0x2E]
                    "default": self.dr.eq(0),
                })
            ).Elif(self.w,
                Case(self.addr, {
                   #8:  self.XDATA.eq( self.dw),         # XDATA          [R]
                   #9:  self.YDATA.eq( self.dw),         # YDATA          [R]
                   #10: self.ZDATA.eq(self.dw),          # ZDATA          [R]
                   #11: self.STATUS.eq(self.dw),         # STATUS         [R]
                   #12: self.FIFO_ENTRIES_L.eq(self.dw), # FIFO_ENTRIES_L [R]
                   #13: self.FIFO_ENTRIES_H.eq(self.dw), # FIFO_ENTRIES_H [R]
                   #14: self.XDATA_L.eq(self.dw),        # XDATA_L        [R]
                   #15: self.XDATA_H.eq(self.dw),        # XDATA_H        [R]
                   #16: self.YDATA_L.eq(self.dw),        # YDATA_L        [R]
                   #17: self.YDATA_H.eq(self.dw),        # YDATA_H        [R]
                   #18: self.ZDATA_L.eq(self.dw),        # ZDATA_L        [R]
                   #19: self.ZDATA_H.eq(self.dw),        # ZDATA_H        [R]
                   #20: self.TEMP_L.eq(self.dw),         # TEMP_L         [R]
                   #21: self.TEMP_H.eq(self.dw),         # TEMP_H         [R]

                    31: self.SOFT_RESET.eq(self.dw),     # SOFT_RESET     [W]
                    32: self.THRESH_ACT_L.eq(self.dw),   # THRESH_ACT_L   [RW]
                    33: self.THRESH_ACT_H.eq(self.dw),   # THRESH_ACT_H   [RW]
                    34: self.TIME_ACT.eq(self.dw),       # TIME_ACT       [RW]
                    35: self.THRESH_INACT_L.eq(self.dw), # THRESH_INACT_L [RW]
                    36: self.THRESH_INACT_H.eq(self.dw), # THRESH_INACT_H [RW]
                    37: self.TIME_INACT_L.eq(self.dw),   # TIME_INACT_L   [RW]
                    38: self.TIME_INACT_H.eq(self.dw),   # TIME_INACT_H   [RW]
                    39: self.ACT_INACT_CTL.eq(self.dw),  # ACT_INACT_CTL  [RW]
                    40: self.FIFO_CONTROL.eq(self.dw),   # FIFO_CONTROL   [RW]
                    41: self.FIFO_SAMPLES.eq(self.dw),   # FIFO_SAMPLES   [RW]
                    42: self.INTMAP1.eq(self.dw),        # INTMAP1        [RW]
                    43: self.INTMAP2.eq(self.dw),        # INTMAP2        [RW]
                    44: self.FILTER_CTL.eq(self.dw),     # FILTER_CTL     [RW]
                    45: self.POWER_CTL.eq(self.dw),      # POWER_CTL      [RW]
                   #46: self.SELF_TEST.eq(self.dw),      # SELF_TEST      [RW]
                })
            )
        ]

class AccelCore(Module, AutoCSR):
    def __init__(self, freq, baud, pads):
        # Physical pins interface
        #self.sck  = Signal()  # SCK pin input
        #self.mosi = Signal()  # MOSI pin input
        #self.miso = Signal()  # MISO pin output
        #self.csn  = Signal()  # CSN pin input
        #self.tx   = Signal()  # UART tx pin
        #self.rx   = Signal()  # UART rx pin
        #self.int1 = Signal()  # INT1 interrupt pin
        #self.int2 = Signal()  # INT2 interrupt pin

        # Led debug
        # self.led  = Signal()

        # Constant, parametter setting
        FIFO_DEPTH = 170

        # Internal core signals
        self.miso = Signal()  # Internal miso signal
        self.rxc  = Signal()  # Data RX complete (wire)
        self.rxd  = Signal(8) # RX data

        # Core behavior internal signals
        self.fifo_axis_level = Signal(max=512)
        self.fifo_samples = Signal(9, reset=0x80)

        # Misc signals
        self.sck_cnt  = Signal(2) # SCK edge detect counter
        self.sck_r    = Signal()  # SCK rising edge detect signal (wire)
        self.sck_f    = Signal()  # SCK falling edge detect signal (wire)
        self.csn_cnt  = Signal(2) # CSN edge detect counter
        self.csn_r    = Signal()  # CSN rising edge detect signal (wire)
        self.csn_f    = Signal()  # SCK falling edge detect signal (wire)
        self.mosi_cnt = Signal(2) # MOSI edge detect counter
        self.bitcnt   = Signal(3) # Bit count
        self.mosi_s   = Signal()  # MOSI sample (wire)
        self.tx_buf   = Signal(8) # TX data buffer

        # Register set internal bus, signals
        self.bus_addr = Signal(8)
        self.bus_dw   = Signal(8)
        self.bus_dr   = Signal(8)
        self.bus_r    = Signal()
        self.bus_w    = Signal()

        # CMD & ADDR storage
        self.str_cmd  = Signal(8)
        self.str_addr = Signal(8)

        # Connect to register set
        reg = ResetInserter()(RegisterArray())
        self.submodules += reg

        # Connect register array bus
        self.comb += [
            reg.addr.eq(self.bus_addr),
            reg.r.eq(self.bus_r),
            reg.w.eq(self.bus_w),
            reg.dw.eq(self.bus_dw),
            self.bus_dr.eq(reg.dr),
        ]

        # LED debug
        self.comb += [
            pads.led0.eq(reg.STATUS[0]), # DATA_READY
            pads.led1.eq(reg.STATUS[1]), # FIFO_READY
            pads.led2.eq(reg.STATUS[2]), # FIFO_WATERMARK
            pads.led3.eq(reg.STATUS[3]), # FIFO_OVERRUN
            pads.led4.eq(reg.STATUS[4]), # ACT
            pads.led5.eq(reg.STATUS[5]), # INACT
            pads.led6.eq(reg.STATUS[6]), # AWAKE
        ]

        # Connect to the FIFO buffer
        fifo = SyncFIFOBuffered(width=48, depth=FIFO_DEPTH)
        self.submodules += fifo

        # FIFO entry for accessing
        self.fifo_entry = Signal(48, reset_less=True)
        self.fifo_byte_sent = Signal(3, reset=0, reset_less=True)
        self.fifo_entry_read_cnt = Signal(max=FIFO_DEPTH, reset=0, reset_less=True)

        # Submodule FSM handles data in/out activities
        fsm = ResetInserter()(FSM(reset_state = "IDLE"))
        self.submodules += fsm

        # To make sure we can reset the FSM properly
        self.comb += [
            fsm.reset.eq(pads.csn),
        ]

        # FSM behavior description
        fsm.act("IDLE",
            If(~pads.csn,
                NextValue(self.tx_buf, 0),
                NextState("CMD_PHASE"),
            ),
            NextValue(fifo.re, 0),
        )
        fsm.act("CMD_PHASE",
            If(self.rxc,
                NextValue(self.str_cmd, self.rxd),
                NextState("CMD_DECODE"),
            )
        )
        fsm.act("CMD_DECODE",
            If(self.str_cmd == 0x0A, # Reg write
                NextState("ADDR_PHASE"),
            ).Elif(self.str_cmd == 0x0B, # Reg read
                NextState("ADDR_PHASE"),
            ).Elif(self.str_cmd == 0x0D, # FIFO read
                NextState("READ_FIFO"),
            ).Else(
                #NextValue(self.led, 0),
                NextState("IDLE"),
            )
        )
        fsm.act("ADDR_PHASE",
            If(self.rxc,
                NextValue(self.str_addr, self.rxd),
                NextState("DETERMINE_REG_ACCESS"),
            )
        )
        fsm.act("DETERMINE_REG_ACCESS",
            If(self.str_cmd == 0x0A, # Reg write
                NextState("REG_VALUE_SHIFTIN"),
            ).Elif(self.str_cmd == 0x0B, # Reg read
                If((self.str_addr >= 14) & (self.str_addr <= 19), # Read X, Y, Z registers
                    NextState("READ_FIFO"),
                ).Else( # Read other registers
                    NextValue(self.bus_addr, self.str_addr),
                    NextValue(self.bus_r, 1),
                    NextState("LOAD_SHIFT_OUT_DATA"),
                ),
            ).Else(
                NextState("IDLE"),
            )
        )
        fsm.act("REG_VALUE_SHIFTIN",
            If(self.rxc,
                NextValue(self.bus_addr, self.str_addr),
                NextValue(self.bus_dw, self.rxd),
                NextValue(self.bus_w, 1),
                NextState("REG_WRITE_STROBE"),
            )
        )
        fsm.act("REG_WRITE_STROBE",
            NextState("REG_WRITE_VALUE"),
        )
        fsm.act("REG_WRITE_VALUE",
            NextValue(self.bus_w, 0),
            NextState("REG_WRITE_NEXT"),
        )
        fsm.act("REG_WRITE_NEXT",
            If(self.csn_r,
                NextState("IDLE"),
            ).Elif(self.rxc,
                NextValue(self.bus_addr, self.bus_addr + 1),
                NextValue(self.bus_dw, self.rxd),
                NextValue(self.bus_w, 1),
                NextState("REG_WRITE_STROBE"),
            )
        )
        fsm.act("LOAD_SHIFT_OUT_DATA",
            NextState("LOAD_TX_BUF"),
        )
        fsm.act("LOAD_TX_BUF",
            NextValue(self.tx_buf, self.bus_dr),
            NextValue(self.bus_r, 0),
            NextState("SHIFTING_OUT"),
        )
        fsm.act("SHIFTING_OUT",
            If(self.rxc,
                NextState("SHIFT_OUT_DONE"),
            )
        )
        fsm.act("SHIFT_OUT_DONE",
            If(self.csn_r,
                NextState("IDLE"),
            ).Elif(self.bus_addr < 0x2D,
                NextValue(self.bus_addr, self.bus_addr + 1),
                NextValue(self.bus_r, 1),
                NextState("LOAD_SHIFT_OUT_DATA"),
            ).Else(
                NextState("IDLE"),
            )
        )
        fsm.act("READ_FIFO",
            If(self.fifo_byte_sent == 0,
                If(fifo.readable,
                    NextState("READ_FIFO_ENTRY"),
                ),
            ).Elif(self.str_cmd == 0x0D, # Read FIFO command
                NextState("PREPARE_SHIFT_FIFO_BYTE_OUT"),
            ).Elif(self.str_cmd == 0x0B, # Read X, Y, Z registers
                NextState("PREPARE_SHIFT_XYZ_BYTE_OUT"),
            ).Else( # Invalid state
                NextState("IDLE"),
            ),
        )
        fsm.act("READ_FIFO_ENTRY",
            NextValue(self.fifo_entry_read_cnt, self.fifo_entry_read_cnt + 1),
            NextValue(self.fifo_byte_sent, 1),
            NextValue(self.fifo_entry, fifo.dout), # Read FIFO entry
            NextValue(fifo.re, 1), # Read strobe for next entry
            If(self.str_cmd == 0x0D, # Read FIFO command
                NextState("PREPARE_SHIFT_FIFO_BYTE_OUT"),
            ).Elif(self.str_cmd == 0x0B, # Read X, Y, Z registers
                NextState("PREPARE_SHIFT_XYZ_BYTE_OUT"),
            ).Else( # Invalid state
                NextState("IDLE"),
            ),
        )
        fsm.act("PREPARE_SHIFT_XYZ_BYTE_OUT",
            NextValue(fifo.re, 0),
            If(self.str_addr == 14, # XL
                If(self.fifo_byte_sent <= 1,
                    NextValue(self.fifo_byte_sent, 1),
                    NextValue(self.tx_buf, self.fifo_entry[0:8]),
                    NextState("SHIFT_XYZ_BYTE_OUT"),
                ).Else(
                    NextValue(self.fifo_byte_sent, 0),
                    NextState("READ_FIFO"),
                ),
            ).Elif(self.str_addr == 15, # XH
                If(self.fifo_byte_sent <= 2,
                    NextValue(self.fifo_byte_sent, 2),
                    NextValue(self.tx_buf, self.fifo_entry[8:16]),
                    NextState("SHIFT_XYZ_BYTE_OUT"),
                ).Else(
                    NextValue(self.fifo_byte_sent, 0),
                    NextState("READ_FIFO"),
                ),
            ).Elif(self.str_addr == 16, # YL
                If(self.fifo_byte_sent <= 3,
                    NextValue(self.fifo_byte_sent, 3),
                    NextValue(self.tx_buf, self.fifo_entry[16:24]),
                    NextState("SHIFT_XYZ_BYTE_OUT"),
                ).Else(
                    NextValue(self.fifo_byte_sent, 0),
                    NextState("READ_FIFO"),
                ),
            ).Elif(self.str_addr == 17, # YH
                If(self.fifo_byte_sent <= 4,
                    NextValue(self.fifo_byte_sent, 4),
                    NextValue(self.tx_buf, self.fifo_entry[24:32]),
                    NextState("SHIFT_XYZ_BYTE_OUT"),
                ).Else(
                    NextValue(self.fifo_byte_sent, 0),
                    NextState("READ_FIFO"),
                ),
            ).Elif(self.str_addr == 18, # ZL
                If(self.fifo_byte_sent <= 5,
                    NextValue(self.fifo_byte_sent, 5),
                    NextValue(self.tx_buf, self.fifo_entry[32:40]),
                    NextState("SHIFT_XYZ_BYTE_OUT"),
                ).Else(
                    NextValue(self.fifo_byte_sent, 0),
                    NextState("READ_FIFO"),
                ),
            ).Elif(self.str_addr == 19, # ZH
                If(self.fifo_byte_sent <= 6,
                    NextValue(self.fifo_byte_sent, 6),
                    NextValue(self.tx_buf, self.fifo_entry[40:48]),
                    NextState("SHIFT_XYZ_BYTE_OUT"),
                ).Else(
                    NextValue(self.fifo_byte_sent, 0),
                    NextState("READ_FIFO"),
                ),
            ).Else( # self.str_addr exceeds 19
                NextValue(self.bus_addr, self.str_addr),
                NextState("SHIFT_OUT_DONE"),
            ),
        )
        fsm.act("SHIFT_XYZ_BYTE_OUT",
            If(self.rxc, # Just wait for byte shifting
                If(self.fifo_byte_sent < 6,
                    NextValue(self.fifo_byte_sent, self.fifo_byte_sent + 1),
                ).Else(
                    NextValue(self.fifo_byte_sent, 0),
                ),
                If(self.str_addr < 19,
                    NextValue(self.str_addr, self.str_addr + 1),
                    NextState("PREPARE_SHIFT_XYZ_BYTE_OUT"),
                ).Else(
                    NextValue(self.bus_addr, self.str_addr),
                    NextState("SHIFT_OUT_DONE"),
                )
            ),
        )
        fsm.act("PREPARE_SHIFT_FIFO_BYTE_OUT",
            NextValue(fifo.re, 0),
            If(self.fifo_byte_sent == 1,
                NextValue(self.tx_buf, self.fifo_entry[0:8]),
                NextState("SHIFT_FIFO_BYTE_OUT"),
            ).Elif(self.fifo_byte_sent == 2,
                NextValue(self.tx_buf, self.fifo_entry[8:16]),
                NextState("SHIFT_FIFO_BYTE_OUT"),
            ).Elif(self.fifo_byte_sent == 3,
                NextValue(self.tx_buf, self.fifo_entry[16:24]),
                NextState("SHIFT_FIFO_BYTE_OUT"),
            ).Elif(self.fifo_byte_sent == 4,
                NextValue(self.tx_buf, self.fifo_entry[24:32]),
                NextState("SHIFT_FIFO_BYTE_OUT"),
            ).Elif(self.fifo_byte_sent == 5,
                NextValue(self.tx_buf, self.fifo_entry[32:40]),
                NextState("SHIFT_FIFO_BYTE_OUT"),
            ).Elif(self.fifo_byte_sent == 6,
                NextValue(self.tx_buf, self.fifo_entry[40:48]),
                NextState("SHIFT_FIFO_BYTE_OUT"),
            ).Elif(self.fifo_byte_sent >= 7,
                NextValue(self.fifo_byte_sent, 0),
                NextState("READ_FIFO"),
            ),
        )
        fsm.act("SHIFT_FIFO_BYTE_OUT",
            If(self.rxc, # Just wait for byte shifting
                NextValue(self.fifo_byte_sent, self.fifo_byte_sent + 1),
                NextState("PREPARE_SHIFT_FIFO_BYTE_OUT"),
            ),
        )

        # Edge detect signal combinatorial
        self.comb += [
            self.sck_r.eq(self.sck_cnt == 1),
            self.sck_f.eq(self.sck_cnt == 2),
            self.csn_r.eq(self.csn_cnt == 1),
            self.csn_f.eq(self.csn_cnt == 2),
        ]

        # MOSI data sampling
        self.comb += [
            self.mosi_s.eq(self.mosi_cnt[1]),
        ]

        # Edge detector behavior description
        self.sync += [
            self.sck_cnt[1].eq(self.sck_cnt[0]),
            self.sck_cnt[0].eq(pads.sck),
            self.csn_cnt[1].eq(self.csn_cnt[0]),
            self.csn_cnt[0].eq(pads.csn),
            self.mosi_cnt[1].eq(self.mosi_cnt[0]),
            self.mosi_cnt[0].eq(pads.mosi),
        ]

        # RX data behavior description
        self.sync += [
            If(self.csn_f,
                self.bitcnt.eq(0),
                self.rxd.eq(0),
            ).Elif(self.sck_f,
                self.bitcnt.eq(self.bitcnt+1),
                self.rxd.eq(self.rxd << 1),
                self.rxd[0].eq(self.mosi_s),
            )
        ]

        # RX completed notification
        self.sync += [
            self.rxc.eq(~pads.csn & self.sck_f & (self.bitcnt == 7)),
        ]

        # TX data behavior description
        self.sync += [
            If(self.csn_f,
                self.tx_buf.eq(0),
            ).Elif(self.sck_f,
                self.tx_buf.eq(self.tx_buf<<1),
            )
        ]

        # MISO output behavior description
        self.comb += [
            self.miso.eq(self.tx_buf[7]),
        ]

        # Implement tristate on MISO pad
        self.dummy = Signal()
        self.specials += Tristate(pads.miso, self.miso, ~pads.csn, self.dummy)

        ########### Accel behavior implementation #################

        # System clock domain
        self.sys_rst = ResetSignal("sys")

        # SoC to accel IP core CSR interface
        self.soc2ip_dx = CSRStorage(16)
        self.soc2ip_dy = CSRStorage(16)
        self.soc2ip_dz = CSRStorage(16)
        self.soc2ip_we = CSRStorage(1, reset = 0)  # SoC write data strobe
        self.soc2ip_st = CSRStorage(1, reset = 0)  # SoC status
        self.soc2ip_full = CSRStatus(1)
        self.soc2ip_done = CSRStatus(1, reset = 0)

        self.comb += [
            self.fifo_axis_level.eq(fifo.level*3),   # fifo.level / 3
            reg.FIFO_ENTRIES_L.eq(self.fifo_axis_level[:8]), # FIFO_ENTRIES_L
            reg.FIFO_ENTRIES_H.eq(self.fifo_axis_level[8:]), # FIFO_ENTRIES_H
            self.fifo_samples[:8].eq(reg.FIFO_SAMPLES[:8]),  # FIFO_SAMPLES (LSB)
            self.fifo_samples[8:].eq(reg.FIFO_CONTROL[3]),   # FIFO_SAMPLES (MSB) = FIFO_CONTROL[3] (AH)
        ]

        self.sync += [
            If(reg.SOFT_RESET == 0x52, # Soft reset request
               reg.reset.eq(1),
            ).Else(
               reg.reset.eq(0),
            ),
        ]

        self.sync += [
            reg.STATUS[3].eq(fifo.level>=FIFO_DEPTH), # FIFO_OVERRUN
            If(self.fifo_axis_level >= self.fifo_samples,
                reg.STATUS[2].eq(1),                  # FIFO_WATERMARK is set
            ),
            #If((2*self.fifo_axis_level) <= self.fifo_samples,
            #    reg.STATUS[2].eq(0),                 # FIFO_WATERMARK is cleared
            #    pads.led1.eq(0),                     # LED debug off
            #),
            If(3*self.fifo_entry_read_cnt >= self.fifo_samples,
                self.fifo_entry_read_cnt.eq(0),       # Reset FIFO entry read counter
                reg.STATUS[2].eq(0),                  # FIFO_WATERMARK is cleared
            ),
            reg.STATUS[1].eq(fifo.level>=1),          # FIFO_READY (at least one valid sample in the FIFO buffer)
            reg.STATUS[0].eq(fifo.level>=1),          # DATA_READY (new valid sample available) (not implement)
        ]

        # Add output data rate controller
        odrctrl = ODRController(freq=freq, fbase=400)
        self.submodules += odrctrl

        # Connect to ODR parametter input
        self.comb += [
            odrctrl.odr.eq(reg.FILTER_CTL[:3]),       # ODR[2:0] (FILTER_CTL)
        ]

        # ODR controller
        self.sync += [
            If(reg.POWER_CTL[:2] == 0x02,             # MEASURE[1:0] = 0x02 (POWER_CTL)
                #If(reg.STATUS[2],                    # And FIFO_WATERMARK is cleared
                #    odrctrl.ena.eq(0)
                #).Else(
                #    odrctrl.ena.eq(1)
                #),
                odrctrl.ena.eq(1),
            ).Else(
                odrctrl.ena.eq(0),
            ),
        ]

        # Interrupt signaling
        self.comb += [
            # Both 2 bits FIFO_WATERMARK in STATUS & INTMAP1 are set
            If(~reg.INTMAP1[7],
                pads.int1.eq(reg.INTMAP1[2] & reg.STATUS[2]),   # Active high
            ).Else(
                pads.int1.eq(~reg.INTMAP1[2] | ~reg.STATUS[2]), # Active low
            ),
            # Both 2 bits FIFO_WATERMARK in STATUS & INTMAP2 are set
            If(~reg.INTMAP2[7],
                pads.int2.eq(reg.INTMAP2[2] & reg.STATUS[2]),   # Active high
            ).Else(
                pads.int2.eq(~reg.INTMAP2[2] | ~reg.STATUS[2]), # Active low
            )
        ]

        # ODR interrupt from IP to SoC
        self.submodules.ev = EventManager()
        self.ev.ip2soc_irq = EventSourcePulse() # Rising edge interrupt
        self.ev.finalize()

        # ODR controller rising edge output triggers interrupt signal
        self.sync += [
            If(reg.reset,
                self.sys_rst.eq(1), # System reset
            ).Elif(odrctrl.foutr & self.soc2ip_st.storage,
                self.ev.ip2soc_irq.trigger.eq(1),
            ).Else(
                self.ev.ip2soc_irq.trigger.eq(0),
            )
        ]

        # Detect rising edge soc2ip_we
        soc2ip_we_edt = EdgeDetector()
        self.submodules += soc2ip_we_edt

        self.comb += [
            soc2ip_we_edt.i.eq(self.soc2ip_we.storage),
        ]

        # CSR fifo full status
        self.comb += [
            # FIFO full detect implementation (except FIFO stream mode)
            self.soc2ip_full.status.eq((fifo.level >= FIFO_DEPTH) & (reg.FIFO_CONTROL[:2] != 0x02))
        ]

        ################## Active motion detection ##################
        self.abs_act_x = Signal(11)
        self.abs_act_y = Signal(11)
        self.abs_act_z = Signal(11)
        self.time_act = Signal(8)
        self.start_act = Signal(1, reset=1)
        self.thresh_act = Signal(11)
        self.sum_act = Signal(20)

        # Active motion detector threshold
        self.comb += [
            self.thresh_act[0:8].eq(reg.THRESH_ACT_L[0:8]),
            self.thresh_act[8:11].eq(reg.THRESH_ACT_H[0:3]),
        ]

        # Active motion detector timer implementation
        self.sync += [
            If(soc2ip_we_edt.r & reg.ACT_INACT_CTL[0] & (self.time_act < 0xFF),
                self.time_act.eq(self.time_act + 1),
            )
        ]

        # Motion detector behavior implementation
        act_fsm = FSM(reset_state = "IDLE")
        self.submodules += act_fsm

        # Active motion detector state machine behavior implementation
        act_fsm.act("IDLE",
            If(~reg.ACT_INACT_CTL[0],        # ACT_EN[0]
                NextValue(reg.STATUS[4], 0), # Turn off ACT flag
                NextValue(self.start_act, 0),
            ).Elif(self.start_act,
                NextValue(reg.STATUS[4], 0), # Turn off ACT flag
                NextValue(self.time_act, 0),
                NextState("ACT_TRACKING"),
            ),
        )
        act_fsm.act("ACT_TRACKING",
            NextValue(self.start_act, 0),
            If(soc2ip_we_edt.r,
                NextValue(self.abs_act_x, self.soc2ip_dx.storage[0:11]),
                NextValue(self.abs_act_y, self.soc2ip_dy.storage[0:11]),
                NextValue(self.abs_act_z, self.soc2ip_dz.storage[0:11]),
                NextState("ACT_CALCULATING"),
            ),
        )
        act_fsm.act("ACT_CALCULATING",
            If(self.start_act,
                NextState("IDLE"),
            ).Else(
                NextValue(self.sum_act,
                          self.abs_act_x*self.abs_act_x +
                          self.abs_act_y*self.abs_act_y +
                          self.abs_act_z*self.abs_act_z),
                NextState("ACT_CHECKING"),
            )
        )
        act_fsm.act("ACT_CHECKING",
            If(self.start_act,
                NextState("IDLE"),
            ).Elif(~reg.ACT_INACT_CTL[1], # (ACT_REF[1]) Current only supports absolute comparision
                If(self.sum_act > self.thresh_act*self.thresh_act, # x^2 + y^2 + Z^2 > thresh_act^2
                    If(self.time_act >= reg.TIME_ACT, # Active motion detected
                        NextValue(reg.STATUS[4], 1),  # Turn on ACT flag
                        NextState("IDLE"),            # Act detecting finished
                    ).Else(
                        NextState("ACT_TRACKING"),    # I'm on the right way, continue tracking
                    ),
                ).Else(
                    NextValue(self.time_act, 0),      # Oop! restart tracking again
                    NextState("ACT_TRACKING"),
                ),
            ).Else(
                NextState("IDLE"),
            ),
        )

        ################## Inactive motion detection ##################
        self.abs_inact_x = Signal(11)
        self.abs_inact_y = Signal(11)
        self.abs_inact_z = Signal(11)
        self.time_inact = Signal(16)
        self.time_inact_reg = Signal(16)
        self.start_inact = Signal(1, reset=1)
        self.thresh_inact = Signal(11)
        self.sum_inact = Signal(20)

        # Inactive motion detector threshold
        self.comb += [
            self.thresh_inact[0:8].eq(reg.THRESH_INACT_L[0:8]),
            self.thresh_inact[8:11].eq(reg.THRESH_INACT_H[0:3]),
            self.time_inact_reg[0:8].eq(reg.TIME_INACT_L[0:8]),
            self.time_inact_reg[8:16].eq(reg.TIME_INACT_H[0:8]),
        ]

        # Inactive motion detector timer implementation
        self.sync += [
            If(soc2ip_we_edt.r & reg.ACT_INACT_CTL[2] & (self.time_inact < 0xFFFF),
                self.time_inact.eq(self.time_inact + 1),
            )
        ]

        # Inactive motion detector behavior implementation
        inact_fsm = FSM(reset_state = "IDLE")
        self.submodules += inact_fsm

        # Motion detector state machine behavior implementation
        inact_fsm.act("IDLE",
            If(~reg.ACT_INACT_CTL[2],        # INACT_EN[2]
                NextValue(reg.STATUS[5], 0), # Turn off INACT flag
                NextValue(self.start_inact, 0),
            ).Elif(self.start_inact,
                NextValue(reg.STATUS[5], 0), # Turn off INACT flag
                NextValue(self.time_inact, 0),
                NextState("INACT_TRACKING"),
            ),
        )
        inact_fsm.act("INACT_TRACKING",
            NextValue(self.start_inact, 0),
            If(soc2ip_we_edt.r,
                NextValue(self.abs_inact_x, self.soc2ip_dx.storage[0:11]),
                NextValue(self.abs_inact_y, self.soc2ip_dy.storage[0:11]),
                NextValue(self.abs_inact_z, self.soc2ip_dz.storage[0:11]),
                NextState("INACT_CALCULATING"),
            ),
        )
        inact_fsm.act("INACT_CALCULATING",
            If(self.start_inact,
                NextState("IDLE"),
            ).Else(
                NextValue(self.sum_inact,
                          self.abs_inact_x*self.abs_inact_x +
                          self.abs_inact_y*self.abs_inact_y +
                          self.abs_inact_z*self.abs_inact_z),
                NextState("INACT_CHECKING"),
            )
        )
        inact_fsm.act("INACT_CHECKING",
            If(self.start_inact,
                NextState("IDLE"),
            ).Elif(~reg.ACT_INACT_CTL[3], # (INACT_REF[3]) Current only supports absolute comparision
                If(self.sum_inact < self.thresh_inact*self.thresh_inact, # x^2 + y^2 + Z^2 < thresh_inact^2
                    If(self.time_act >= self.time_inact_reg, # Inactive motion detected
                        NextValue(reg.STATUS[5], 1),         # Turn on INACT flag
                        NextState("IDLE"),                   # Act detecting finished
                    ).Else(
                        NextState("INACT_TRACKING"),         # I'm on the right way, continue tracking
                    ),
                ).Else(
                    NextValue(self.time_inact, 0),           # Oop! restart tracking again
                    NextState("INACT_TRACKING"),
                ),
            ).Else(
                NextState("IDLE"),
            ),
        )

        ################## Active/Inactive AWAKE glag efficiency ##################

        # Motion detecting mode FSM implementation
        dtm_fsm = FSM(reset_state = "IDLE")
        self.submodules += dtm_fsm

        dtm_fsm.act("IDLE",
            If(reg.ACT_INACT_CTL[4:6] == 0x01,     # LINK mode
                NextState("START_LINK_MODE"),
            ).Elif(reg.ACT_INACT_CTL[4:6] == 0x03, # LOOP mode
                NextState("START_LOOP_MODE"),
            ).Else(
                NextValue(reg.STATUS[6], 1),       # AWAKE = 1 (default)
            ),
        )
        dtm_fsm.act("START_LOOP_MODE",
            If(reg.STATUS[4],                      # ACT detected
                NextValue(reg.STATUS[6], 1),       # AWAKE = 1
                NextValue(self.start_inact, 1),    # start inactive motion detection
                NextState("INACT_MOTION_DETECT"),
            ).Elif(reg.STATUS[5],                  # INACT detected
                NextValue(reg.STATUS[6], 0),       # AWAKE = 0
                NextValue(self.start_act, 1),      # start active motion detection
                NextState("ACT_MOTION_DETECT"),
            ).Else(
                NextState("IDLE"),
            )
        )
        dtm_fsm.act("INACT_MOTION_DETECT",
            If(reg.STATUS[5],                      # INACT detected
                NextValue(reg.STATUS[6], 0),       # AWAKE = 0
                If(reg.ACT_INACT_CTL[4:6] == 0x03, # LOOP mode
                    NextValue(self.start_act, 1),  # start active motion detection
                ).Else(
                    NextState("IDLE"),
                ),
            ),
        )
        dtm_fsm.act("ACT_MOTION_DETECT",
            If(reg.STATUS[4],                      # INACT detected
                NextValue(reg.STATUS[6], 1),       # AWAKE = 1
                If(reg.ACT_INACT_CTL[4:6] == 0x03, # LOOP mode
                    NextValue(self.start_inact, 1),# start inactive motion detection
                ).Else(
                    NextState("IDLE"),
                ),
            ),
        )
        dtm_fsm.act("START_LINK_MODE",
            If(reg.STATUS[4],                # ACT detected
                NextValue(reg.STATUS[6], 1), # AWAKE = 1
                NextState("WAIT_FOR_ACT_CLEAR"),
            ).Elif(reg.STATUS[5],            # INACT detected
                NextValue(reg.STATUS[6], 0), # AWAKE = 0
                NextState("WAIT_FOR_INACT_CLEAR"),
            ).Else(
                NextState("IDLE"),
            ),
        )
        dtm_fsm.act("WAIT_FOR_ACT_CLEAR",
           If(~reg.STATUS[4],
                NextValue(self.start_inact, 1), # start inactive motion detection
                NextState("IDLE"),
           ),
        )
        dtm_fsm.act("WAIT_FOR_INACT_CLEAR",
           If(~reg.STATUS[5],
                NextValue(self.start_act, 1),   # start active motion detection
                NextState("IDLE"),
           ),
        )

        ################## Write data from CSRs to the accel fifo #################
        ffsm = FSM(reset_state = "IDLE")
        self.submodules += ffsm

        # Copy csr fifo to accel fifo behavior implementation
        ffsm.act("IDLE",
            If(soc2ip_we_edt.r,
                NextValue(self.soc2ip_done.status, 0), # clear done flag
                If(reg.FIFO_CONTROL[:2] == 0x02, # FIFO_MODE = 0x02, stream mode
                    NextValue(fifo.din[0:16], self.soc2ip_dx.storage),
                    NextValue(fifo.din[16:32], self.soc2ip_dy.storage),
                    NextValue(fifo.din[32:48], self.soc2ip_dz.storage),
                    NextValue(fifo.we, 1),
                    NextState("FIFO_WRITE_STROBE"),
                ).Elif(fifo.writable, # Treast as oldest saved mode
                    NextValue(fifo.din[0:16], self.soc2ip_dx.storage),
                    NextValue(fifo.din[16:32], self.soc2ip_dy.storage),
                    NextValue(fifo.din[32:48], self.soc2ip_dz.storage),
                    NextValue(fifo.we, 1),
                    NextState("FIFO_WRITE_STROBE"),
                ),
            ),
        )
        ffsm.act("FIFO_WRITE_STROBE",
            NextValue(fifo.we, 0),
            NextState("FIFO_WRITE"),
        )
        ffsm.act("FIFO_WRITE",
            NextValue(self.soc2ip_done.status, 1), # Set done flag
            NextState("IDLE"),
        )


class ODRController(Module):
    # freg  : sys_clk frequency
    # fbase : based frequency output (after prescaler)
    def __init__(self, freq, fbase=400):
        # Exteral interface signals
        self.ena = Signal()
        self.odr = Signal(3)
        self.foutr = Signal()
        # Module local signals
        self.fout   = Signal()
        self.prescaler = Signal(max=int(freq/(2*fbase))-1)      # 200Hz counting
        self.cnt = Signal(5)
        self.cpt = Signal(5)

        # Programmable divider's parametter definition
        self.comb += [
            Case(self.odr, {
                0:         self.cpt.eq(31), # 12.5 Hz
                1:         self.cpt.eq(15), # 25   Hz
                2:         self.cpt.eq(7),  # 50   Hz
                3:         self.cpt.eq(3),  # 100  Hz
                4:         self.cpt.eq(1),  # 200  Hz
                "default": self.cpt.eq(0),  # 400  Hz
            })
        ]

        # Programmable clock divider implementation
        self.sync += [
            If(self.ena,
                If(self.prescaler >= int(freq/(2*fbase))-1,
                    self.prescaler.eq(0),
                    If(self.odr >= 5,
                        self.fout.eq(~self.fout), # Fout = fbase
                    ).Elif(self.cnt >= self.cpt,
                        self.cnt.eq(0),
                        self.fout.eq(~self.fout), # Fout = fbase/n
                    ).Else(
                        self.cnt.eq(self.cnt + 1),
                    ),
                ).Else(
                    self.prescaler.eq(self.prescaler + 1),
                )
            ).Else(
                self.fout.eq(0),
            )
        ]

        # Generate rissing edge output
        edt = ResetInserter()(EdgeDetector())
        self.submodules += edt

        self.comb += [
            edt.reset.eq(~self.ena),
            edt.i.eq(self.fout),
            self.foutr.eq(edt.r),
        ]

def SpiSlaveTestBench(dut):
    cnt1 = 0
    cnt2 = 0
    for cycle in range(2000):

        if cycle % 2 != 0:
            if cnt1 < 10:
                cnt1 = cnt1 + 1
            else:
                cnt1 = 0
                yield dut.sck.eq(~dut.sck)

        if cnt2 < 20:
            cnt2 = cnt2 + 1
        else:
            cnt2 = 0
            yield dut.mosi.eq(randrange(2))

        if cycle >= 0 and cycle < 2:
            yield dut.csn.eq(1)

        if cycle > 30 and cycle < 32:
            yield dut.csn.eq(0)
            yield dut.txd.eq(0xA5)

        yield

def ReadRegTestBench(dut):
    t = 3  # Number of transfer byte on si line
    u = 5  # Number of total shifted byte
    s = 4  # SCK toggle at cycle 4th
    n = 10 # n cycles per sck toggle
    i = 0
    j = 0
    cmd_addr = 0x0B02

    for cycle in range(1000):
        # Generate si
        if cycle == (s + j*n*2) and j < 2*8*t:
            if (cmd_addr & 0x8000):
                yield dut.mosi.eq(1)
            else:
                yield dut.mosi.eq(0)
            cmd_addr = cmd_addr << 1
            j = j + 1
        # Generate sck
        if cycle == (s + n/2 + i*n) and i < 2*8*u:
            yield dut.sck.eq(~dut.sck)
            i = i + 1
        elif i >= 2*8*u:
            yield dut.csn.eq(1)

        if cycle > 0 and cycle < 3:
            yield dut.csn.eq(1)

        if cycle > 3 and cycle < 5:
            yield dut.csn.eq(0)

        yield

def WriteRegTestBench(dut):
    t = 3  # Number of transfer byte on si line
    u = 3  # Number of total shifted byte
    s = 4  # SCK toggle at cycle 4th
    n = 10 # n cycles per sck toggle
    i = 0
    j = 0
    cmd_addr_data = 0x0A2055

    for cycle in range(1000):
        # Generate si
        if cycle == (s + j*n*2) and j < 2*8*t:
            if (cmd_addr_data & 0x800000):
                yield dut.si.eq(1)
            else:
                yield dut.si.eq(0)
            cmd_addr_data = cmd_addr_data << 1
            j = j + 1
        # Generate sck
        if cycle == (s + n/2 + i*n) and i < 2*8*u:
            yield dut.sck.eq(~dut.sck)
            i = i + 1
        elif i >= 2*8*u:
            yield dut.csn.eq(1)

        if cycle > 1 and cycle < 3:
            yield dut.csn.eq(0)

        yield

def WriteReadRegTestBench(dut):
    n             = 10 # n cycles per sck toggle
    flag1         = 0
    flag2         = 0
    wr_done_cycle = 0
    wr_space      = n  # Gap (cycles) between read/write frames
    cmd_addr_data = 0x0A2055 # Write register
    cmd_addr      = 0x0B20   # Read register

    #################### For write phase ##################
    t  = 3 # Number of transfer byte on si line (write phase)
    u  = 3 # Number of total shifted byte (write phase)
    sw = 6 # SCK toggle at cycle 4th (write phase)
    i  = 0
    j  = 0
    #################### For read phase ###################
    y  = 2 # Number of transfer byte on si line (read phase)
    z  = 3 # Number of total shifted byte (read phase)
    sr = 0 # SCK toggle at cycle 4th (read phase)
    k  = 0
    h  = 0

    for cycle in range(2000):

        #################### Start new write phase ##################
        if cycle >= 0 and cycle < 2:
            yield dut.csn.eq(1)

        if cycle > 3 and cycle < 5:
            yield dut.csn.eq(0)

        # Generate si
        if cycle == (sw + i*n*2) and i < 2*8*t:
            if (cmd_addr_data & 0x800000):
                yield dut.mosi.eq(1)
            else:
                yield dut.mosi.eq(0)
            cmd_addr_data = cmd_addr_data << 1
            i = i + 1

        # Generate sck for write phase
        if cycle == (sw + n/2 + j*n) and j < 2*8*u:
            yield dut.sck.eq(~dut.sck)
            j = j + 1
        elif j == 2*8*u and flag1 == 0:
            print("Write done at cycle: {}".format(cycle))
            wr_done_cycle = cycle
            sr = wr_done_cycle + wr_space
            yield dut.csn.eq(1)
            flag1 = 1

        #################### Start new read phase ##################

        if flag1 == 1:
            if cycle == wr_done_cycle + wr_space/2:
                yield dut.csn.eq(0)
                print("sr = : {}".format(sr))

            # Generate mosi read phase
            if cycle == (sr + k*n*2) and k < 2*8*y:
                if (cmd_addr & 0x8000):
                    yield dut.mosi.eq(1)
                else:
                    yield dut.mosi.eq(0)
                cmd_addr = cmd_addr << 1
                k = k + 1

            # Generate sck for read phase
            if cycle == (sr + n/2 + h*n) and h < 2*8*z:
                yield dut.sck.eq(~dut.sck)
                h = h + 1
            elif h == 2*8*z and flag2 == 0:
                yield dut.csn.eq(1)
                flag2 = 1

        yield

def ODRControllerTestBench(dut):

    for cycle in range(10000):

        if cycle == 2:
            yield dut.ena.eq(1)
            yield dut.odr.eq(3)

        yield

if __name__ == "__main__":

    #dut = AccelCore(freq=50000000, baud=115200)
    #print(verilog.convert(AccelCore(freq=50000000, baud=115200)))
    #run_simulation(dut, WriteRegTestBench(dut), clocks={"sys": 10}, vcd_name="AccelCore.vcd")
    #run_simulation(dut, ReadRegTestBench(dut), clocks={"sys": 10}, vcd_name="AccelCore.vcd")
    #run_simulation(dut, WriteReadRegTestBench(dut), clocks={"sys": 10}, vcd_name="AccelCore.vcd")
    #run_simulation(dut, ReadFiFoTestBench(dut), clocks={"sys": 10}, vcd_name="AccelCore.vcd")
    #run_simulation(dut, UARTWriteFIFOTestBench(dut), clocks={"sys": 10}, vcd_name="AccelCore.vcd")
    #os.system("gtkwave AccelCore.vcd")

    dut = ODRController(freq=10000, fbase=400)
    #print(verilog.convert(ODRController(freq=50000000, fbase=400)))
    run_simulation(dut, ODRControllerTestBench(dut), clocks={"sys": 10}, vcd_name="ODRController.vcd")
    #os.system("gtkwave ODRController.vcd")
  