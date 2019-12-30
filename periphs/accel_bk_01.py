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

from migen import *
from migen.fhdl import verilog
from migen.fhdl.specials import Tristate
from migen.genlib.misc import WaitTimer
from migen.genlib.fifo import SyncFIFOBuffered

from random import randrange

class Debouncer(Module):
    def __init__(self, cycles=1):
        self.i = Signal()
        self.o = Signal()

        timer = WaitTimer(cycles - 1)
        self.submodules += timer
        new = Signal()
        rst = Signal(reset=1)
        self.sync += [
            If(timer.wait,
                If(timer.done,
                    timer.wait.eq(0),
                    new.eq(~new),
                ),
            ).Elif(self.i == new,
                timer.wait.eq(1),
            ),
            If(rst,
                rst.eq(0),
                timer.wait.eq(0),
                new.eq(~self.i),
            ),
        ]
        self.comb += [
            self.o.eq(Mux(timer.wait, new, self.i)),
        ]

class EdgeDetector(Module):
    def __init__(self):
        self.i   = Signal() # Signal input
        self.r   = Signal() # Rising edge detect
        self.f   = Signal() # Falling edge detect
        self.cnt = Signal(2)

        self.comb += [
            self.r.eq(self.cnt == 1),
            self.f.eq(self.cnt == 2),
        ]

        self.sync += [
            self.cnt[1].eq(self.cnt[0]),
            self.cnt[0].eq(self.i),
        ]

class RegisterArray(Module):
    def __init__(self):
        self.addr   = Signal(8)
        self.dw     = Signal(8)
        self.dr     = Signal(8)
        self.r      = Signal()
        self.w      = Signal()

        # Register set
        self.reg0   = Signal(8, reset=0xAD) # DEVID_AD       [R]
        self.reg1   = Signal(8, reset=0x1D) # DEVID_MST      [R]
        self.reg2   = Signal(8, reset=0xF2) # PARTID         [R]
        self.reg3   = Signal(8, reset=0x01) # REVID          [R]

        self.reg8   = Signal(8, reset=0x00) # XDATA          [R] internally writable
        self.reg9   = Signal(8, reset=0x00) # YDATA          [R] internally writable
        self.reg10  = Signal(8, reset=0x00) # ZDATA          [R] internally writable
        self.reg11  = Signal(8, reset=0x40) # STATUS         [R] internally writable
        self.reg12  = Signal(8, reset=0x00) # FIFO_ENTRIES_L [R] internally writable
        self.reg13  = Signal(8, reset=0x00) # FIFO_ENTRIES_H [R] internally writable
        self.reg14  = Signal(8, reset=0x00) # XDATA_L        [R] internally writable
        self.reg15  = Signal(8, reset=0x00) # XDATA_H        [R] internally writable
        self.reg16  = Signal(8, reset=0x00) # YDATA_L        [R] internally writable
        self.reg17  = Signal(8, reset=0x00) # YDATA_H        [R] internally writable
        self.reg18  = Signal(8, reset=0x00) # ZDATA_L        [R] internally writable
        self.reg19  = Signal(8, reset=0x00) # ZDATA_H        [R] internally writable
        self.reg20  = Signal(8, reset=0x00) # TEMP_L         [R] internally writable
        self.reg21  = Signal(8, reset=0x00) # TEMP_H         [R] internally writable

        self.reg31  = Signal(8, reset=0x00) # SOFT_RESET     [W]
        self.reg32  = Signal(8, reset=0x00) # THRESH_ACT_L   [RW]
        self.reg33  = Signal(8, reset=0x00) # THRESH_ACT_H   [RW]
        self.reg34  = Signal(8, reset=0x00) # TIME_ACT       [RW]
        self.reg35  = Signal(8, reset=0x00) # THRESH_INACT_L [RW]
        self.reg36  = Signal(8, reset=0x00) # THRESH_INACT_H [RW]
        self.reg37  = Signal(8, reset=0x00) # TIME_INACT_L   [RW]
        self.reg38  = Signal(8, reset=0x00) # TIME_INACT_H   [RW]
        self.reg39  = Signal(8, reset=0x00) # ACT_INACT_CTL  [RW]
        self.reg40  = Signal(8, reset=0x00) # FIFO_CONTROL   [RW]
        self.reg41  = Signal(8, reset=0x80) # FIFO_SAMPLES   [RW]
        self.reg42  = Signal(8, reset=0x00) # INTMAP1        [RW]
        self.reg43  = Signal(8, reset=0x00) # INTMAP2        [RW]
        self.reg44  = Signal(8, reset=0x13) # FILTER_CTL     [RW]
        self.reg45  = Signal(8, reset=0x00) # POWER_CTL      [RW]
       #self.reg46  = Signal(8, reset=0x00) # SELF_TEST      [RW]

        self.sync += [
            If(self.r,
                Case(self.addr, {
                    0:  self.dr.eq( self.reg0), # DEVID_AD       [R]  [0x00]
                    1:  self.dr.eq( self.reg1), # DEVID_MST      [R]  [0x01]
                    2:  self.dr.eq( self.reg2), # PARTID         [R]  [0x02]
                    3:  self.dr.eq( self.reg3), # REVID          [R]  [0x03]

                    8:  self.dr.eq( self.reg8), # XDATA          [R]  [0x08]
                    9:  self.dr.eq( self.reg9), # YDATA          [R]  [0x09]
                    10: self.dr.eq(self.reg10), # ZDATA          [R]  [0x0A]
                    11: self.dr.eq(self.reg11), # STATUS         [R]  [0x0B]
                    12: self.dr.eq(self.reg12), # FIFO_ENTRIES_L [R]  [0x0C]
                    13: self.dr.eq(self.reg13), # FIFO_ENTRIES_H [R]  [0x0D]
                    14: self.dr.eq(self.reg14), # XDATA_L        [R]  [0x0E]
                    15: self.dr.eq(self.reg15), # XDATA_H        [R]  [0x0F]
                    16: self.dr.eq(self.reg16), # YDATA_L        [R]  [0x10]
                    17: self.dr.eq(self.reg17), # YDATA_H        [R]  [0x11]
                    18: self.dr.eq(self.reg18), # ZDATA_L        [R]  [0x12]
                    19: self.dr.eq(self.reg19), # ZDATA_H        [R]  [0x13]
                    20: self.dr.eq(self.reg20), # TEMP_L         [R]  [0x14]
                    21: self.dr.eq(self.reg21), # TEMP_H         [R]  [0x15]

                   #31: self.dr.eq(self.reg31), # SOFT_RESET     [W]  [0x1F]
                    32: self.dr.eq(self.reg32), # THRESH_ACT_L   [RW] [0x20]
                    33: self.dr.eq(self.reg33), # THRESH_ACT_H   [RW] [0x21]
                    34: self.dr.eq(self.reg34), # TIME_ACT       [RW] [0x22]
                    35: self.dr.eq(self.reg35), # THRESH_INACT_L [RW] [0x23]
                    36: self.dr.eq(self.reg36), # THRESH_INACT_H [RW] [0x24]
                    37: self.dr.eq(self.reg37), # TIME_INACT_L   [RW] [0x25]
                    38: self.dr.eq(self.reg38), # TIME_INACT_H   [RW] [0x26]
                    39: self.dr.eq(self.reg39), # ACT_INACT_CTL  [RW] [0x27]
                    40: self.dr.eq(self.reg40), # FIFO_CONTROL   [RW] [0x28]
                    41: self.dr.eq(self.reg41), # FIFO_SAMPLES   [RW] [0x29]
                    42: self.dr.eq(self.reg42), # INTMAP1        [RW] [0x2A]
                    43: self.dr.eq(self.reg43), # INTMAP2        [RW] [0x2B]
                    44: self.dr.eq(self.reg44), # FILTER_CTL     [RW] [0x2C]
                    45: self.dr.eq(self.reg45), # POWER_CTL      [RW] [0x2D]
                   #46: self.dr.eq(self.reg46), # SELF_TEST      [RW] [0x2E]
             "default": self.dr.eq(0),
                })
            ).Elif(self.w,
                Case(self.addr, {
                   #8:  self.reg8.eq( self.dw), # XDATA          [R]
                   #9:  self.reg9.eq( self.dw), # YDATA          [R]
                   #10: self.reg10.eq(self.dw), # ZDATA          [R]
                   #11: self.reg11.eq(self.dw), # STATUS         [R]
                   #12: self.reg12.eq(self.dw), # FIFO_ENTRIES_L [R]
                   #13: self.reg13.eq(self.dw), # FIFO_ENTRIES_H [R]
                   #14: self.reg14.eq(self.dw), # XDATA_L        [R]
                   #15: self.reg15.eq(self.dw), # XDATA_H        [R]
                   #16: self.reg16.eq(self.dw), # YDATA_L        [R]
                   #17: self.reg17.eq(self.dw), # YDATA_H        [R]
                   #18: self.reg18.eq(self.dw), # ZDATA_L        [R]
                   #19: self.reg19.eq(self.dw), # ZDATA_H        [R]
                   #20: self.reg20.eq(self.dw), # TEMP_L         [R]
                   #21: self.reg21.eq(self.dw), # TEMP_H         [R]

                    31: self.reg31.eq(self.dw), # SOFT_RESET     [W]
                    32: self.reg32.eq(self.dw), # THRESH_ACT_L   [RW]
                    33: self.reg33.eq(self.dw), # THRESH_ACT_H   [RW]
                    34: self.reg34.eq(self.dw), # TIME_ACT       [RW]
                    35: self.reg35.eq(self.dw), # THRESH_INACT_L [RW]
                    36: self.reg36.eq(self.dw), # THRESH_INACT_H [RW]
                    37: self.reg37.eq(self.dw), # TIME_INACT_L   [RW]
                    38: self.reg38.eq(self.dw), # TIME_INACT_H   [RW]
                    39: self.reg39.eq(self.dw), # ACT_INACT_CTL  [RW]
                    40: self.reg40.eq(self.dw), # FIFO_CONTROL   [RW]
                    41: self.reg41.eq(self.dw), # FIFO_SAMPLES   [RW]
                    42: self.reg42.eq(self.dw), # INTMAP1        [RW]
                    43: self.reg43.eq(self.dw), # INTMAP2        [RW]
                    44: self.reg44.eq(self.dw), # FILTER_CTL     [RW]
                    45: self.reg45.eq(self.dw), # POWER_CTL      [RW]
                   #46: self.reg46.eq(self.dw), # SELF_TEST      [RW]
                })
            )
        ]

class AccelCore(Module):
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
        self.led  = Signal(1, reset=1)

        # Internal core signals
        self.miso = Signal()  # Internal miso signal
        self.rxc  = Signal()  # Data RX complete (wire)
        self.rxd  = Signal(8) # RX data

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
        reg = RegisterArray()
        self.submodules += reg

        self.comb += [
            reg.addr.eq(self.bus_addr),
            reg.r.eq(self.bus_r),
            reg.w.eq(self.bus_w),
            reg.dw.eq(self.bus_dw),
            self.bus_dr.eq(reg.dr),
        ]

        # Connect to the FIFO buffer
        fifo = SyncFIFOBuffered(width=8, depth=1024)
        self.submodules += fifo

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
            )
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
                NextValue(self.led, 0),
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
                NextValue(self.bus_addr, self.str_addr),
                NextValue(self.bus_r, 1),
                NextState("LOAD_SHIFT_OUT_DATA"),
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
            If(fifo.readable,
                NextValue(fifo.re, 1),
                NextState("READ_FIFO_STROBE"),
            ).Else( #FIFO is empty
                NextState("IDLE"),
            )
        )
        fsm.act("READ_FIFO_STROBE",
            NextValue(fifo.re, 0),
            NextValue(self.tx_buf, fifo.dout),
            NextState("FIFO_SHIFTING_OUT"),
        )
        fsm.act("FIFO_SHIFTING_OUT",
            If(self.rxc,
                NextState("READ_FIFO"),
            )
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


        ############## Integrate UART submodules ###################

        uart = UART(freq=freq, baud=baud)
        self.submodules += uart

        # Expose UART pins
        self.comb += [
            pads.tx.eq(uart.tx),
            uart.rx.eq(pads.rx),
        ]

        # Write data from UART to FIFO if possible
        uffsm = FSM(reset_state = "NORMAL")
        self.submodules += uffsm

        uffsm.act("NORMAL",
            If(fifo.writable & uart.readable,
                If(reg.reg40[:2] == 0x02, # FIFO_MODE = 0x02, stream mode
                    If(fifo.level >= 1024,
                       NextValue(fifo.re, 1), # Kick out the oldest element
                       NextState("KICK_OUT"),
                    ).Else(
                        NextValue(fifo.din, uart.dout),
                        NextValue(fifo.we, 1),
                    ),
                ).Elif(fifo.level <= 1023, # Treast as oldest saved mode
                    NextValue(fifo.din, uart.dout),
                    NextValue(fifo.we, 1),
                ),
            ).Else(
                NextValue(fifo.we, 0),
            )
        )
        uffsm.act("KICK_OUT",
            NextValue(fifo.din, uart.dout), # Write the newest element to the FIFO
            NextValue(fifo.we, 1),
            NextState("NORMAL"),
        )

        ########### Accel behavior implementation #################
        self.fifo_h_level = Signal(max=512)
        self.fifo_samples = Signal(9, reset=0x80)

        self.comb += [
            self.fifo_h_level.eq(fifo.level[1:]),    # fifo.level / 2
            reg.reg12.eq(self.fifo_h_level[:8]),     # FIFO_ENTRIES_L
            reg.reg13.eq(self.fifo_h_level[8:]),     # FIFO_ENTRIES_H
            self.fifo_samples[:8].eq(reg.reg41[:8]), # FIFO_SAMPLES (LSB)
            self.fifo_samples[8:].eq(reg.reg40[3]),  # FIFO_SAMPLES (MSB) = FIFO_CONTROL[3] (AH)
        ]

        self.sync += [
            reg.reg11[3].eq(fifo.level>=1024),       # FIFO_OVERRUN
            If(self.fifo_h_level >= self.fifo_samples,
                reg.reg11[2].eq(1),                  # FIFO_WATERMARK is set
            ),
            If(self.fifo_h_level == 0,
                reg.reg11[2].eq(0),                  # FIFO_WATERMARK is cleared
            ),
            reg.reg11[1].eq(fifo.level>=6),          # FIFO_READY (at least one valid sample in the FIFO buffer)
            reg.reg11[0].eq(fifo.level>=6),          # DATA_READY (new valid sample available) (not implement)
        ]

        # Add output data rate controller
        odrctrl = ODRController(freq=freq, fbase=400)
        self.submodules += odrctrl

        # Connect to ODR parametter input
        self.comb += [
            odrctrl.odr.eq(reg.reg44[:3]),           # ODR[2:0] (FILTER_CTL)
        ]

        # ODR controller
        self.sync += [
            If(reg.reg45[:2] == 0x02,                # MEASURE[1:0] = 0x02 (POWER_CTL)
                If(reg.reg11[2],                     # And FIFO_WATERMARK is cleared
                    odrctrl.ena.eq(0)
                ).Else(
                    odrctrl.ena.eq(1)
                ),
            ).Else(
                odrctrl.ena.eq(0),
            ),
            If(odrctrl.foutr,
                uart.din.eq(0x52),                   # Send request (R) to get more data from host PC
                uart.tx_start.eq(1),                 # Send request
            ),
        ]

        # Interrupt signaling
        self.comb += [
            # Both 2 bits FIFO_WATERMARK in STATUS & INTMAP1 are set
            If(~reg.reg42[7],
                pads.int1.eq(reg.reg42[2] & reg.reg11[2]),   # Active high
            ).Else(
                pads.int1.eq(~reg.reg42[2] | ~reg.reg11[2]), # Active low
            ),
            # Both 2 bits FIFO_WATERMARK in STATUS & INTMAP2 are set
            If(~reg.reg43[7],
                pads.int2.eq(reg.reg43[2] & reg.reg11[2]),   # Active high
            ).Else(
                pads.int2.eq(~reg.reg43[2] | ~reg.reg11[2]), # Active low
            )
        ]

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
        self.prescaler = Signal(max=int(freq/fbase)-1)
        self.cnt = Signal(max=int(fbase/12.5)-1)
        self.cpt = Signal(max=int(fbase/12.5)-1)

        # Programmable divider's parametter definition
        self.comb += [
            Case(self.odr, {
                0:         self.cpt.eq(int(fbase/(12.5*1))-1),  # 12.5 Hz
                1:         self.cpt.eq(int(fbase/(12.5*2))-1),  # 25   Hz
                2:         self.cpt.eq(int(fbase/(12.5*4))-1),  # 50   Hz
                3:         self.cpt.eq(int(fbase/(12.5*8))-1),  # 100  Hz
                4:         self.cpt.eq(int(fbase/(12.5*16))-1), # 200  Hz
                "default": self.cpt.eq(0),                      # 400  Hz
            })
        ]

        # Programmable clock divider implementation
        self.sync += [
            If(self.ena,
                If(self.prescaler == int(freq/fbase)-1,
                    self.prescaler.eq(0),
                    If(self.odr >= 5,
                        self.fout.eq(~self.fout), # Fout = fbase
                    ).Elif(self.cnt == self.cpt,
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

class SyncFIFOTest(Module):
    def __init__(self, width, depth):
        self.din = Signal(width)
        self.dout = Signal(width)
        self.level = Signal(max=depth+2)
        self.writable = Signal()
        self.readable = Signal()
        self.we = Signal()
        self.re = Signal()

        fifo = SyncFIFOBuffered(width, depth)
        self.submodules += fifo

        self.comb += [
            fifo.din.eq(self.din),
            self.dout.eq(fifo.dout),
            self.level.eq(fifo.level),
            self.writable.eq(fifo.writable),
            self.readable.eq(fifo.readable),
            fifo.we.eq(self.we),
            fifo.re.eq(self.re),
        ]

class UART(Module):
    # freg  : sys_clk frequency
    # baud  : Uart baud rate
    # ratio : Over sampling ratio (choose 100)
    def __init__(self, freq, baud=115200, ratio=100):
        # Exteral interface signals
        self.tx = Signal(1, reset=1)
        self.rx = Signal()
        # Internal interface signals
        self.din = Signal(8)
        self.dout = Signal(8)
        self.writable = Signal() # Assert indicates din can be written
        self.readable = Signal()  # Assert inditated dout can be read
        self.tx_start = Signal()
        # Module local signals
        self.prescaler = Signal(max=int(freq/(baud*ratio))-1)
        self.rxcount = Signal(max=ratio-1)
        self.txcount = Signal(max=ratio-1)
        self.rxsampling = Signal()
        self.txshifting = Signal()
        self.rx_ena = Signal()
        self.tx_ena = Signal()
        self.rxbitcnt = Signal(max=9)
        self.txbitcnt = Signal(max=10)

        # To detect rising/falling edge of rx pin
        edt = EdgeDetector()
        self.submodules += edt

        # RX edge detecting
        self.comb += [
            edt.i.eq(self.rx)
        ]

        # Sampling behavior
        self.sync += [
            If(self.prescaler == int(freq/(baud*ratio))-1,
                self.prescaler.eq(0),
                # TX sampling strobe generation
                If(self.tx_ena,
                    If(self.txcount == ratio-1,
                        self.txcount.eq(0),
                        self.txshifting.eq(1),
                    ).Else(
                        self.txshifting.eq(0),
                        self.txcount.eq(self.txcount + 1),
                    ),
                ),
                # RX sampling strobe generation
                If(self.rx_ena,
                    If(self.rxcount == ratio-1,
                        self.rxcount.eq(0),
                        self.rxsampling.eq(1),
                    ).Else(
                        self.rxsampling.eq(0),
                        self.rxcount.eq(self.rxcount + 1),
                    ),
                ),
            ).Else(
                self.prescaler.eq(self.prescaler + 1),
                self.txshifting.eq(0),
                self.rxsampling.eq(0),
            )
        ]

        # RX submodule FSM handles data in/out activities
        rxfsm = FSM(reset_state = "IDLE")
        self.submodules += rxfsm

        # RX FSM behavior description
        rxfsm.act("IDLE",
            If(edt.f, # START condition
                NextValue(self.dout, 0),
                NextValue(self.rx_ena, 1),
                NextValue(self.rxcount, int(ratio/2)-1),
                NextValue(self.rxbitcnt, 0),
                NextState("START"),
            )
        )
        rxfsm.act("START",
            If(self.rxsampling,
                If(~self.rx,
                    NextState("GET_BITS"),
                ).Else(
                    NextValue(self.rx_ena, 0),
                    NextState("IDLE"),
                )
            )
        )
        rxfsm.act("GET_BITS",
            If(self.rxsampling,
                If(self.rxbitcnt < 8,
                    NextValue(self.dout[7], self.rx),
                    NextValue(self.dout, self.dout >> 1),
                ),
                If(self.rxbitcnt == 8,
                    # Shift bit
                    NextValue(self.rxbitcnt, self.rxbitcnt + 1),
                ).Elif(self.rxbitcnt == 9, # STOP bit condition
                    NextValue(self.rx_ena, 0),
                    NextState("IDLE"),
                ).Else(
                   NextValue(self.rxbitcnt, self.rxbitcnt + 1),
                )
            )
        )

        # TX submodule FSM handles data in/out activities
        txfsm = FSM(reset_state = "IDLE")
        self.submodules += txfsm

        # TX FSM behavior description
        txfsm.act("IDLE",
            If(self.tx_start, # START condition
                NextValue(self.tx_start, 0),
                NextValue(self.tx_ena, 1),
                NextValue(self.tx, 0),
                NextValue(self.txcount, int(ratio)-1),
                NextValue(self.txbitcnt, 0),
                NextState("START"),
            )
        )
        txfsm.act("START",
            If(self.txshifting,
                NextState("SHIFT_OUT"),
            )
        )
        txfsm.act("SHIFT_OUT",
            If(self.txshifting,
                If(self.txbitcnt < 8,
                    NextValue(self.tx, self.din[0]),
                    NextValue(self.din, self.din >> 1),
                ),
                If(self.txbitcnt == 8,
                    # Shift bit
                    NextValue(self.txbitcnt, self.txbitcnt + 1),
                    NextValue(self.tx, 1),
                ).Elif(self.txbitcnt == 10, # STOP bit + IDLE
                    NextValue(self.tx_ena, 0),
                    NextState("IDLE"),
                ).Else(
                   NextValue(self.txbitcnt, self.txbitcnt + 1),
                )
            )
        )

        # Status signal notification
        self.comb += [
            self.readable.eq(self.rxsampling & (self.rxbitcnt == 8)),
            self.writable.eq(~self.tx_ena),
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

def UARTWriteFIFOTestBench(dut):

    r = 100  # Over sampling ratio
    m = 4    # Prescaler ratio
    s = 12   # RXD goes low at cycle 12th
    data = 0 # Received data
    i = 0
    setup_pos = 0

    for cycle in range(14000):

        ###### Receive byte 1 #########
        if cycle == 0:
            yield dut.rx.eq(1)

        if cycle == 12:
            yield dut.rx.eq(0)
            s = 12
            data = 0x12A # Data received 0x2A
            i = 0
            setup_pos = 0

        ###### Receive byte 2 #########
        if cycle == 4200:
            yield dut.rx.eq(1)

        if cycle == 4500:
            yield dut.rx.eq(0)
            s = 4500
            data = 0x117 # Data received 0x17
            i = 0
            setup_pos = 0

        ###### Receive byte 3 #########
        if cycle == 8000:
            yield dut.rx.eq(1)

        if cycle == 8500:
            yield dut.rx.eq(0)
            s = 8500
            data = 0x1A5 # Data received 0xA5
            i = 0
            setup_pos = 0

        ###### generate rx waveform #########
        if cycle == s + m*(r/2+1) + m*(r+1)*i:
            sampling = cycle
            setup_pos = sampling + m*(r/2+1)
            i = i + 1

        if cycle == setup_pos and i > 0:
            if i < 9:
                if data & 0x01:
                    yield dut.rx.eq(1)
                else:
                    yield dut.rx.eq(0)
                data = data >> 1
            else:
                yield dut.rx.eq(1)

        yield

def SyncFIFOTestTestBench(dut):

    for cycle in range(1000):

        if cycle == 3:
            yield dut.din.eq(0xA3)

        if cycle == 4:
            yield dut.din.eq(0xA4)

        if cycle == 5:
            yield dut.din.eq(0xA5)

        if cycle == 6:
            yield dut.din.eq(0x00)

        if cycle == 3:
            yield dut.we.eq(1)

        if cycle == 5:
            yield dut.we.eq(1)

        if cycle == 6:
            yield dut.we.eq(0)

        if cycle > 7:
            yield dut.re.eq(1)
        else:
            yield dut.re.eq(0)

        yield

def UARTTestBench(dut):

    r = 100  # Over sampling ratio
    m = 4    # Prescaler ratio
    s = 12   # RXD goes low at cycle 12th
    data = 0 # Received data
    i = 0
    setup_pos = 0

    for cycle in range(10000):

        ###### Receive byte 1 #########
        if cycle == 0:
            yield dut.rx.eq(1)

        if cycle == 12:
            yield dut.rx.eq(0)
            s = 12
            data = 0x12A # Data received 0x2A
            i = 0
            setup_pos = 0

        ###### Receive byte 2 #########
        if cycle == 4200:
            yield dut.rx.eq(1)

        if cycle == 4500:
            yield dut.rx.eq(0)
            s = 4500
            data = 0x117 # Data received 0x17
            i = 0
            setup_pos = 0

        ###### generate rx waveform #########
        if cycle == s + m*(r/2+1) + m*(r+1)*i:
            sampling = cycle
            setup_pos = sampling + m*(r/2+1)
            i = i + 1

        if cycle == setup_pos and i > 0:
            if i < 9:
                if data & 0x01:
                    yield dut.rx.eq(1)
                else:
                    yield dut.rx.eq(0)
                data = data >> 1
            else:
                yield dut.rx.eq(1)

        yield

def ODRControllerTestBench(dut):

    for cycle in range(10000):

        if cycle == 20:
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

    #dut = SyncFIFOTest(width=8, depth=2)
    #print(verilog.convert(SyncFIFOTest(width=8, depth=32)))
    #run_simulation(dut, SyncFIFOTestTestBench(dut), clocks={"sys": 10}, vcd_name="SyncFIFOTest.vcd")
    #os.system("gtkwave SyncFIFOTest.vcd")

    #dut = UART(freq=50000000, baud=115200)
    #print(verilog.convert(UART(freq=50000000, baud=115200)))
    #run_simulation(dut, UARTTestBench(dut), clocks={"sys": 10}, vcd_name="UART.vcd")

    dut = ODRController(freq=8000, fbase=400)
    #print(verilog.convert(ODRController(freq=50000000, fbase=400)))
    run_simulation(dut, ODRControllerTestBench(dut), clocks={"sys": 10}, vcd_name="ODRController.vcd")
    #os.system("gtkwave ODRController.vcd")
  