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
from migen.genlib.misc import WaitTimer
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
        self.s = Signal() # Signal input
        self.d = Signal() # Delay signal output
        self.e = Signal() # e = s ^ d
        self.r = Signal() # Rising edge detect
        self.f = Signal() # Falling edge detect

        self.sync += [
            self.d.eq(self.s)
        ]

        self.comb += [
            self.e.eq(self.d ^ self.s),
            self.r.eq(self.e & self.s),
            self.f.eq(self.e & ~self.s),
        ]

class ShifterIn(Module):
    def __init__(self):
        self.sck     = Signal()
        self.si      = Signal()
        self.start   = Signal()
        self.rising  = Signal()
        self.falling = Signal()
        self.done    = Signal()
        self.dout    = Signal(8)
        self.cnt     = Signal(4)

        fsm = FSM(reset_state = "IDLE")
        edt = EdgeDetector()

        self.submodules += fsm

        fsm.act("IDLE",
            If(self.start == 1,
                NextValue(self.cnt, 0x00),
                NextState("SHIFTING"),
            )
        )
        fsm.act("SHIFTING",
            If(self.cnt == 8,
                NextValue(self.done, 1),
                NextState("IDLE"),
            ),
            If(self.rising,
                If(self.cnt < 8,
                    NextValue(self.cnt, self.cnt + 1),
                    If(self.si,
                        NextValue(self.dout[0], 1),
                    ).Else(
                        NextValue(self.dout[0], 0),
                    )
                )
            ),
            If(self.falling,
                If(self.cnt < 8,
                    NextValue(self.dout, self.dout << 1)
                )
            )
        )

        self.sync += [
            If(self.rising, self.start.eq(0)),
            If(self.falling, self.done.eq(0))
        ]

class ShifterOut(Module):
    def __init__(self):
        self.sck     = Signal()
        self.so      = Signal()
        self.start   = Signal()
        self.rising  = Signal()
        self.falling = Signal()
        self.done    = Signal()
        self.din     = Signal(8)
        self.cnt     = Signal(4)

        fsm = FSM(reset_state = "IDLE")
        edt = EdgeDetector()

        self.submodules += fsm

        fsm.act("IDLE",
            If(self.start == 1,
                NextValue(self.cnt, 0x00),
                NextState("SHIFTING"),
            )
        )
        fsm.act("SHIFTING",
            If(self.cnt == 8,
                NextValue(self.done, 1),
                NextValue(self.so, 0),
                NextState("IDLE"),
            ),
            If(self.rising,
                If(self.cnt < 8, # Hold
                    NextValue(self.cnt, self.cnt + 1),
                    NextValue(self.din, self.din << 1)
                ).Else(
                    NextValue(self.so, 0)
                )
            ),
            If(self.falling,
                If(self.cnt < 8, # Setup
                    If(self.din[7],
                        NextValue(self.so, 1),
                    ).Else(
                        NextValue(self.so, 0),
                    )
                )
            )
        )

        self.sync += [
            If(self.falling, self.start.eq(0)),
            If(self.falling, self.done.eq(0))
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

        self.reg8   = Signal(8, reset=0x00) # XDATA          [R] internally writeable
        self.reg9   = Signal(8, reset=0x00) # YDATA          [R] internally writeable
        self.reg10  = Signal(8, reset=0x00) # ZDATA          [R] internally writeable
        self.reg11  = Signal(8, reset=0x40) # STATUS         [R] internally writeable
        self.reg12  = Signal(8, reset=0x00) # FIFO_ENTRIES_L [R] internally writeable
        self.reg13  = Signal(8, reset=0x00) # FIFO_ENTRIES_H [R] internally writeable
        self.reg14  = Signal(8, reset=0x00) # XDATA_L        [R] internally writeable
        self.reg15  = Signal(8, reset=0x00) # XDATA_H        [R] internally writeable
        self.reg16  = Signal(8, reset=0x00) # YDATA_L        [R] internally writeable
        self.reg17  = Signal(8, reset=0x00) # YDATA_H        [R] internally writeable
        self.reg18  = Signal(8, reset=0x00) # ZDATA_L        [R] internally writeable
        self.reg19  = Signal(8, reset=0x00) # ZDATA_H        [R] internally writeable
        self.reg20  = Signal(8, reset=0x00) # TEMP_L         [R] internally writeable
        self.reg21  = Signal(8, reset=0x00) # TEMP_H         [R] internally writeable

        self.reg31  = Signal(8, reset=0x00) # SOFT_RESET     [W]
        self.reg32  = Signal(8, reset=0x00) # THRESH_ACT_L   [RW]
        self.reg33  = Signal(8, reset=0x00) # THRESH_ACT_H   [RW]
        self.reg34  = Signal(8, reset=0x00) # TIME_ACT       [RW]
        self.reg35  = Signal(8, reset=0x00) # THRESH_INACT_L [RW]
        self.reg36  = Signal(8, reset=0x00) # THRESH_INACT_H [RW]
        self.reg37  = Signal(8, reset=0x00) # TIME_INACT_L   [RW]
        self.reg38  = Signal(8, reset=0x00) # TIME_INACT_H   [RW]
        self.reg39  = Signal(8, reset=0x00) # ACT_INACT_CTL  [RW]
        self.reg40  = Signal(8, reset=0x00) # FIFO_CONTRO    [RW]
        self.reg41  = Signal(8, reset=0x00) # FIFO_SAMPLES   [RW]
        self.reg42  = Signal(8, reset=0x00) # INTMAP1        [RW]
        self.reg43  = Signal(8, reset=0x00) # INTMAP2        [RW]
        self.reg44  = Signal(8, reset=0x13) # FILTER_CTL     [RW]
        self.reg45  = Signal(8, reset=0x40) # POWER_CTL      [RW]
        self.reg46  = Signal(8, reset=0x00) # SELF_TEST      [RW]

        self.sync += [
            If(self.r,
                Case(self.addr, {
                    0:  self.dr.eq( self.reg0),
                    1:  self.dr.eq( self.reg1),
                    2:  self.dr.eq( self.reg2),
                    3:  self.dr.eq( self.reg3),

                    8:  self.dr.eq( self.reg8),
                    9:  self.dr.eq( self.reg9),
                    10: self.dr.eq(self.reg10),
                    11: self.dr.eq(self.reg11),
                    12: self.dr.eq(self.reg12),
                    13: self.dr.eq(self.reg13),
                    14: self.dr.eq(self.reg14),
                    15: self.dr.eq(self.reg15),
                    16: self.dr.eq(self.reg16),
                    17: self.dr.eq(self.reg17),
                    18: self.dr.eq(self.reg18),
                    19: self.dr.eq(self.reg19),
                    20: self.dr.eq(self.reg20),
                    21: self.dr.eq(self.reg21),

                    32: self.dr.eq(self.reg32),
                    33: self.dr.eq(self.reg33),
                    34: self.dr.eq(self.reg34),
                    35: self.dr.eq(self.reg35),
                    36: self.dr.eq(self.reg36),
                    37: self.dr.eq(self.reg37),
                    38: self.dr.eq(self.reg38),
                    39: self.dr.eq(self.reg39),
                    40: self.dr.eq(self.reg40),
                    41: self.dr.eq(self.reg41),
                    42: self.dr.eq(self.reg42),
                    43: self.dr.eq(self.reg43),
                    44: self.dr.eq(self.reg44),
                    45: self.dr.eq(self.reg45),
                    46: self.dr.eq(self.reg46),
                })
            ).Elif(self.w,
                Case(self.addr, {
                    8:  self.reg8.eq( self.dw),
                    9:  self.reg9.eq( self.dw),
                    10: self.reg10.eq(self.dw),
                    11: self.reg11.eq(self.dw),
                    12: self.reg12.eq(self.dw),
                    13: self.reg13.eq(self.dw),
                    14: self.reg14.eq(self.dw),
                    15: self.reg15.eq(self.dw),
                    16: self.reg16.eq(self.dw),
                    17: self.reg17.eq(self.dw),
                    18: self.reg18.eq(self.dw),
                    19: self.reg19.eq(self.dw),
                    20: self.reg20.eq(self.dw),
                    21: self.reg21.eq(self.dw),

                    31: self.reg31.eq(self.dw),
                    32: self.reg32.eq(self.dw),
                    33: self.reg33.eq(self.dw),
                    34: self.reg34.eq(self.dw),
                    35: self.reg35.eq(self.dw),
                    36: self.reg36.eq(self.dw),
                    37: self.reg37.eq(self.dw),
                    38: self.reg38.eq(self.dw),
                    39: self.reg39.eq(self.dw),
                    40: self.reg40.eq(self.dw),
                    41: self.reg41.eq(self.dw),
                    42: self.reg42.eq(self.dw),
                    43: self.reg43.eq(self.dw),
                    44: self.reg44.eq(self.dw),
                    45: self.reg45.eq(self.dw),
                    46: self.reg46.eq(self.dw),
                })
            )
        ]

class AccelCore(Module):
    def __init__(self):
        # Physical pin signals
        self.sck     = Signal()
        self.so      = Signal()
        self.si      = Signal()
        self.csn     = Signal(1, reset=1)

        # Input signals interface
        self.dout    = Signal(8)
        self.si_done = Signal()
        self.so_done = Signal()
        self.start   = Signal()

        # Output signals interface
        self.addr     = Signal(8)
        self.dw       = Signal(8)
        self.dr       = Signal(8)
        self.r        = Signal()
        self.w        = Signal()

        # Internal registers
        self.str_addr = Signal(8)
        self.str_cmd  = Signal(8)
        self.cnt      = Signal(8)
        self.reg_done = Signal()

        # Define edgedetecter, shifter in/out
        edt1 = ResetInserter()(EdgeDetector())
        edt2 = EdgeDetector()
        sti  = ResetInserter()(ShifterIn())
        sto  = ResetInserter()(ShifterOut())
        self.submodules += edt1, edt2, sti, sto

        # Connect to edgedetecter, shifter in/out
        self.comb += [
            edt1.s.eq(self.sck),
            sti.rising.eq(edt1.r),
            sti.falling.eq(edt1.f),
            sto.rising.eq(edt1.r),
            sto.falling.eq(edt1.f),
            sti.sck.eq(self.sck),
            sto.sck.eq(self.sck),
            sti.si.eq(self.si),
            self.so.eq(sto.so),
            edt2.s.eq(self.csn),
        ]

        # Connect to register set
        reg = RegisterArray()
        self.submodules += reg

        self.comb += [
            reg.addr.eq(self.addr),
            reg.r.eq(self.r),
            reg.w.eq(self.w),
            reg.dw.eq(self.dw),
            self.dr.eq(reg.dr),
        ]

        # Submodule FSM
        fsm = ResetInserter()(FSM(reset_state = "IDLE"))
        self.submodules += fsm

        # Make sure csn can reset submodules
        self.comb += [
            fsm.reset.eq(self.csn),
            edt1.reset.eq(self.csn),
            sti.reset.eq(self.csn),
            sto.reset.eq(self.csn),
        ]

        fsm.act("IDLE",
            If(edt2.f, # csn falling edge
                NextValue(self.cnt, 0),
                NextState("START"),
            )
        )
        fsm.act("START",
            If(self.cnt >= 2,
                NextValue(self.start, 1),
            ).Else(
                 NextValue(self.cnt, self.cnt + 1),
            ),
            If(self.start,
                NextValue(self.start, 0),
                NextValue(sti.done, 0),
                NextValue(sti.start, 1),
                NextState("GET_COMMAND"),
            )
        )
        fsm.act("GET_COMMAND",
            If(sti.done,
                NextValue(self.str_cmd, sti.dout),
                NextState("CMD_DECODE"),
            )
        )
        fsm.act("CMD_DECODE",
            If(self.str_cmd == 0x0A, # Reg write
                NextValue(sti.done, 0),
                NextValue(sti.start, 1),
                NextState("REG_ADDR"),
            ).Elif(self.str_cmd == 0x0B, # Reg read
                NextValue(sti.done, 0),
                NextValue(sti.start, 1),
                NextState("REG_ADDR"),
            ).Elif(self.str_cmd == 0x0D, # FIFO read
                NextState("READ_FIFO"),
            ).Else(
                NextState("IDLE"),
            )
        )
        fsm.act("REG_ADDR",
            If(sti.done,
                NextValue(self.str_addr, sti.dout),
                NextState("DETERMINE_REG_ACCESS"),
            )
        )
        fsm.act("DETERMINE_REG_ACCESS",
            If(self.str_cmd == 0x0A, # Reg write
                NextValue(sti.done, 0),
                NextValue(sti.start, 1),
                NextState("REG_VALUE_SHIFTIN"),
            ).Elif(self.str_cmd == 0x0B, # Reg read
                NextValue(self.addr, self.str_addr),
                NextValue(self.r, 1),
                NextState("REG_READ_STROBE"),
            ).Else(
                NextState("IDLE"),
            )
        )
        fsm.act("REG_VALUE_SHIFTIN",
            If(sti.done,
                NextValue(self.addr, self.str_addr),
                NextValue(self.dw, sti.dout),
                NextValue(self.w, 1),
                NextState("REG_WRITE_STROBE"),
            )
        )
        fsm.act("REG_WRITE_STROBE",
            NextState("REG_WRITE_VALUE"),
        )
        fsm.act("REG_WRITE_VALUE",
            NextValue(self.w, 0),
            NextState("IDLE"),
        )
        fsm.act("REG_READ_STROBE",
            NextState("LOAD_SHIFT_OUT_DATA"),
        )
        fsm.act("LOAD_SHIFT_OUT_DATA",
            NextValue(sto.din, self.dr),
            NextState("START_SHIFT_OUT"),
        )
        fsm.act("START_SHIFT_OUT",
            NextValue(self.r, 0),
            NextValue(sto.done, 0),
            NextValue(sto.start, 1),
            NextState("SHIFTING_OUT"),
        )
        fsm.act("SHIFTING_OUT",
            If(sto.done,
                NextState("SHIFT_OUT_DONE"),
            )
        )
        fsm.act("SHIFT_OUT_DONE",
            If(edt2.r | self.csn, # csn rising edge
                NextState("IDLE"),
            ).Elif(self.addr < 0x2D,
                NextValue(self.addr, self.addr + 1),
                NextValue(self.r, 1),
                NextState("REG_READ_STROBE"),
            ).Else(
                NextState("IDLE"),
            )
        )
        fsm.act("READ_FIFO",

        )

def ReadRegTestBench(dut):
    t = 3  # Number of transfer byte on si line
    u = 5  # Number of total shifted byte
    s = 4  # SCK toggle at cycle 4th
    n = 10 # n cycles per sck toggle
    i = 0
    j = 0
    cmd_addr = 0x0B00

    for cycle in range(1000):
        # Generate si
        if cycle == (s + j*n*2) and j < 2*8*t:
            if (cmd_addr & 0x8000):
                yield dut.si.eq(1)
            else:
                yield dut.si.eq(0)
            cmd_addr = cmd_addr << 1
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
    sw = 4 # SCK toggle at cycle 4th (write phase)
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
        if cycle > 1 and cycle < 3:
            yield dut.csn.eq(0)

        # Generate si
        if cycle == (sw + i*n*2) and i < 2*8*t:
            if (cmd_addr_data & 0x800000):
                yield dut.si.eq(1)
            else:
                yield dut.si.eq(0)
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

            # Generate si read phase
            if cycle == (sr + k*n*2) and k < 2*8*y:
                if (cmd_addr & 0x8000):
                    yield dut.si.eq(1)
                else:
                    yield dut.si.eq(0)
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

if __name__ == "__main__":
    dut = AccelCore()
    print(verilog.convert(AccelCore()))
    #run_simulation(dut, ReadRegTestBench(dut), clocks={"sys": 10}, vcd_name="AccelCore.vcd")
    #run_simulation(dut, WriteRegTestBench(dut), clocks={"sys": 10}, vcd_name="AccelCore.vcd")
    #run_simulation(dut, WriteReadRegTestBench(dut), clocks={"sys": 10}, vcd_name="AccelCore.vcd")
    #os.system("gtkwave AccelCore.vcd")
    