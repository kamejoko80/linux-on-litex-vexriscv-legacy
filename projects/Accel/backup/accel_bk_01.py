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
        self.i   = Signal() # Signal input
        self.r   = Signal() # Rising edge detect
        self.f   = Signal() # Falling edge detect
        self.csn = Signal() # SPI csn pin
        self.cnt = Signal(2)
        
        self.comb += [
            self.r.eq(self.cnt == 1),
            self.f.eq(self.cnt == 2),
        ]
        
        self.sync += [
            If(self.csn,
               self.cnt.eq(0),
            ).Else(
               self.cnt[1].eq(self.cnt[0]),
               self.cnt[0].eq(self.i),                
            )
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

class SpiSlave(Module):
    def __init__(self):
        # Physical pins interface
        self.sck  = Signal()  # SCK pin input
        self.mosi = Signal()  # MOSI pin input
        self.miso = Signal()  # MISO pin output
        self.csn  = Signal()  # CSN pin input   
        
        # Internal core signals
        self.rxc  = Signal()  # Data RX complete
        self.txr  = Signal()  # Data TX request
        self.rxd  = Signal(8) # RX data
        self.txd  = Signal(8, reset=0xA5) # TX data
      
        # Misc signals
        self.sck_cnt  = Signal(2) # SCK edge detect counter
        self.sck_r    = Signal()  # SCK rising edge detect signal
        self.sck_f    = Signal()  # SCK falling edge detect signal         
        self.mosi_cnt = Signal(2) # MOSI edge detect counter      
        self.bitcnt   = Signal(3) # Bit count
        self.mosi_s   = Signal()  # MOSI sample
        self.tx_buf   = Signal(8) # TX data buffer 
        
        # Edge detect signal combinatorial
        self.comb += [
            self.sck_r.eq(self.sck_cnt == 1),
            self.sck_f.eq(self.sck_cnt == 2),
        ]
        
        # MOSI data sampling
        self.comb += [
            self.mosi_s.eq(self.mosi_cnt[1]),
        ]

        # Need debouncer to get better rising/falling detection
        sck_db  = Debouncer(cycles=2)
        mosi_db = Debouncer(cycles=2)
        self.submodules += sck_db, mosi_db
        
        # Connect to debouncer I/O
        self.comb += [
            sck_db.i.eq(self.sck),
            mosi_db.i.eq(self.mosi),
        ]
        
        # Edge detector behavior description
        self.sync += [
            If(self.csn,
                self.sck_cnt.eq(0),
                self.mosi_cnt.eq(0),
            ).Else(
                self.sck_cnt[1].eq(self.sck_cnt[0]),
                self.sck_cnt[0].eq(sck_db.o),
                self.mosi_cnt[1].eq(self.mosi_cnt[0]),
                self.mosi_cnt[0].eq(mosi_db.o),
            )
        ]

        # RX data behavior description
        self.sync += [
            If(self.csn,
                self.bitcnt.eq(0),
                self.rxd.eq(0),
            ).Elif(self.sck_r,
                self.bitcnt.eq(self.bitcnt+1),
                self.rxd.eq(self.rxd << 1),
                self.rxd[0].eq(self.mosi_s),
            )
        ]
            
        # RX completed notification
        self.sync += [       
            self.rxc.eq(~self.csn & self.sck_r & (self.bitcnt == 7)),
        ]
        
        # TX data behavior description
        self.sync += [
            If(self.csn,
                self.tx_buf.eq(0),  
            ).Elif(self.bitcnt == 0,
                self.tx_buf.eq(self.txd),
            ).Elif(self.sck_f,
                self.tx_buf.eq(self.tx_buf<<1),
            )            
        ]        
        
        # MISO output behavior description
        self.comb += [
            self.miso.eq(self.tx_buf[7]),
        ]
        
        # TX data request notification
        self.comb += [
            self.txr.eq(~self.csn & (self.bitcnt == 0)),    
        ]        
              
def SpiSlaveGenerator(dut):
    cnt1 = 0
    cnt2 = 0
    for cycle in range(1000):

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

        if cycle > 0 and cycle < 2:
            yield dut.csn.eq(1)
            
        if cycle > 2 and cycle < 4:
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
