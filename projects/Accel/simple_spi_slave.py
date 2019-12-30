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

class SpiSlave(Module):
    def __init__(self):
        # Physical pins interface
        self.sck  = Signal()  # SCK pin input
        self.mosi = Signal()  # MOSI pin input
        self.miso = Signal()  # MISO pin output
        self.csn  = Signal(1, reset=1)  # CSN pin input   
        
        # Internal core signals
        self.rxc  = Signal()  # Data RX complete
        self.txr  = Signal()  # Data TX request
        self.rxd  = Signal(8) # RX data
        self.txd  = Signal(8) # TX data
      
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
        
def EdgeDetectorTestBench(dut):
    cnt = 0
    for cycle in range(1000):
        if cnt < 20:
            cnt = cnt + 1
        else:
            cnt = 0
            yield dut.i.eq(randrange(2))

        if cycle > 2 and cycle < 4:
            yield dut.csn.eq(0)

        yield        
        
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

        if cycle > 2 and cycle < 4:
            yield dut.csn.eq(0)
            yield dut.txd.eq(0xA5)  

        yield        
        
        
if __name__ == "__main__":
    #dut = EdgeDetector()
    #print(verilog.convert(EdgeDetector()))
    #run_simulation(dut, EdgeDetectorTestBench(dut), clocks={"sys": 10}, vcd_name="EdgeDetector.vcd")
    #os.system("gtkwave EdgeDetector.vcd")
    
    dut = SpiSlave()
    #print(verilog.convert(SpiSlave()))
    run_simulation(dut, SpiSlaveGenerator(dut), clocks={"sys": 10}, vcd_name="SpiSlave.vcd")
    os.system("gtkwave SpiSlave.vcd")