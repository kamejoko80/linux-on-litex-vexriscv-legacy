#
# Created 2019-2020 Henry Dang <henrydang@fossil.com>
#

import os

from migen import *
from migen.fhdl import verilog
from migen.genlib.misc import WaitTimer
from migen.genlib.fifo import SyncFIFOBuffered

from litex.soc.interconnect.csr import *
from litex.soc.interconnect.csr_eventmanager import *

from random import randrange

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

class SPIMasterController(Module, AutoCSR):
    def __init__(self, freq, baudrate):
        # Physical pins interface
        self.sck  = Signal()         # SCK pin input
        self.mosi = Signal()         # MOSI pin input
        self.miso = Signal()         # MISO pin output
        self.ssn  = Signal(reset=1)  # Slave select pin output
        self.int  = Signal()         # Interrupt pin output

        # CSR interface
        self.config  = CSRStorage(4, reset=0x0C) # [CPOL, CPHA, IPOL, IE]
        self.tx_data = CSRStorage(8)
        self.rx_data = CSRStatus(8)
        self.done    = CSRStatus(reset=1)
        self.start   = CSR()
        self.csn     = CSRStorage()

        # Internal signals
        self.prescaler = Signal(max=int(freq/baudrate))
        self.frame     = Signal(reset=1)
        self.spi_clk   = Signal()
        self.tx_buf    = Signal(8)
        self.rx_buf    = Signal(8)
        self.edge_cnt  = Signal(5)
        self.irq       = Signal()

        # SPI clock generation
        self.sync += [
            If(~self.frame,
                If(self.prescaler == int(freq/baudrate),
                    self.prescaler.eq(0),
                    self.edge_cnt.eq(self.edge_cnt + 1),
                    If(self.edge_cnt < 16,
                        self.spi_clk.eq(~self.spi_clk)
                    ).Else(
                        self.frame.eq(1),
                        self.rx_data.status.eq(self.rx_buf),
                        self.done.status.eq(1),
                        self.irq.eq(1)
                    )
                ).Else(
                    self.prescaler.eq(self.prescaler + 1)
                )
            )
        ]

        ####### pin signal assignment #######

        self.comb += [
            # pin sck
            If(self.config.storage[0] == 0, # CPOL = 0
                self.sck.eq(self.spi_clk)
            ).Else(
                self.sck.eq(~self.spi_clk)
            ),
            # pin mosi
            self.mosi.eq(self.tx_buf[7]),
            # pin ssn
            self.ssn.eq(self.csn.storage),
            # pin int
            If(self.config.storage[3],     # IE   = 1 (Interrupt enable)
                If(self.config.storage[2], # IPOL = 1 (Active high)
                    self.int.eq(self.irq)
                ).Else(
                    self.int.eq(~self.irq)
                )
            ).Else(
                self.int.eq(0) # Should be configured as Hi-Z
            )
        ]

        # SPI start condition
        self.sync += [
            If(self.start.re & self.start.r & self.done.status,
                self.tx_buf.eq(self.tx_data.storage),
                self.rx_buf.eq(0),
                self.prescaler.eq(0),
                self.frame.eq(0),
                self.done.status.eq(0),
                self.edge_cnt.eq(0),
                self.irq.eq(0),
            )
        ]

        # Generate rising/falling edge output
        edt = ResetInserter()(EdgeDetector())
        self.submodules += edt

        self.comb += [
            edt.reset.eq(self.frame),
            edt.i.eq(self.spi_clk),
        ]

        # Submodule FSM handles data in/out activities
        fsm = FSM(reset_state = "IDLE")
        self.submodules += fsm

        # FSM behavior description
        fsm.act("IDLE",
            If(~self.frame,
                If(self.config.storage[1],  # CPHA = 1
                    If(edt.r,
                        NextState("SHIFT"),
                    )
                ).Else(
                    NextState("SHIFT"),
                )
            ).Else(
                NextState("IDLE"),
            )
        )
        fsm.act("SHIFT",
            If(self.frame,
                NextState("IDLE")
            ).Else(
                If(self.config.storage[1], # CPHA = 1
                    If(edt.r,
                        NextValue(self.tx_buf, self.tx_buf << 1),
                        NextValue(self.rx_buf[0], self.mosi),
                        NextState("SHIFT")
                    ),
                    If(edt.f,
                        NextValue(self.rx_buf, self.rx_buf << 1)
                    )
                ).Else( # CPHA = 0
                    If(edt.f,
                        NextValue(self.tx_buf, self.tx_buf << 1),
                        NextValue(self.rx_buf[0], self.mosi),
                        NextState("SHIFT")
                    ),
                    If(edt.r,
                        NextValue(self.rx_buf, self.rx_buf << 1)
                    )
                )
            )
        )

def SPIMasterControllerTestBench(dut):

    for cycle in range(5000):

        if cycle == 2:
            yield dut.tx_data.storage.eq(0xA5)
            yield dut.start.re.eq(1)
            yield dut.start.r.eq(1)

        if cycle == 3:
            yield dut.start.re.eq(0)
            yield dut.start.r.eq(0)

        if cycle == 1000:
            yield dut.tx_data.storage.eq(0x24)
            yield dut.start.re.eq(1)
            yield dut.start.r.eq(1)

        if cycle == 1001:
            yield dut.start.re.eq(0)
            yield dut.start.r.eq(0)

        yield

if __name__ == "__main__":

    dut = SPIMasterController(freq=50000000, baudrate=1000000)
    #print(verilog.convert(SPIMasterController(freq=50000000, baudrate=1000000)))
    run_simulation(dut, SPIMasterControllerTestBench(dut), clocks={"sys": 10}, vcd_name="SPIMasterController.vcd")

 