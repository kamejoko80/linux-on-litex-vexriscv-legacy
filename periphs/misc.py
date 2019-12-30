import os
import sys

from migen import *
from migen.genlib.fifo import SyncFIFOBuffered
from migen.genlib.misc import WaitTimer
from migen.fhdl.specials import Tristate

from litex.soc.interconnect import wishbone
from litex.soc.integration.soc_core import mem_decoder

from litex.soc.interconnect.csr import *
from litex.soc.interconnect.csr_eventmanager import *

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


# GPIO interrupt
class GpioLED(Module, AutoCSR):
    def __init__(self, pad):
        self.led = CSRStorage(8)
        self.comb += pad.eq(self.led.storage)

# GPIO interrupt
class GpioISR(Module, AutoCSR):
    def __init__(self, pad, rissing_edge_detect = False):
        # Add int to module
        self.submodules.ev = EventManager()

        if rissing_edge_detect:
            self.ev.gpio_rising_int = EventSourcePulse()
            self.ev.finalize()
            self.comb += self.ev.gpio_rising_int.trigger.eq(pad)
        else:
            self.ev.gpio_falling_int = EventSourceProcess()
            self.ev.finalize()
            self.comb += self.ev.gpio_falling_int.trigger.eq(pad)

# Simple Adder8 module
class Adder8(Module, AutoCSR):
    def __init__(self):
        self.op1 = CSRStorage(8)
        self.op2 = CSRStorage(8)
        self.sum = CSRStatus(8)
        self.ena = CSRStorage(1, reset = 0)

        self.sync += [
            If(self.ena.storage == 1,
                self.sum.status.eq(self.op1.storage + self.op2.storage),
            )
        ]

# Simple Uart module
class MyUart(Module, AutoCSR):
    def __init__(self, txd, led):
        self.tx_dat = CSRStorage(8)
        self.tx_ena = CSRStorage(1, reset = 0)
        self.tx_bsy = CSRStatus(1)

        tx_status = Signal()

        self.comb += self.tx_bsy.status.eq(tx_status)

        self.specials += [
            Instance("my_uart",
                    i_din=self.tx_dat.storage,
                    i_wr_en=self.tx_ena.storage,
                    i_clk_in=ClockSignal(),
                    o_tx=txd,
                    o_tx_busy=tx_status,
                    )
        ]

    def add_source(self, platform):
            platform.add_source(os.path.join("periphs/verilog/uart", "my_uart.v"))

# Custom UART module
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
        self.readable = Signal()  # Assert inditates dout can be read
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

# Simple wishbone gpio module
class WbGpio(Module):
    def __init__(self, led):
        self.bus = bus = wishbone.Interface()
        led_wire = Signal(1, reset=1)

        self.comb += led.eq(led_wire)

        # run mw addr 0/1 1 to turn on/off the led
        self.sync += [
            bus.ack.eq(0),
            If(bus.cyc & bus.stb & ~bus.ack,
                bus.ack.eq(1),
                If(bus.we,
                    led_wire.eq(bus.dat_w[0])
                )
            )
        ]

# Wishbone to avalon bridge
class W2ABridge(Module):
    def __init__(self):
        self.bus = bus = wishbone.Interface()

        self.specials += [
            Instance("wb_to_avalon_bridge",
                    # WB IF
                    i_wb_clk_i = ClockSignal(),
                    i_wb_rst_i = ResetSignal(),
                    i_wb_adr_i = bus.adr,
                    i_wb_dat_i = bus.dat_w,
                    i_wb_sel_i = bus.sel,
                    i_wb_we_i  = bus.we,
                    i_wb_cyc_i = bus.cyc,
                    i_wb_stb_i = bus.stb,
                    i_wb_cti_i = bus.cti,
                    i_wb_bte_i = bus.bte,
                    o_wb_dat_o = bus.dat_r,
                    o_wb_ack_o = bus.ack,
                    )
        ]

    def add_source(self, platform):
            platform.add_source(os.path.join("periphs/verilog/w2a", "wb_to_avalon_bridge.v"))

# SJA1000 opencore can controller module
class SJA1000(Module, AutoCSR):
    def __init__(self, canif):
        # falling edge interrupt
        self.submodules.ev = EventManager()
        self.ev.can_irq = EventSourceProcess()
        self.ev.finalize()

        # can interrupt signal
        can_irq_signal = Signal()

        # wishbone bus
        self.bus = bus = wishbone.Interface()

        self.comb += [
            self.ev.can_irq.trigger.eq(can_irq_signal),
            canif.irq.eq(can_irq_signal) # drives the LED
        ]

        self.specials += [
            Instance("can_top",
                    # WB IF
                    i_wb_clk_i   = ClockSignal(),
                    i_wb_rst_i   = ResetSignal(),
                    i_wb_dat_i   = bus.dat_w,
                    o_wb_dat_o   = bus.dat_r,
                    i_wb_cyc_i   = bus.cyc,
                    i_wb_stb_i   = bus.stb,
                    i_wb_we_i    = bus.we,
                    i_wb_adr_i   = bus.adr,
                    o_wb_ack_o   = bus.ack,
                    # MISC
                    i_clk_i      = ClockSignal(),
                    i_rx_i       = canif.rx,
                    o_tx_o       = canif.tx,
                    o_bus_off_on = canif.boo,
                    o_irq_on     = can_irq_signal,
                    o_clkout_o   = canif.clkout,
                    )
        ]

    def add_source(self, platform):
            platform.add_source(os.path.join("periphs/verilog/can", "timescale.v"))
            platform.add_source(os.path.join("periphs/verilog/can", "can_defines.v"))
            platform.add_source(os.path.join("periphs/verilog/can", "can_top.v"))
            platform.add_source(os.path.join("periphs/verilog/can", "can_acf.v"))
            platform.add_source(os.path.join("periphs/verilog/can", "can_btl.v"))
            platform.add_source(os.path.join("periphs/verilog/can", "can_ibo.v"))
            platform.add_source(os.path.join("periphs/verilog/can", "can_register_asyn.v"))
            platform.add_source(os.path.join("periphs/verilog/can", "can_register_syn.v"))
            platform.add_source(os.path.join("periphs/verilog/can", "can_bsp.v"))
            platform.add_source(os.path.join("periphs/verilog/can", "can_crc.v"))
            platform.add_source(os.path.join("periphs/verilog/can", "can_fifo.v"))
            platform.add_source(os.path.join("periphs/verilog/can", "can_register_asyn_syn.v"))
            platform.add_source(os.path.join("periphs/verilog/can", "can_registers.v"))
            platform.add_source(os.path.join("periphs/verilog/can", "can_register.v"))

# Opencore SPI master
class SpiMaster(Module, AutoCSR):
    def __init__(self, pads):
        # rissing edge interrupt
        self.submodules.ev = EventManager()
        self.ev.spi_irq = EventSourcePulse()
        self.ev.finalize()

        # wishbone bus
        self.bus = bus = wishbone.Interface()

        # inverted clk output
        sclk_inv = Signal()

        self.comb += [
           # pads.sclk.eq(~sclk_inv) # Need to invert to test with ADC128S102
           pads.sclk.eq(sclk_inv)
        ]

        self.specials += [
            Instance("spi_top",
                    # WB IF
                    i_wb_clk_i   = ClockSignal(),
                    i_wb_rst_i   = ResetSignal(),
                    i_wb_adr_i   = bus.adr,
                    i_wb_dat_i   = bus.dat_w,
                    i_wb_sel_i   = bus.sel,
                    i_wb_we_i    = bus.we,
                    i_wb_cyc_i   = bus.cyc,
                    i_wb_stb_i   = bus.stb,
                    o_wb_dat_o   = bus.dat_r,
                    o_wb_ack_o   = bus.ack,
                    o_wb_err_o   = bus.err,

                    # SPI signals
                    o_wb_int_o   = self.ev.spi_irq.trigger, # SPI IRQ
                    o_ss_pad_o   = pads.csn,       # SPI chip select need
                    o_sclk_pad_o = sclk_inv,       # SPI clkout
                    o_mosi_pad_o = pads.mosi,      # SPI mosi
                    i_miso_pad_i = pads.miso,      # SPI miso
                    )
        ]

    def add_source(self, platform):
            platform.add_source(os.path.join("periphs/verilog/spi", "spi_defines.v"))
            platform.add_source(os.path.join("periphs/verilog/spi", "spi_clgen.v"))
            platform.add_source(os.path.join("periphs/verilog/spi", "spi_shift.v"))
            platform.add_source(os.path.join("periphs/verilog/spi", "spi_top.v"))
            platform.add_source(os.path.join("periphs/verilog/spi", "timescale.v"))

class MailBox(Module):
    def __init__(self, fifo_depth=8):
        self.dout_r = Signal(8)         # CSR(dout)           :r      (i)
        self.dout_re = Signal()         # CSR(dout)           :re     (i)
        self.int_r  = Signal()          # CSR(int)            :r      (i)
        self.int_re = Signal()          # CSR(int)            :re     (i)
        self.int    = Signal()          # Interrupt notification      (o)
        self.din_status = Signal(8)     # CSRStatus(din)      :status (o)
        self.rd_r = Signal()            # CSR(rd)             :r      (i)
        self.rd_re = Signal()           # CSR(rd)             :re     (i)
        self.len_status = Signal(8)     # CSRStatus(level)    :status (o)

        # Connect to the FIFO buffer
        fifo = SyncFIFOBuffered(width=8, depth=fifo_depth)
        self.submodules += fifo

        # Mailbox behavior implementation
        self.comb += [
            fifo.din.eq(self.dout_r),
            fifo.we.eq(self.dout_re),
            self.int.eq(self.int_r & self.int_re),
            self.din_status.eq(fifo.dout),
            fifo.re.eq(self.rd_r & self.rd_re),
            self.len_status.eq(fifo.level),
        ]

class MailBoxSenderInf(Module, AutoCSR):
    def __init__(self, pads):
        self.dout     = CSR(8)
        self.int      = CSR()

        self.comb += [
            pads.dout_r.eq(self.dout.r),
            pads.dout_re.eq(self.dout.re),
            pads.int_r.eq(self.int.r),
            pads.int_re.eq(self.int.re),
        ]

class MailBoxReceiverInf(Module, AutoCSR):
    def __init__(self, pads):
        self.din      = CSRStatus(8)
        self.len      = CSRStatus(8)
        self.rd       = CSR()

        # Rising edge interrupt
        self.submodules.ev = EventManager()
        self.ev.mbx_int = EventSourcePulse()
        self.ev.finalize()

        self.comb += [
            pads.rd_r.eq(self.rd.r),
            pads.rd_re.eq(self.rd.re),
            self.din.status.eq(pads.din_status),
            self.len.status.eq(pads.len_status),
            self.ev.mbx_int.trigger.eq(pads.int),
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

class SPIMasterController(Module, AutoCSR):
    def __init__(self, freq, baudrate, pads):
        # CSR interface
        self.config    = CSRStorage(4, reset=0x00) # [IE, IPOL, CPHA, CPOL]
        self.tx_data   = CSRStorage(8)
        self.done      = CSRStatus(reset=1)
        self.start     = CSR()

        # SPI internal signals
        self.csn       = CSRStorage(reset=1)
        self.sck       = Signal()
        self.mosi      = Signal()
        self.miso_0    = Signal()     # miso line 0
        self.miso_1    = Signal()     # miso line 1
        self.miso_2    = Signal()     # miso line 2
        self.miso_3    = Signal()     # miso line 3

        # Check if we need more miso line
        self.rx_data_0 = CSRStatus(8) # miso line 0
        self.rx_data_1 = CSRStatus(8) # miso line 1
        self.rx_data_2 = CSRStatus(8) # miso line 2
        self.rx_data_3 = CSRStatus(8) # miso line 3

        # Internal signals
        self.prescaler = Signal(max=int(freq/baudrate))
        self.frame     = Signal(reset=1)
        self.spi_clk   = Signal()
        self.tx_buf    = Signal(8)
        self.rx_buf    = Array(Signal(8) for a in range(4))
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
                        self.rx_data_0.status.eq(self.rx_buf[0]), # miso line 0
                        self.rx_data_1.status.eq(self.rx_buf[1]), # miso line 1
                        self.rx_data_2.status.eq(self.rx_buf[2]), # miso line 2
                        self.rx_data_3.status.eq(self.rx_buf[3]), # miso line 3
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
            # signal sck
            If(self.config.storage[0] == 0, # CPOL = 0
                self.sck.eq(self.spi_clk)
            ).Else(
                self.sck.eq(~self.spi_clk)
            ),
            # signal mosi
            self.mosi.eq(self.tx_buf[7]),
            # pin irq
            If(self.config.storage[3],     # IE   = 1 (Interrupt enable)
                If(self.config.storage[2], # IPOL = 1 (Active high)
                    pads.irq.eq(self.irq)
                ).Else(
                    pads.irq.eq(~self.irq)
                )
            ).Else(
                pads.irq.eq(0) # Should be configured as Hi-Z
            )
        ]

        # pin sck_x
        if hasattr(pads, "sck_0"):
            self.comb += pads.sck_0.eq(self.sck)     # sck line 0
        if hasattr(pads, "sck_1"):
            self.comb += pads.sck_1.eq(self.sck)     # sck line 1
        if hasattr(pads, "sck_2"):
            self.comb += pads.sck_2.eq(self.sck)     # sck line 2
        if hasattr(pads, "sck_3"):
            self.comb += pads.sck_3.eq(self.sck)     # sck line 3

        # pin miso_x
        if hasattr(pads, "miso_0"):
            self.comb += self.miso_0.eq(pads.miso_0) # miso line 0
        if hasattr(pads, "miso_1"):
            self.comb += self.miso_1.eq(pads.miso_1) # miso line 1
        if hasattr(pads, "miso_2"):
            self.comb += self.miso_2.eq(pads.miso_2) # miso line 2
        if hasattr(pads, "miso_3"):
            self.comb += self.miso_3.eq(pads.miso_3) # miso line 3

        # pin mosi_x
        if hasattr(pads, "mosi_0"):
            self.comb += pads.mosi_0.eq(self.mosi)   # miso line 0
        if hasattr(pads, "mosi_1"):
            self.comb += pads.mosi_1.eq(self.mosi)   # miso line 1
        if hasattr(pads, "mosi_2"):
            self.comb += pads.mosi_2.eq(self.mosi)   # miso line 2
        if hasattr(pads, "mosi_3"):
            self.comb += pads.mosi_3.eq(self.mosi)   # miso line 3

        # pin csn_x
        if hasattr(pads, "csn_0"):
            self.comb += pads.csn_0.eq(self.csn.storage) # csn line 0
        if hasattr(pads, "csn_1"):
            self.comb += pads.csn_1.eq(self.csn.storage) # csn line 1
        if hasattr(pads, "csn_2"):
            self.comb += pads.csn_2.eq(self.csn.storage) # csn line 2
        if hasattr(pads, "csn_3"):
            self.comb += pads.csn_3.eq(self.csn.storage) # csn line 3

        # SPI start condition
        self.sync += [
            If(self.start.re & self.start.r & self.done.status,
                self.tx_buf.eq(self.tx_data.storage),
                self.rx_buf[0].eq(0), # miso line 0
                self.rx_buf[1].eq(0), # miso line 1
                self.rx_buf[2].eq(0), # miso line 2
                self.rx_buf[3].eq(0), # miso line 3
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
                        NextValue(self.rx_buf[0][0], self.miso_0),      # miso line 0
                        NextValue(self.rx_buf[1][0], self.miso_1),      # miso line 1
                        NextValue(self.rx_buf[2][0], self.miso_2),      # miso line 2
                        NextValue(self.rx_buf[3][0], self.miso_3),      # miso line 3
                        NextState("SHIFT")
                    ),
                    If(edt.f,
                        NextValue(self.rx_buf[0], self.rx_buf[0] << 1), # miso line 0
                        NextValue(self.rx_buf[1], self.rx_buf[1] << 1), # miso line 1
                        NextValue(self.rx_buf[2], self.rx_buf[2] << 1), # miso line 2
                        NextValue(self.rx_buf[3], self.rx_buf[3] << 1), # miso line 3
                    )
                ).Else( # CPHA = 0
                    If(edt.f,
                        NextValue(self.tx_buf, self.tx_buf << 1),
                        NextValue(self.rx_buf[0][0], self.miso_0),      # miso line 0
                        NextValue(self.rx_buf[1][0], self.miso_1),      # miso line 1
                        NextValue(self.rx_buf[2][0], self.miso_2),      # miso line 2
                        NextValue(self.rx_buf[3][0], self.miso_3),      # miso line 3
                        NextState("SHIFT")
                    ),
                    If(edt.r,
                        NextValue(self.rx_buf[0], self.rx_buf[0] << 1), # miso line 0
                        NextValue(self.rx_buf[1], self.rx_buf[1] << 1), # miso line 1
                        NextValue(self.rx_buf[2], self.rx_buf[2] << 1), # miso line 2
                        NextValue(self.rx_buf[3], self.rx_buf[3] << 1), # miso line 3
                    )
                )
            )
        )

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

#if __name__ == "__main__":

    #dut = SyncFIFOTest(width=8, depth=2)
    #print(verilog.convert(SyncFIFOTest(width=8, depth=32)))
    #run_simulation(dut, SyncFIFOTestTestBench(dut), clocks={"sys": 10}, vcd_name="SyncFIFOTest.vcd")
    #os.system("gtkwave SyncFIFOTest.vcd")

    #dut = UART(freq=50000000, baud=115200)
    #print(verilog.convert(UART(freq=50000000, baud=115200)))
    #run_simulation(dut, UARTTestBench(dut), clocks={"sys": 10}, vcd_name="UART.vcd")

    #dut = SPIMasterController(freq=50000000, baudrate=1000000)
    #print(verilog.convert(SPIMasterController(freq=50000000, baudrate=1000000)))
    #run_simulation(dut, SPIMasterControllerTestBench(dut), clocks={"sys": 10}, vcd_name="SPIMasterController.vcd")
  