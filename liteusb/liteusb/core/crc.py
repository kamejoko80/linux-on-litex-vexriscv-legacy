from collections import OrderedDict
from functools import reduce
from operator import xor

from migen import *
from migen.genlib.misc import chooser

from litex.soc.interconnect import stream

from liteusb.common import *


class CRCEngine(Module):
    """Cyclic Redundancy Check Engine

    Compute next CRC value from last CRC value and data input using
    an optimized asynchronous LFSR.

    Parameters
    ----------
    dat_width : int
        Width of the data bus.
    width : int
        Width of the CRC.
    polynom : int
        Polynom of the CRC (ex: 0x04C11DB7 for IEEE 802.3 CRC)

    Attributes
    ----------
    d : in
        Data input.
    last : in
        last CRC value.
    next :
        next CRC value.
    """
    def __init__(self, dat_width, width, polynom):
        self.data = Signal(dat_width)
        self.last = Signal(width)
        self.next = Signal(width)

        # # #

        def _optimize_eq(l):
            """
            Replace even numbers of XORs in the equation
            with an equivalent XOR
            """
            d = OrderedDict()
            for e in l:
                if e in d:
                    d[e] += 1
                else:
                    d[e] = 1
            r = []
            for key, value in d.items():
                if value%2 != 0:
                    r.append(key)
            return r

        # compute and optimize CRC's LFSR
        curval = [[("state", i)] for i in range(width)]
        for i in range(dat_width):
            feedback = curval.pop() + [("din", i)]
            for j in range(width-1):
                if (polynom & (1<<(j+1))):
                    curval[j] += feedback
                curval[j] = _optimize_eq(curval[j])
            curval.insert(0, feedback)

        # implement logic
        for i in range(width):
            xors = []
            for t, n in curval[i]:
                if t == "state":
                    xors += [self.last[n]]
                elif t == "din":
                    xors += [self.data[n]]
            self.comb += self.next[i].eq(reduce(xor, xors))


@ResetInserter()
@CEInserter()
class CRC32(Module):
    """IEEE 802.3 CRC

    Implement an IEEE 802.3 CRC generator/checker.

    Parameters
    ----------
    dat_width : int
        Width of the data bus.

    Attributes
    ----------
    d : in
        Data input.
    value : out
        CRC value (used for generator).
    error : out
        CRC error (used for checker).
    """
    width = 32
    polynom = 0x04C11DB7
    init = 2**width-1
    check = 0xC704DD7B

    def __init__(self, dat_width):
        self.data = Signal(dat_width)
        self.value = Signal(self.width)
        self.error = Signal()

        # # #

        self.submodules.engine = CRCEngine(dat_width, self.width, self.polynom)
        reg = Signal(self.width, reset=self.init)
        self.sync += reg.eq(self.engine.next)
        self.comb += [
            self.engine.data.eq(self.data),
            self.engine.last.eq(reg),

            self.value.eq(~reg[::-1]),
            self.error.eq(self.engine.next != self.check)
        ]


class CRCInserter(Module):
    """CRC Inserter

    Append a CRC at the end of each packet.

    Parameters
    ----------
    layout : layout
        Layout of the dataflow.

    Attributes
    ----------
    sink : in
        Packets input without CRC.
    source : out
        Packets output with CRC.
    """
    def __init__(self, crc_class, layout):
        self.sink = sink = stream.Endpoint(layout)
        self.source = source = stream.Endpoint(layout)
        self.busy = Signal()

        # # #

        dw = len(sink.data)
        crc = crc_class(dw)
        fsm = FSM(reset_state="IDLE")
        self.submodules += crc, fsm

        fsm.act("IDLE",
            crc.reset.eq(1),
            sink.ready.eq(1),
            If(sink.valid,
                sink.ready.eq(0),
                NextState("COPY"),
            )
        )
        fsm.act("COPY",
            crc.ce.eq(sink.valid & source.ready),
            crc.data.eq(sink.data),
            sink.connect(source),
            source.last.eq(0),
            If(sink.valid & sink.last & source.ready,
                NextState("INSERT"),
            )
        )
        ratio = crc.width//dw
        if ratio > 1:
            cnt = Signal(max=ratio, reset=ratio-1)
            cnt_done = Signal()
            fsm.act("INSERT",
                source.valid.eq(1),
                chooser(crc.value, cnt, source.data, reverse=True),
                If(cnt_done,
                    source.last.eq(1),
                    If(source.ready, NextState("IDLE"))
                )
            )
            self.comb += cnt_done.eq(cnt == 0)
            self.sync += \
                If(fsm.ongoing("IDLE"),
                    cnt.eq(cnt.reset)
                ).Elif(fsm.ongoing("INSERT") & ~cnt_done,
                    cnt.eq(cnt - source.ready)
                )
        else:
            fsm.act("INSERT",
                source.valid.eq(1),
                source.last.eq(1),
                source.data.eq(crc.value),
                If(source.ready, NextState("IDLE"))
            )
        self.comb += self.busy.eq(~fsm.ongoing("IDLE"))


class LiteUSBCRC32Inserter(CRCInserter):
    def __init__(self):
        CRCInserter.__init__(self, CRC32, user_description(8))


class CRCChecker(Module):
    """CRC Checker

    Check CRC at the end of each packet.

    Parameters
    ----------
    layout : layout
        Layout of the dataflow.

    Attributes
    ----------
    sink : in
        Packets input with CRC.
    source : out
        Packets output without CRC and "error" set to 0
        on last when CRC OK / set to 1 when CRC KO.
    """
    def __init__(self, crc_class, layout):
        self.sink = sink = stream.Endpoint(layout)
        self.source = source = stream.Endpoint(layout)
        self.busy = Signal()

        # # #

        dw = len(sink.data)
        crc = crc_class(dw)
        self.submodules += crc
        ratio = crc.width//dw

        error = Signal()
        fifo = ResetInserter()(stream.SyncFIFO(layout, ratio + 1))
        self.submodules += fifo

        fsm = FSM(reset_state="RESET")
        self.submodules += fsm

        fifo_in = Signal()
        fifo_out = Signal()
        fifo_full = Signal()

        self.comb += [
            fifo_full.eq(fifo.level == ratio),
            fifo_in.eq(sink.valid & (~fifo_full | fifo_out)),
            fifo_out.eq(source.valid & source.ready),

            sink.connect(fifo.sink),
            fifo.sink.valid.eq(fifo_in),
            self.sink.ready.eq(fifo_in),

            source.valid.eq(sink.valid & fifo_full),
            source.last.eq(sink.last),
            fifo.source.ready.eq(fifo_out),
            source.payload.eq(fifo.source.payload),

            source.error.eq(sink.error | crc.error),
        ]

        fsm.act("RESET",
            crc.reset.eq(1),
            fifo.reset.eq(1),
            NextState("IDLE"),
        )
        fsm.act("IDLE",
            crc.data.eq(sink.data),
            If(sink.valid & sink.ready,
                crc.ce.eq(1),
                NextState("COPY")
            )
        )
        fsm.act("COPY",
            crc.data.eq(sink.data),
            If(sink.valid & sink.ready,
                crc.ce.eq(1),
                If(sink.last,
                    NextState("RESET")
                )
            )
        )
        self.comb += self.busy.eq(~fsm.ongoing("IDLE"))


class LiteUSBCRC32Checker(CRCChecker):
    def __init__(self):
        CRCChecker.__init__(self, CRC32, user_description(8))
