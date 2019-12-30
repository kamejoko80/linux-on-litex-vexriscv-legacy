from migen import *

from migen.flow.actor import *
from migen.flow.network import *
from migen.actorlib import structuring, spi
from migen.bank.description import *
from migen.bank.eventmanager import *

from litex.soc.interconnect import stream
from litex.soc.interconnect import dma_lasmi

from liteusb.common import *


class LiteUSBDMAWriter(Module, AutoCSR):
    def __init__(self, lasmim):
        self.sink = sink = stream.Endpoint(user_description(8))

        # Pack data
        pack_factor = lasmim.dw//8
        pack = structuring.Pack(phy_description(8), pack_factor, reverse=True)
        cast = structuring.Cast(pack.source.payload.layout, lasmim.dw)

        # DMA
        writer = dma_lasmi.Writer(lasmim)
        self._reset = CSR()
        self.dma = InsertReset(spi.DMAWriteController(writer, mode=spi.MODE_SINGLE_SHOT))
        self.comb += self.dma.reset.eq(self._reset.r & self._reset.re)

        # Remove last/length/dst fields from payload
        self.comb += [
            pack.sink.valid.eq(sink.valid),
            pack.sink.payload.eq(sink.payload),
            sink.ready.eq(pack.sink.ready)
        ]

        # Graph
        g = DataFlowGraph()
        g.add_pipeline(pack, cast, self.dma)
        self.submodules += CompositeActor(g)

        # IRQ
        self.submodules.ev = EventManager()
        self.ev.done = EventSourcePulse()
        self.ev.finalize()
        self.comb += self.ev.done.trigger.eq(sink.valid & sink.last)

        # CRC
        self._crc_failed = CSRStatus()
        self.sync += \
            If(sink.valid & sink.last,
                self._crc_failed.status.eq(sink.error)
            )


class LiteUSBDMAReader(Module, AutoCSR):
    def __init__(self, lasmim, tag):
        self.source = source = stream.Endpoint(user_description(8))

        reader = dma_lasmi.Reader(lasmim)
        self.dma = spi.DMAReadController(reader, mode=spi.MODE_SINGLE_SHOT)

        pack_factor = lasmim.dw//8
        packed_dat = structuring.pack_layout(8, pack_factor)
        cast = structuring.Cast(lasmim.dw, packed_dat)
        unpack = structuring.Unpack(pack_factor, phy_description(8), reverse=True)

        # Graph
        cnt = Signal(32)
        self.sync += \
            If(self.dma.generator._shoot.re,
                cnt.eq(0)
            ).Elif(source.valid & source.ready,
                cnt.eq(cnt + 1)
            )
        g = DataFlowGraph()
        g.add_pipeline(self.dma, cast, unpack)
        self.submodules += CompositeActor(g)
        self.comb += [
            source.valid.eq(unpack.source.valid),
            source.last.eq(cnt == (self.dma.length*pack_factor-1)),
            source.length.eq(self.dma.length*pack_factor),
            source.data.eq(unpack.source.data),
            source.dst.eq(tag),
            unpack.source.ready.eq(source.ready)
        ]

        # IRQ
        self.submodules.ev = EventManager()
        self.ev.done = EventSourcePulse()
        self.ev.finalize()
        self.comb += self.ev.done.trigger.eq(source.valid & source.last)


class LiteUSBDMA(Module, AutoCSR):
    def __init__(self, port, lasmim_dma_wr, lasmim_dma_rd):
        self.submodules.writer = LiteUSBDMAWriter(lasmim_dma_wr)
        self.submodules.reader = LiteUSBDMAReader(lasmim_dma_rd, port.tag)
        self.submodules.ev = SharedIRQ(self.writer.ev, self.reader.ev)
        self.comb += [
            port.source.connect(self.writer.sink),
            self.reader.source.connect(port.sink)
        ]
