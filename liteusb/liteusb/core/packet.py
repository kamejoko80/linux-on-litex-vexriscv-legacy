from migen.genlib.misc import WaitTimer

from litex.soc.interconnect import stream

from liteusb.common import *


class LiteUSBPacketizer(Module):
    def __init__(self):
        self.sink = sink = stream.Endpoint(user_description(8))
        self.source = source = stream.Endpoint(phy_description(8))

        # # #

        # Packet description
        #   - preamble : 4 bytes
        #   - dst      : 1 byte
        #   - length   : 4 bytes
        #   - payload
        header = [
            # preamble
            0x5A,
            0xA5,
            0x5A,
            0xA5,
            # dst
            sink.dst,
            # length
            sink.length[24:32],
            sink.length[16:24],
            sink.length[8:16],
            sink.length[0:8],
        ]

        header_unpack = stream.Unpack(len(header), phy_description(8))
        self.submodules += header_unpack

        for i, byte in enumerate(header):
            chunk = getattr(header_unpack.sink.payload, "chunk" + str(i))
            self.comb += chunk.data.eq(byte)

        fsm = FSM(reset_state="IDLE")
        self.submodules += fsm

        fsm.act("IDLE",
            If(sink.valid,
                NextState("INSERT_HEADER")
            )
        )

        fsm.act("INSERT_HEADER",
            header_unpack.sink.valid.eq(1),
            source.valid.eq(1),
            source.data.eq(header_unpack.source.data),
            header_unpack.source.ready.eq(source.ready),
            If(header_unpack.sink.ready,
                NextState("COPY")
            )
        )

        fsm.act("COPY",
            source.valid.eq(sink.valid),
            source.data.eq(sink.data),
            sink.ready.eq(source.ready),
            If(source.ready & sink.last,
                NextState("IDLE")
            )
        )


class LiteUSBDepacketizer(Module):
    def __init__(self, clk_freq, timeout=10):
        self.sink = sink = stream.Endpoint(phy_description(8))
        self.source = source = stream.Endpoint(user_description(8))

        # # #

        # Packet description
        #   - preamble : 4 bytes
        #   - dst      : 1 byte
        #   - length   : 4 bytes
        #   - payload
        preamble = Array(Signal(8) for i in range(4))

        header = [
            # dst
            source.dst,
            # length
            source.length[24:32],
            source.length[16:24],
            source.length[8:16],
            source.length[0:8],
        ]

        header_pack = ResetInserter()(stream.Pack(phy_description(8), len(header)))
        self.submodules += header_pack

        for i, byte in enumerate(header):
            chunk = getattr(header_pack.source.payload, "chunk" + str(i))
            self.comb += byte.eq(chunk.data)

        fsm = FSM(reset_state="IDLE")
        self.submodules += fsm

        self.comb += preamble[0].eq(sink.data)
        for i in range(1, 4):
            self.sync += If(sink.valid & sink.ready,
                    preamble[i].eq(preamble[i-1])
            )
        fsm.act("IDLE",
            sink.ready.eq(1),
            If((preamble[3] == 0x5A) &
               (preamble[2] == 0xA5) &
               (preamble[1] == 0x5A) &
               (preamble[0] == 0xA5) &
               sink.valid,
                   NextState("RECEIVE_HEADER")
            ),
            header_pack.source.ready.eq(1),
        )

        self.submodules.timer = WaitTimer(clk_freq*timeout)
        self.comb += self.timer.wait.eq(~fsm.ongoing("IDLE"))

        fsm.act("RECEIVE_HEADER",
            header_pack.sink.valid.eq(sink.valid),
            header_pack.sink.payload.eq(sink.payload),
            If(self.timer.done,
                NextState("IDLE")
            ).Elif(header_pack.source.valid,
                NextState("COPY")
            ).Else(
                sink.ready.eq(1)
            )
        )

        self.comb += header_pack.reset.eq(self.timer.done)

        last = Signal()
        cnt = Signal(32)

        fsm.act("COPY",
            source.valid.eq(sink.valid),
            source.last.eq(last),
            source.data.eq(sink.data),
            sink.ready.eq(source.ready),
            If((source.valid & source.ready & last) | self.timer.done,
                NextState("IDLE")
            )
        )

        self.sync += \
            If(fsm.ongoing("IDLE"),
                cnt.eq(0)
            ).Elif(source.valid & source.ready,
                cnt.eq(cnt + 1)
            )
        self.comb += last.eq(cnt == source.length - 1)
