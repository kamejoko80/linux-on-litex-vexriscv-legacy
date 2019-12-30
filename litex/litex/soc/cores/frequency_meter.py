from migen import *
from migen.genlib.cdc import MultiReg, GrayCounter
from migen.genlib.cdc import GrayDecoder

from litex.soc.interconnect.csr import *


class Sampler(Module):
    def __init__(self, width):
        self.latch = Signal()
        self.i = Signal(width)
        self.o = Signal(32)

        # # #

        inc = Signal(width)
        counter = Signal(32)

        # use wrapping property of unsigned arithmeric to reset the counter
        # each cycle (reseting fmeter clock domain is unreliable)
        i_d = Signal(width)
        self.sync += i_d.eq(self.i)
        self.comb += inc.eq(self.i - i_d)

        self.sync += \
            If(self.latch,
                counter.eq(0),
                self.o.eq(counter),
            ).Else(
                counter.eq(counter + inc)
            )


class FrequencyMeter(Module, AutoCSR):
    def __init__(self, period, width=6):
        self.clk = Signal()
        self.value = CSRStatus(32)

        # # #

        self.clock_domains.cd_fmeter = ClockDomain(reset_less=True)
        self.comb += self.cd_fmeter.clk.eq(self.clk)

        # period generation
        period_done = Signal()
        period_counter = Signal(32)
        self.comb += period_done.eq(period_counter == period)
        self.sync += \
            If(period_done,
                period_counter.eq(0),
            ).Else(
                period_counter.eq(period_counter + 1)
            )

        # frequency measurement
        event_counter = ClockDomainsRenamer("fmeter")(GrayCounter(width))
        gray_decoder = GrayDecoder(width)
        sampler = Sampler(width)
        self.submodules += event_counter, gray_decoder, sampler

        self.specials += MultiReg(event_counter.q, gray_decoder.i)
        self.comb += [
            event_counter.ce.eq(1),
            sampler.latch.eq(period_done),
            sampler.i.eq(gray_decoder.o),
            self.value.status.eq(sampler.o)
        ]
