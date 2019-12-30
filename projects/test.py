from migen import *
from migen.fhdl import verilog
from random import randrange

class Baz(Module):
    def __init__(self):
        foo = Signal()
        bar_r = Signal()
        bar_w = Signal()
        
        self.clock_domains.cd_read = ClockDomain("read", reset_less=True)
        self.clock_domains.cd_write = ClockDomain("write", reset_less=True)
        #self.comb += self.cd_read.clk.eq(pads.clkout)
        
        self.sync.read += bar_r.eq(foo)   # when adding just one item to the list, you can use +=
        self.sync.write += bar_w.eq(foo)

def generator(dut):
    for read_clk in range(500):
        yield                
        
if __name__ == "__main__":
    b = Baz()
    # print(verilog.convert(Baz()))
    run_simulation(b, generator(b), clocks={"sys": 10}, vcd_name="Baz.vcd")
