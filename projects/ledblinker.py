from migen import *
from migen.fhdl import verilog

class Blinker(Module):
    def __init__(self, led, maxperiod):
        counter = Signal(max=maxperiod+1)
        period = Signal(max=maxperiod+1)
        self.comb += period.eq(maxperiod)
        self.sync += If(counter == 0,
                        led.eq(~led),
                        counter.eq(period)
                        ).Else(
                        counter.eq(counter - 1)
                        )
def generator(dut):
    for sys_clk in range(500):
        yield

# Create signal and module object        
led = Signal()
my_blinker = Blinker(led, 100)

# Generate Verilog
print(verilog.convert(my_blinker, ios={led}))

# Run simulation (10ns per clock cycle)
# run_simulation(my_blinker, generator(my_blinker), clocks={"sys": 10}, vcd_name="ledblinker.vcd")
