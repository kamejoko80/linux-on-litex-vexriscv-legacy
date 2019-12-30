from migen import *
from migen.fhdl import verilog
from random import randrange
 
 
class Dflipflop(Module):
  def __init__(self, D, Q, Qi):
    self.sync += Q.eq(D)
    self.comb += Qi.eq(~Q)
    
#Simulation and verilog conversion
D  = Signal()
Q  = Signal()
Qi = Signal()

def generator(dut):
    for sys_clk in range(500):
        yield D.eq(randrange(2))
        yield
 
if __name__ == "__main__":
    ff = Dflipflop(D, Q, Qi)
    # Generate Verilog
    # print(verilog.convert(Dflipflop(D, Q, Qi), ios={D,Q,Qi}))
    # Run simulation (10ns per clock cycle)
    run_simulation(ff, generator(ff), clocks={"sys": 10}, vcd_name="Dflipflop.vcd") 



