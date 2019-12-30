import os
from migen import *
from migen.fhdl import verilog
from random import randrange

class Serial(Module):
    def __init__(self, sck_i, sdi_i, sdo_o, csn_i):
        
        self.specials += [
            Instance("shifter",
                        i_sck_i = sck_i,
                        i_sdi_i = sdi_i,
                        o_sdo_o = sdo_o,
                        i_csn_i = csn_i,
                    )
        ]

#Simulation and verilog conversion
sck_i  = Signal()
sdi_i  = Signal()
sdo_o  = Signal()
csn_i  = Signal()
        
def generator(dut):
    for sck_i in range(500):
        yield sdi_i.eq(randrange(2))
        yield        
        
if __name__ == "__main__":
    sr = Serial(sck_i, sdi_i, sdo_o, csn_i)
    #print(verilog.convert(Serial(sck_i, sdi_i, sdo_o, csn_i), ios = {sck_i, sdi_i, sdo_o, csn_i}))
    run_simulation(sr, generator(sr), clocks={"sys": 10}, vcd_name="Serial.vcd") 