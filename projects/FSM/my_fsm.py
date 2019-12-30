from migen import *
from migen.fhdl import verilog

class Example(Module):
    def __init__(self):
        self.s  = Signal(8)
        self.a  = Signal(8)        
        self.b  = Signal(8) 
        self.op = Signal(1, reset=0) # 0 => a + b, 1 a - b
        self.en = Signal(1, reset=0) # 0 => a + b, 1 a - b        
        
        myfsm = FSM(reset_state = "LOAD")
        self.submodules += myfsm

        myfsm.act("LOAD",
            If(self.en == 1,
                NextValue(self.a, 0x05),
                NextValue(self.b, 0x02),
                If(self.op == 0, 
                    NextState("ADD")
                ).Else(
                    NextState("SUB")
                )
            )
        )
        myfsm.act("ADD",
            self.s.eq(self.a + self.b),
            NextState("IDLE")
        )
        myfsm.act("SUB",
            self.s.eq(self.a - self.b),
            NextState("IDLE")
        )
        myfsm.act("IDLE",
            NextState("IDLE")
        )        

if __name__ == "__main__":
    example = Example()
    print(verilog.convert(example, {example.s, example.a, example.b, example.op, example.en}))
