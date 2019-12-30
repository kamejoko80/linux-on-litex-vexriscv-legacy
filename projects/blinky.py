from migen import *
import ice40hx8k
 
class MyLedBlink(Module):
    def __init__(self, platform):
        self.led = led = platform.request("user_led")
        counter = Signal(25)
 
        self.sync += counter.eq(counter + 1)
        self.comb += led.eq(counter[24])
 
platform = ice40hx8k.Platform()
dut = MyLedBlink(platform)
platform.build(dut)
