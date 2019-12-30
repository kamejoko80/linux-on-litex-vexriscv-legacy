# This file is Copyright (c) 2013 Florent Kermarrec <florent@enjoy-digital.fr>
# License: BSD

from litex.build.generic_platform import *
from litex.build.altera import AlteraPlatform
from litex.build.altera.programmer import USBBlaster

# IOs ----------------------------------------------------------------------------------------------

_io = [
    ("clk100", 0, Pins("J10"), IOStandard("3.3-V LVTTL")),

    ("key", 0, Pins("AB12"), IOStandard("3.3-V LVTTL")),
    ("key", 1, Pins("M22"), IOStandard("3.3-V LVTTL")),

    ("serial", 0,
        Subsignal("tx", Pins("H13"), IOStandard("3.3-V LVTTL")),
        Subsignal("rx", Pins("J13"), IOStandard("3.3-V LVTTL"))
    ),

    ("spi", 0,
        Subsignal("sclk", Pins("A18"), IOStandard("3.3-V LVTTL")),
        Subsignal("miso", Pins("A19"), IOStandard("3.3-V LVTTL")),
        Subsignal("mosi", Pins("C15"), IOStandard("3.3-V LVTTL")),
        Subsignal("csn",  Pins("D15"), IOStandard("3.3-V LVTTL")),
        Subsignal("irq",  Pins("B16"), IOStandard("3.3-V LVTTL")),
    ),

    ("spi_slave", 0,
        # SPI slave part
        Subsignal("sck",  Pins("C17"), IOStandard("3.3-V LVTTL")),
        Subsignal("miso", Pins("C18"), IOStandard("3.3-V LVTTL")),
        Subsignal("mosi", Pins("G12"), IOStandard("3.3-V LVTTL")),
        Subsignal("csn",  Pins("H12"), IOStandard("3.3-V LVTTL")),
        Subsignal("irq",  Pins("G14"), IOStandard("3.3-V LVTTL")),
        Subsignal("int1", Pins("G15"), IOStandard("3.3-V LVTTL")),
        Subsignal("int2", Pins("F17"), IOStandard("3.3-V LVTTL")),
        Subsignal("led0", Pins("W4"),  IOStandard("3.3-V LVTTL")),
        Subsignal("led1", Pins("Y4"),  IOStandard("3.3-V LVTTL")),
        Subsignal("led2", Pins("AA4"), IOStandard("3.3-V LVTTL")),
        Subsignal("led3", Pins("A11"), IOStandard("3.3-V LVTTL")),
        Subsignal("led4", Pins("A12"), IOStandard("3.3-V LVTTL")),
        Subsignal("led5", Pins("B13"), IOStandard("3.3-V LVTTL")),
        Subsignal("led6", Pins("A14"), IOStandard("3.3-V LVTTL")),

        # UART part
        Subsignal("tx",   Pins("D17"), IOStandard("3.3-V LVTTL")),
        Subsignal("rx",   Pins("E17"), IOStandard("3.3-V LVTTL")),
    ),
]

# Platform -----------------------------------------------------------------------------------------

class Platform(AlteraPlatform):
    default_clk_name = "clk100"
    default_clk_period = 10

    def __init__(self):
        AlteraPlatform.__init__(self, "EP4CGX75CF23I7", _io)

    def create_programmer(self):
        return USBBlaster()
