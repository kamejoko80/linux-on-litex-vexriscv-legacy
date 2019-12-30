# This file is Copyright (c) 2013 Florent Kermarrec <florent@enjoy-digital.fr>
# License: BSD

from litex.build.generic_platform import *
from litex.build.altera import AlteraPlatform
from litex.build.altera.programmer import USBBlaster

# IOs ----------------------------------------------------------------------------------------------

_io = [
    ("clk50", 0, Pins("T2"), IOStandard("3.3-V LVTTL")),
 
    ("user_led", 0, Pins("AA15"), IOStandard("3.3-V LVTTL")),
  # ("user_led", 1, Pins("AB15"), IOStandard("3.3-V LVTTL")),
  # ("user_led", 2, Pins("AA16"), IOStandard("3.3-V LVTTL")),
  # ("user_led", 3, Pins("AB16"), IOStandard("3.3-V LVTTL")),
  # ("user_led", 4, Pins("AA17"), IOStandard("3.3-V LVTTL")),
  # ("user_led", 5, Pins("AB17"), IOStandard("3.3-V LVTTL")),
  # ("user_led", 6, Pins("AA18"), IOStandard("3.3-V LVTTL")),
    ("user_led", 7, Pins("AB18"), IOStandard("3.3-V LVTTL")),

    ("led0", 0, Pins("E4"), IOStandard("3.3-V LVTTL")),

    ("key", 0, Pins("W13"), IOStandard("3.3-V LVTTL")),
    ("key", 1, Pins("Y13"), IOStandard("3.3-V LVTTL")),

    ("sw", 0, Pins("AA19"), IOStandard("3.3-V LVTTL")),
    ("sw", 1, Pins("AB19"), IOStandard("3.3-V LVTTL")),
    ("sw", 2, Pins("AA20"), IOStandard("3.3-V LVTTL")),
    ("sw", 3, Pins("AB20"), IOStandard("3.3-V LVTTL")),

    ("serial", 0,
        Subsignal("tx", Pins("AB14"), IOStandard("3.3-V LVTTL")),
        Subsignal("rx", Pins("AB13"), IOStandard("3.3-V LVTTL"))
    ),

    ("MyUart", 0, Pins("AB15"), IOStandard("3.3-V LVTTL")),

    #("canif", 0,
    #    Subsignal("tx", Pins("AA16"), IOStandard("3.3-V LVTTL")),
    #    Subsignal("rx", Pins("AB16"), IOStandard("3.3-V LVTTL")),
    #    Subsignal("boo", Pins("AA17"), IOStandard("3.3-V LVTTL")),
    #    Subsignal("irq", Pins("AB17"), IOStandard("3.3-V LVTTL")),
    #    Subsignal("clkout", Pins("AA18"), IOStandard("3.3-V LVTTL"))
    #),

    ("spi", 0,
        Subsignal("sclk", Pins("AA16"), IOStandard("3.3-V LVTTL")), # U8 13 -> PA5 
        Subsignal("miso", Pins("AB16"), IOStandard("3.3-V LVTTL")), # U8 14 -> PA6
        Subsignal("mosi", Pins("AA17"), IOStandard("3.3-V LVTTL")), # U8 15 -> PA7
        Subsignal("csn", Pins("AB17"), IOStandard("3.3-V LVTTL")),  # U8 16 -> PA4
        Subsignal("irq", Pins("AA18"), IOStandard("3.3-V LVTTL")),  # U8 17
    ),    

    ("sdram_clock", 0, Pins("Y6"), IOStandard("3.3-V LVTTL")),
    ("sdram", 0,
        Subsignal("a", Pins("V2 V1 U2 U1 V3 V4 Y2 AA1 Y3 V5 W1 Y4 V6")),
        Subsignal("ba", Pins("Y1 W2")),
        Subsignal("cs_n", Pins("AA3")),
        Subsignal("cke", Pins("W6")),
        Subsignal("ras_n", Pins("AB3")),
        Subsignal("cas_n", Pins("AA4")),
        Subsignal("we_n", Pins("AB4")),
        Subsignal("dq", Pins("AA10 AB9 AA9 AB8 AA8 AB7 AA7 AB5 Y7 W8 Y8 V9 V10 Y10 W10 V11")),
        Subsignal("dm", Pins("AA5 W7")),
        IOStandard("3.3-V LVTTL")
    ),

    ("epcs", 0,
        Subsignal("data0", Pins("Y22")),
        Subsignal("dclk", Pins("Y21")),
        Subsignal("ncs0", Pins("W22")),
        Subsignal("asd0", Pins("W21")),
        IOStandard("3.3-V LVTTL")
    ),

    ("i2c", 0,
        Subsignal("sclk", Pins("AA13")),
        Subsignal("sdat", Pins("AA14")),
        IOStandard("3.3-V LVTTL")
    ),

    ("g_sensor", 0,
        Subsignal("cs_n", Pins("V22")),
        Subsignal("int", Pins("V21")),
        IOStandard("3.3-V LVTTL")
    ),

    ("adc", 0,
        Subsignal("cs_n", Pins("U22")),
        Subsignal("saddr", Pins("U21")),
        Subsignal("sclk", Pins("R22")),
        Subsignal("sdat", Pins("R21")),
        IOStandard("3.3-V LVTTL")
    ),

    ("gpio_0", 0,
        Pins("P22 P21 N22 N21 M22 M21 L22 L21 K22 K21 J22 J21 H22 H21 F22 F21",
            "E22 E21 D22 D21 C22 C21 B22 B21 N20 N19 M20 M19 R1 R2 P1 P2",
            "N1 N2"),
        IOStandard("3.3-V LVTTL")
    ),
    ("gpio_1", 0,
        Pins("M1 M2 J1 J2 H1 H2 F1 F2 E1 D2 C1 C2 B1 B2 B3 A3",
            "B4 A4 C4 C3 B5 A5 B6 A6 B7 A7 B8 A8 B9 A9 B10 A10",
            "B13 A13"),
        IOStandard("3.3-V LVTTL")
    ),
    ("gpio_2", 0,
        Pins("B14 A14 B15 A15 B16 A16 B17 A17 B18 A18 B19 A19 B20"),
        IOStandard("3.3-V LVTTL")
    ),
]

# Platform -----------------------------------------------------------------------------------------
class Platform(AlteraPlatform):
    default_clk_name = "clk50"
    default_clk_period = 20
 
    def __init__(self):
        AlteraPlatform.__init__(self, "EP4CE15F23C8", _io)
