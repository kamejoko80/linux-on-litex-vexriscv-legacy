from litex.build.generic_platform import *
from litex.build.lattice import LatticePlatform

_io = [
    ("rgb_led", 0,
        Subsignal("r", Pins("41")),
        Subsignal("g", Pins("40")),
        Subsignal("b", Pins("39")),
        IOStandard("LVCMOS33")
    ),

    ("user_sw", 0, Pins("23"), IOStandard("LVCMOS33")),
    ("user_sw", 1, Pins("25"), IOStandard("LVCMOS33")),
    ("user_sw", 2, Pins("34"), IOStandard("LVCMOS33")),
    ("user_sw", 3, Pins("43"), IOStandard("LVCMOS33")),

    # The ICE40UP5K-B-EVN does not use the provided FT2232H chip to provide a
    # UART port. One must use their own USB-to-serial cable instead to get a UART.
    # We have chosen to use 48B and 51A for "tx" and "rx" respectively on Header B
    # to implement UART connections. The board comes unpopulated and will need to
    # have headers soldered.
    ("serial", 0,
        Subsignal("tx", Pins("9"), Misc("PULLUP")), # J3 16A Pin18
        Subsignal("rx", Pins("6")),                 # J3 13B Pin20
        IOStandard("LVCMOS33")
    ),

    # Only usable in PROG FLASH mode and J7 attached (see PCB silkscreen).
    ("spiflash", 0,
        Subsignal("cs_n", Pins("16"), IOStandard("LVCMOS33")),
        Subsignal("clk", Pins("15"), IOStandard("LVCMOS33")),
        Subsignal("mosi", Pins("14"), IOStandard("LVCMOS33")),
        Subsignal("miso", Pins("17"), IOStandard("LVCMOS33")),
    ),

    ("spi", 0,
        Subsignal("sclk", Pins("4"), IOStandard("LVCMOS33")),  # J3 8A     IOB_8A
        Subsignal("miso", Pins("3"), IOStandard("LVCMOS33")),  # J3 9B     IOB_9B
        Subsignal("mosi", Pins("48"), IOStandard("LVCMOS33")), # J3 4A     IOB_4A
        Subsignal("csn", Pins("45"), IOStandard("LVCMOS33")),  # J3 5B     IOB_5B  
        Subsignal("irq", Pins("47"), IOStandard("LVCMOS33")),  # J3 2A     IOB_2A
    ),

    ("spi_slave", 0,
        Subsignal("sck", Pins("20"), IOStandard("LVCMOS33")),  # J3 25B_G3 IOB_25B_G3
        Subsignal("miso", Pins("19"), IOStandard("LVCMOS33")), # J3 29B    IOB_29B
        Subsignal("mosi", Pins("18"), IOStandard("LVCMOS33")), # J3 31B    IOB_31B
        Subsignal("csn", Pins("11"), IOStandard("LVCMOS33")),  # J3 20A    IOB_20A  
        Subsignal("irq", Pins("10"), IOStandard("LVCMOS33")),  # J3 18A    IOB_18A
    ),     
    
    ("clk12", 0, Pins("35"), IOStandard("LVCMOS33"))
]

class Platform(LatticePlatform):
    default_clk_name = "clk12"
    default_clk_period = 83.333
 
    def __init__(self):
        LatticePlatform.__init__(self, "ice40-up5k-sg48", _io, toolchain="icestorm")