# This file is Copyright (c) 2019 Henry Dang <henrydang@fossil.com>
# License: BSD

from litex.build.generic_platform import *
from litex.build.xilinx import XilinxPlatform, VivadoProgrammer

# IOs ----------------------------------------------------------------------------------------------

_io = [

    ("clk50", 0, Pins("D18"), IOStandard("LVCMOS33")),
    ("clk", 0, Pins("D18"), IOStandard("LVCMOS33")), # Alias of clk50

    ("serial", 0,
        Subsignal("rx", Pins("B22")),   # U4 08
        Subsignal("tx", Pins("A24")),   # U4 10
        IOStandard("LVCMOS33")
    ),

    ("serial_test", 0,
        Subsignal("rx", Pins("B25")),   # U4 12
        Subsignal("tx", Pins("C22")),   # U4 14
        IOStandard("LVCMOS33")
    ),

    ("spi", 0,
        Subsignal("sck_0", Pins("A22")),  # U4 07 (sck line  0)
        Subsignal("miso_0", Pins("A23")), # U4 09 (miso line 0)
        Subsignal("mosi_0", Pins("A25")), # U4 11 (mosi line 0)
        Subsignal("csn_0", Pins("C23")),  # U4 13 (csn line  0)
        Subsignal("irq", Pins("B24")),    # U4 15
        IOStandard("LVCMOS33"),
    ),

    ("spi", 1,
        # line 0
        Subsignal("sck_0",  Pins("B26")), # U4 17 (sck line  0)
        Subsignal("miso_0", Pins("D24")), # U4 19 (miso line 0)
        Subsignal("mosi_0", Pins("D25")), # U4 21 (mosi line 0)
        Subsignal("csn_0",  Pins("D26")), # U4 23 (csn line  0)
        # line 1
        Subsignal("sck_1",  Pins("G26")), # U4 25 (sck line  1)
        Subsignal("miso_1", Pins("F25")), # U4 27 (miso line 1)
        Subsignal("mosi_1", Pins("J26")), # U4 29 (mosi line 1)
        Subsignal("csn_1",  Pins("H24")), # U4 31 (csn line  1)
        # line 2
        Subsignal("sck_2",  Pins("K26")), # U4 33 (sck line  2)
        Subsignal("miso_2", Pins("L25")), # U4 35 (miso line 2)
        Subsignal("mosi_2", Pins("M26")), # U4 37 (mosi line 2)
        Subsignal("csn_2",  Pins("M25")), # U4 39 (csn line  2)
        # line 3
        Subsignal("sck_3",  Pins("P26")), # U4 41 (sck line  3)
        Subsignal("miso_3", Pins("P25")), # U4 43 (miso line 3)
        Subsignal("mosi_3", Pins("U25")), # U4 45 (mosi line 3)
        Subsignal("csn_3",  Pins("T25")), # U4 47 (csn line  3)
        # IRQ pin
        Subsignal("irq",    Pins("W26")), # U4 49
        IOStandard("LVCMOS33"),
    ),

    ("spi_slave", 0,
        # SPI slave part
        Subsignal("sck", Pins("AB26")), # U4 60
        Subsignal("miso", Pins("AA24")),# U4 58
        Subsignal("mosi", Pins("Y25")), # U4 56
        Subsignal("csn", Pins("V24")),  # U4 54
        Subsignal("int1", Pins("W25")), # U4 52
        Subsignal("int2", Pins("V26")), # U4 50
       #Subsignal("irq", Pins("H2")),   #
        Subsignal("led0", Pins("J26")), # LED0 U4 29
        Subsignal("led1", Pins("H24")), # LED1 U4 31
        Subsignal("led2", Pins("B19")), # LED2
        Subsignal("led3", Pins("A19")), # LED3
        Subsignal("led4", Pins("K26")), # LED4 U4 33
        Subsignal("led5", Pins("L25")), # LED5 U4 35
        Subsignal("led6", Pins("M26")), # LED6 U4 37
        # UART part
        Subsignal("tx", Pins("M25")),   # U4 39
        Subsignal("rx", Pins("P26")),   # U4 41
        IOStandard("LVCMOS33"),
    ),
]

# Connectors ---------------------------------------------------------------------------------------

_connectors = [

]

# Platform -----------------------------------------------------------------------------------------

class Platform(XilinxPlatform):
    default_clk_name = "clk50"
    default_clk_period = 20.0

    def __init__(self):
        XilinxPlatform.__init__(self, "xc7a100tfgg676-2", _io, _connectors, toolchain="vivado")
        self.toolchain.bitstream_commands = \
            ["set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]"]
        self.toolchain.additional_commands = \
            ["write_cfgmem -force -format bin -interface spix4 -size 16 "
             "-loadbit \"up 0x0 {build_name}.bit\" -file {build_name}.bin"]
        self.add_platform_command("set_property INTERNAL_VREF 0.675 [get_iobanks 34]")

    def create_programmer(self):
        return VivadoProgrammer(flash_part="MT25QL128")
