from litescope.software.driver.reg import *

from liteusb.software.ftdi import FTDIComDevice

# TODO: create LiteX comm_ftdi or comm_usb

class LiteUSBWishboneDriverFTDI:
    cmds = {
        "write": 0x01,
        "read":  0x02
    }
    def __init__(self, interface, mode, tag, addrmap=None, busword=8, debug=False):
        self.interface = interface
        self.mode = mode
        self.tag = tag
        self.debug = debug
        self.com = FTDIComDevice(self.interface,
                                 mode=mode,
                                 uart_tag=tag,
                                 dma_tag=16, # XXX FIXME
                                 verbose=debug)
        if addrmap is not None:
            self.regs = build_map(addrmap, busword, self.read, self.write)

    def open(self):
        self.com.open()

    def close(self):
        self.com.close()

    def read(self, addr, burst_length=1):
        datas = []
        msg = []
        self.com.uartflush()
        msg.append(self.cmds["read"])
        msg.append(burst_length)
        word_addr = addr//4
        msg.append((word_addr >> 24) & 0xff)
        msg.append((word_addr >> 16) & 0xff)
        msg.append((word_addr >> 8) & 0xff)
        msg.append((word_addr >> 0) & 0xff)
        self.com.uartwrite(msg)
        for i in range(burst_length):
            data = 0
            for k in range(4):
                data = data << 8
                data |= self.com.uartread()
            if self.debug:
                print("RD {:08X} @ {:08X}".format(data, addr + 4*i))
            datas.append(data)
        if burst_length == 1:
            return datas[0]
        else:
            return datas

    def write(self, addr, data):
        if isinstance(data, list):
            burst_length = len(data)
        else:
            burst_length = 1
            data = [data]
        msg = []
        msg.append(self.cmds["write"])
        msg.append(burst_length)
        word_addr = addr//4
        msg.append((word_addr >> 24) & 0xff)
        msg.append((word_addr >> 16) & 0xff)
        msg.append((word_addr >> 8) & 0xff)
        msg.append((word_addr >> 0) & 0xff)
        for i in range(len(data)):
            dat = data[i]
            for j in range(4):
                msg.append((dat >> 24) & 0xff)
                dat = dat << 8
            if self.debug:
                print("WR {:08X} @ {:08X}".format(data[i], addr + 4*i))
        self.com.uartwrite(msg)


def LiteUSBWishboneDriver(chip="ft2232h", *args, **kwargs):
    drivers = {
        "ft2232h": LiteUSBWishboneDriverFTDI
    }
    return drivers[chip](*args, **kwargs)
