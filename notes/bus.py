#!/usr/bin/env python
#
# A higher level interface to the N64 controller's bus,
# using the CRC, address encoding, and low level interface
# implemented elsewhere here.
#
# --Micah Dowty <micah@navi.cx>
#

from addr_encoder import addrEncode
from crc import smallTableCRC
from devices import SerialBridge
import struct, time

class Bus:
    def __init__(self, bridge=None):
        if not bridge:
            bridge = SerialBridge()
        self.bridge = bridge

    def _checkCRC(self, crcString, dataString):
        crc = ord(crcString)
        myCrc = smallTableCRC([ord(c) for c in dataString])

        if crc == myCrc ^ 0xFF:
            # Inverted CRC, seems to indicate the controller pak is initialized
            return 1

        elif crc == myCrc:
            # Non-inverted CRC, controller pak not initialized
            return 0

        else:
            print "CRC error: got 0x%02X, calculated 0x%02X" % (crc, myCrc)

    def write(self, address, data):
        """Write a 32-byte block to the given (non-coded) address"""
        packet = struct.pack(">BH32s", 3, addrEncode(address), data)
        crc = self.bridge.write(packet, 1)[1]
        return self._checkCRC(crc, packet[3:])

    def read(self, address):
        """Read a 32-byte block from the given (non-coded) address"""
        reply = self.bridge.write(struct.pack(">BH", 2, addrEncode(address)), 33)[1]
        crc = reply[-1]
        data = reply[:-1]
        self._checkCRC(crc, data)
        return data

    def probe(self):
        """Initialize the controller and probe for the attached pak"""
        # Identify/initialize the controller itself. This seems
        # to be necessary for the controller to re-detect the controller pak
        # if it has been removed or inserted.
        controllerId = self.bridge.write(chr(0), 3)[1]
        if controllerId == "\x05\x00\x02":
            print "Identified an N64 controller with no controller pak"
            return
        elif controllerId == "\x05\x00\x01":
            print "Identified an N64 controller with controller pak"
        else:
            print "Unknown controller identity %r" % controllerId

        # Initialize the controller pak by writing to 0x8000
        if self.write(0x8000, "\xFE" * 32) == 1:
            print "Inverted CRC, controller pak is initialized"
        else:
            print "Non-inverted CRC, controller pak isn't initialized"
            return

        # Read back 0x8000 to identify the peripheral
        id = ord(self.read(0x8000)[0])

        if id == 0x80:
            print "Detected rumble pak"
            return "rumble"

        elif id == 0x00:
            print "Detected memory pak"
            return "memory"

        else:
            print "Hmm, got back 0x%02X from reading 0x8000 after init" % id


def rumbleDemo(b):
    print "Testing rumble pak"
    # Rumble motor on...
    b.write(0xC000, chr(1) * 32)
    time.sleep(0.2)
    # Rumble motor off...
    b.write(0xC000, chr(0) * 32)

def memoryDemo(b):
    print "Reading back RAM"
    addr = 0x0000
    while addr < 0x10000:
        print "0x%04X:  %s" % (addr, " ".join(["%02X" % ord(byte) for byte in b.read(addr)]))
        addr += 32

def demo():
    b = Bus()
    id = b.probe()

    if id == "memory":
        memoryDemo(b)
    elif id == "rumble":
        rumbleDemo(b)

if __name__ == "__main__":
    demo()

### The End ###
