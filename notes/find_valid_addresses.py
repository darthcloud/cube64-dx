#!/usr/bin/env python3
#
# Create a map of valid addresses. RA4 on the serial_bridge should
# be connected to the WR pin on the controller bus.
#

from devices import SerialBridge

b = SerialBridge()
f = open("valid.addresses", "w")

for addr in range(0x10000):
    count = b.busWrite(bytes([0])*32, addr)[0]
    if count:
        f.write("%d\n" % addr)
        print("{}  ******".format(addr))
    else:
        print("{}".format(addr))

### The End ###
