#!/usr/bin/env python
#
# Create a map of valid addresses. RA4 on the serial_bridge should
# be connected to the WR pin on the controller bus.
#

from devices import SerialBridge

b = SerialBridge()
f = open("valid.addresses", "w")

for addr in xrange(0x10000):
    count = b.busWrite([0]*32, addr)[0]
    if count:
        f.write("%d\n" % addr)
        print "%d  ******" % addr
    else:
        print "%d" % addr

### The End ###
