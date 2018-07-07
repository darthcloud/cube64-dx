#!/usr/bin/env python3
#
# Using the valid.addresses file created by find_valid_addresses,
# generate another map of just the addresses that generate a pulse
# on a particular address line. This creates files that will be
# used to generate a  mapping from resulting address to the codes that
# can generate it.
#

from devices import SerialBridge
import sys

b = SerialBridge()
addrs = map(int, open("valid.addresses").readlines())
f = open(sys.argv[1], "w")

for addr in addrs:
    count = int.from_bytes(b.busWrite(bytes([0])*32, addr)[0], byteorder='big')
    if count:
        f.write("%d\n" % addr)
        print("{}  ******".format(addr))
    else:
        print("{}".format(addr))

### The End ###
