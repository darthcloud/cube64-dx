#!/usr/bin/env python
#
# Read the memory pak's 32k SRAM to a binary file specified on the command line.
#
# --Micah Dowty <micah@navi.cx>
#

from bus import Bus
import sys

b = Bus()
if b.probe() != "memory":
    sys.exit(1)

f = open(sys.argv[1], "wb")

addr = 0x0000
while addr < 0x8000:
    sys.stdout.write("\r0x%04X (%.02f%%)" % (addr, addr * 100.0 / 0x8000))
    f.write(b.read(addr))
    addr += 32
sys.stdout.write("\n")

### The End ###
