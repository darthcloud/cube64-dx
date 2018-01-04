#!/usr/bin/env python2
#
# Print CRC calculation process to help validate firmware.
#
# --Jacques Gagnon <darthcloud@gmail.com>
#

import cPickle
from crc import extractTable, verifyAlgorithm, reversedLargeTableCRC

if __name__ == "__main__":
    testVectors = cPickle.load(open("crc_test_vectors.p", "rb"))

    table = list(reversed(extractTable(testVectors)))

    packet = [0x80] * 32
    #packet = [0x00] * 32

    crc = reversedLargeTableCRC(packet, table, 1)

### The End ###
