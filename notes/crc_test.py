#!/usr/bin/env python3
"""
# Print CRC calculation process to help validate firmware.
#
# --Jacques Gagnon <darthcloud@gmail.com>
"""

import pickle
from crc import extractTable, reversedLargeTableCRC

def main():
    """ MAIN """
    test_vectors = pickle.load(open("crc_test_vectors.p", "rb"))

    table = list(reversed(extractTable(test_vectors)))

    packet = bytes([0x80]) * 32
    #packet = bytes([0x00]) * 32

    reversedLargeTableCRC(packet, table, 1)

if __name__ == "__main__":
    main()

### The End ###
