#!/usr/bin/env python
#
# Implementation of a CRC algorithm compatible with the one used
# for bus reads and writes on N64 controllers. This algorithm was
# created by examining the output of the N64's CRC for several test
# vectors using the process outlined below. No details of Nintendo's
# implementation are available to me.
#
# 1. I first tried using a standard CRC-8 algorithm with every possible
#    polynomial, but none matched the controller's output. This apparently
#    isn't a conventional CRC algorithm.
#
# 2. I first noticed that, for a packet with two bits set, the CRC is
#    the Exclusive NOR of the CRCs for packets with each of the two bits
#    set individually.
#
# 3. This XNOR pattern extends to any number of bits set. Extrapolating
#    from this, it's possible to build a table with inverted CRCs for
#    each possible bit set, and use that to calculate a CRC for any message.
#    The CRC is initialized to 0xFF, then XOR'ed by the table entry for each
#    bit set in the packet.
#
# 4. A few patterns were found in the 256-byte table that lets us reduce
#    it to a 32-byte table.
#

import cPickle, sys


def extractTable(vectors):
    """Extract a 256-byte table from the test vectors, that can be used to
       compute any CRC. This table is the inverted CRC generated for every
       possible message with exactly one bit set.
       """
    table = []
    for byte in xrange(32):
        for bit in xrange(8):
            packet = [0] * 32
            packet[byte] = 1<<(7-bit)
            table.append(vectors[tuple(packet)] ^ 0xFF)
    return table


def verifyAlgorithm(vectors, f, *args, **kwargs):
    """Run the given algorithm with test vectors plus the given extra parameters,
       showing the results in a compact form.
       """
    print "Testing algorithm"
    passed = 0
    showFailures = 1

    vectorlist = vectors.keys()
    vectorlist.sort()
    for vector in vectorlist:
        result = f(vector, *args, **kwargs)
        expected = vectors[vector]
        if result == expected:
            passed += 1
        elif showFailures:
            print "Failed: %s => expected %02X, got %02X" % (
                " ".join(["%02X" % b for b in vector]), expected, result)
            showFailures = 0
    print "%d/%d tests passed" % (passed, len(vectors))


def bin(i, width=8):
    return ''.join([ "01"[i & (1<<(width-1-bit)) != 0]
                     for bit in xrange(width) ])


def tableCompress(table):
    """Given the 256-byte table, this returns a smaller 32-byte table
       with what's left over after decoding a few patterns in the original
       """
    b = 0
    prev = 0xCD
    smallTable = []
    for byte in xrange(32):
        output = 0
        for bit in xrange(8):
            e = table[b]
            b += 1

            e = ((e<<bit) | (e>>(8-bit))) & 0xFF
            c = e ^ prev
            prev = e
            c = ((c>>bit) | (c<<(8-bit))) & 0xFF
            assert (c == 0x00) or (c == 0x42)

            if c:
                output |= 1<<(7-bit)
        smallTable.append(output)
    return smallTable


def largeTableCRC(packet, table):
    """Table-driven reimplementation of Nintendo's CRC using a 256-byte table"""
    crc = 0xFF
    for byte in xrange(32):
        for bit in xrange(8):
            if packet[byte] & 1<<(7-bit):
                crc ^= table[byte*8 + bit]
    return crc


def smallTableCRC(packet, table=None):
    """Table-driven reimplementation of Nintendo's CRC using a 32-byte table"""
    if not table:
        # This is the one it should generate for a correct data set
        table = [224, 42, 204, 69, 167, 33, 41, 54, 227, 208, 106, 50, 236, 58, 243,
                 239, 192, 85, 152, 139, 78, 66, 82, 109, 199, 160, 212, 101, 216, 117, 231, 223]

    crc = 0xFF
    y = 0xCD
    for byte in xrange(32):
        tableByte = table[byte]
        packetByte = packet[byte]

        for bit in xrange(8):
            if tableByte & 0x80:
                y ^= 0x42
            if packetByte & 0x80:
                crc ^= y

            tableByte <<= 1
            packetByte <<= 1
            y = ((y >> 1) | (y << 7)) & 0xFF

    return crc


if __name__ == "__main__":
    testVectors = cPickle.load(open("crc_test_vectors.p", "rb"))

    print "Large-table CRC:"
    table = extractTable(testVectors)
    verifyAlgorithm(testVectors, largeTableCRC, table)

    print "Small-table CRC:"
    table = tableCompress(table)
    print table
    verifyAlgorithm(testVectors, smallTableCRC, table)



### The End ###
