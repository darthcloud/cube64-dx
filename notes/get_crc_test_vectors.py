#!/usr/bin/env python
#
# Using a microcontroller with the serial_bridge firmware installed,
# this sends several controller bus writes with different content.
# It reads back the controller-generated CRCs for each packet, storing
# the packet and CRC as test vectors for our reverse engineered CRC
# algorithm.
#
# The test vectors are stored as a pickled dictionary, mapping a 32-tuple
# of packet bytes to a CRC value.
#
# The test vectors include all zeros, all ones, a one in each possible
# bit position, pairs of ones, and several random packets. The packets with
# a one in each  possible bit position turn out to be very important
# for reproducing the CRC results.
#
# --Micah Dowty <micah@navi.cx>
#

import random, cPickle
from devices import SerialBridge


def packetGenerator():
    """A generator that yields all the packets we're going to test"""
    # Seed the PRNG with a constant so our output is predictable
    random.seed(12345)

    # All zeros
    packet = [0] * 32
    yield packet

    # All ones
    packet = [0xFF] * 32
    yield packet

    # Each bit set individually
    for byte in xrange(32):
        for bit in xrange(8):
            packet = [0] * 32
            packet[byte] = 1<<(7-bit)
            yield packet

    # Random pairs of bits set
    for i in xrange(500):
        packet = [0] * 32
        for j in xrange(2):
            byte = random.randint(0, 31)
            bit = random.randint(0, 7)
            packet[byte] |= 1<<(7-bit)
        yield packet

    # Completely random packets
    for i in xrange(500):
        yield [random.randint(0,255) for j in xrange(32)]


if __name__ == "__main__":
    filename = "crc_test_vectors.p"
    b = SerialBridge()
    vec = b.genVectors(packetGenerator())
    cPickle.dump(vec, open(filename, "wb"), -1)
    print "Saved to %s" % filename

### The End ###
