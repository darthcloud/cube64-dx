#!/usr/bin/env python3
from crc import bin, pickle

# Create a mapping from valid addresses to the resulting start address on the bus.
# We start by mapping each valid address to 0x0000...
addrmap = {}
for code in open("valid.addresses").readlines():
    addrmap[int(code)] = 0

# Set the proper bits read from the individual map files
for pin in range(5, 15):
    for code in open("a%d.map" % pin).readlines():
        addrmap[int(code)] = addrmap[int(code)] | (1 << pin)

# Reverse the mapping, so we get a map from addresses to the codes that generate them
codemap = {}
for code, addr in addrmap.items():
    codemap.setdefault(addr,[]).append(code)

# Print the results- address first, then list of codes, sorted by address
addrs = sorted(codemap.keys())
vectors = {}
for addr in addrs:
    # Sort the list of codes
    codes = sorted(codemap[addr])
    print("0x{:04X} :  {}".format(addr, "  ".join(["0x%04X (%s %s)" % (code, bin(code >> 5, 11), bin(code, 5)) for code in codes])))

    # It turns out that each address has exactly two codes, because
    # there's an extra address bit I haven't found on the bus connector yet.
    # Generate test vectors using this knowledge.
    vectors[addr] = codes[0]
    vectors[addr | 0x8000] = codes[1]

# Save the test vectors
pickle.dump(vectors, open("address_test_vectors.p", "wb"), -1)

### The End ###


