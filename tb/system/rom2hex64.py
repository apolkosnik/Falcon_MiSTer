#!/usr/bin/env python3
# Convert a big-endian Atari ROM image into the 64-bit little-endian word
# hex file the DDR3 model loads with $readmemh (one word per line, the
# byte at the lowest address in bits 7:0, as the real DDR3 holds it).
import sys
data = open(sys.argv[1], 'rb').read()
data += b'\xff' * ((-len(data)) % 8)
with open(sys.argv[2], 'w') as f:
    for i in range(0, len(data), 8):
        f.write(data[i:i+8][::-1].hex() + '\n')
