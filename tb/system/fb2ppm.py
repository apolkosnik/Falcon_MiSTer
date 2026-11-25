#!/usr/bin/env python3
# The VME card's picture as the scaler shows it, from a simulator run: its log's last
# "fb:" line (format, size, base, stride) and "fb: pal" lines, and a --dump of guest memory
# that covers the frame buffer.  8 bits a pixel through the palette, or 16 bits (565/1555,
# low byte first).
#   fb2ppm.py LOG DUMP DUMP_ADDR OUT.ppm      (DUMP_ADDR in hex, as given to --dump)
import re, sys

log, dump, daddr, out = sys.argv[1], sys.argv[2], int(sys.argv[3], 16), sys.argv[4]
fb = None
pal = [(0, 0, 0)] * 256
for line in open(log, errors='replace'):
    m = re.match(r'fb: en (\d) format ([0-9a-f]+) (\d+)x(\d+) base ([0-9a-f]+) stride (\d+)', line)
    if m:
        fb = dict(en=int(m[1]), fmt=int(m[2], 16), w=int(m[3]), h=int(m[4]), base=int(m[5], 16), stride=int(m[6]))
    m = re.match(r'fb: pal ([0-9a-f]+) = ([0-9a-f]+)', line)
    if m:
        v = int(m[2], 16)
        pal[int(m[1], 16)] = (v >> 16, (v >> 8) & 255, v & 255)
if not fb or not fb['en']:
    sys.exit('no picture on at the end')
mem = open(dump, 'rb').read()
start = (fb['base'] & 0xFFFFFF) - daddr
depth = fb['fmt'] & 7
px = bytearray()
for y in range(fb['h']):
    row = start + y * fb['stride']
    for x in range(fb['w']):
        if depth == 3:
            px += bytes(pal[mem[row + x]])
        else:
            v = mem[row + 2 * x] | mem[row + 2 * x + 1] << 8
            if fb['fmt'] & 8:
                r, g, b = (v >> 10) & 31, (v >> 5) & 31, v & 31
                px += bytes((r << 3 | r >> 2, g << 3 | g >> 2, b << 3 | b >> 2))
            else:
                r, g, b = v >> 11, (v >> 5) & 63, v & 31
                px += bytes((r << 3 | r >> 2, g << 2 | g >> 4, b << 3 | b >> 2))
with open(out, 'wb') as f:
    f.write(b'P6\n%d %d\n255\n' % (fb['w'], fb['h']))
    f.write(px)
print('%dx%d format %02x from %06x stride %d -> %s' % (fb['w'], fb['h'], fb['fmt'], fb['base'] & 0xFFFFFF, fb['stride'], out))
