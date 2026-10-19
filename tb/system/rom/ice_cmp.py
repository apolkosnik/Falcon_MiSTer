#!/usr/bin/env python3
# ice_cmp.py CORE.bin REF.bin: the depacker test ROM's result against the
# reference.  The output is written top-down from a4+$910430: every byte from
# there down to the core's a6 must match; the lowest point reached tells how
# far it got.
import struct, sys
c = open(sys.argv[1], 'rb').read(); r = open(sys.argv[2], 'rb').read()
marker, v, a5, a6, pc = struct.unpack('>IIIII', c[0x400:0x414])
lo, hi = 0x23FC2, 0x23FC2 + 0x910430
if marker == 0xD0E0600D:
    print('stopped by vector offset %03x (%s); a5 %08x a6 %08x pc %08x' % (v, 'the depacker returned' if v == 0x80 else 'AN EXCEPTION', a5, a6, pc))
else:
    print('still running at the end of the run')
# the lowest written point: below it the core memory still equals the input
i = hi - 1
while i >= lo and c[i] == r[i]:
    i -= 1
low = i + 1
# find whether anything at or above 'low' differs (it does not, by
# construction); report the mismatch at i, if it is inside the output written
# so far (where the core's memory is no longer the packed input)
print('output matches the reference from %08x down to %08x (%d bytes of %d)' % (hi, low, hi - low, hi - lo))
if low > lo:
    init_same = c[i] != r[i]
    print('below: core %s ref %s at %08x' % (c[i - 7:i + 1].hex(), r[i - 7:i + 1].hex(), i - 7))
