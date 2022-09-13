#!/usr/bin/env python

import struct

font = {}

with open('HaxorMedium-13.bdf') as fp:
    bstr = None
    code = None

    for line in fp:
        line = line.strip().split()

        if line[0] == 'ENDCHAR':
            font[code] = bstr
            bstr = None

        elif line[0] == 'ENCODING':
            code = int(line[1])

        elif line[0] == 'BITMAP':
            bstr = []

        elif bstr is not None:
            bstr.append(int(line[0], 16))


empty = [0] * 16

with open('HaxorMedium-13.bin', 'wb') as fp:
    for i in range(256):
        glyph = font.get(i, empty)
        assert len(glyph) == 16

        for byte in glyph:
            fp.write(struct.pack('B', byte))


# vim:set sw=4 ts=4 et:
