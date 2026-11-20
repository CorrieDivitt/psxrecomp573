#!/usr/bin/env python3
"""Seed function pointers stored in the data of the two programs the 700B01
BIOS loads into RAM (GX700 CHECK, boot shell). A word counts when it points
into that program's own text, is 4-aligned, and the target is preceded by a
function epilogue (`jr $ra` two words before it, i.e. the previous function's
jr + delay slot), which is how these PsyQ-built images lay functions out."""
import json, struct, sys
rom, seedf = sys.argv[1], sys.argv[2]
d = open(rom, "rb").read()
WIN = [("GX700 CHECK", 0x28800, 0x80010000, 0x10000), ("Boot shell", 0x60800, 0x80380000, 0x18000)]
s = json.load(open(seedf))
have = {int(x["address"], 16) for x in s["seeds"]}
added = 0
for name, off, ram, ln in WIN:
    img = d[off:off + ln]
    words = struct.unpack("<%dI" % (ln // 4), img)
    for i, w in enumerate(words):
        if not (ram <= w < ram + ln) or w & 3:
            continue
        t = (w - ram) // 4
        if t < 2 or words[t - 2] != 0x03E00008:
            continue
        r = 0xBFC00000 + off + (w - ram)
        if r in have:
            continue
        s["seeds"].append({"address": "0x%08X" % r, "label": "ptr_%08X" % w,
                           "rationale": "%s data word at 0x%08X points to 0x%08X, after a jr $ra epilogue" % (name, ram + i * 4, w)})
        have.add(r); added += 1
# Function addresses materialised in code: lui rX,hi then addiu/ori rY,rX,lo
# within 8 instructions (the PsyQ driver tables are filled this way at run
# time, e.g. GX700 CHECK's table at *(0x8001E910)).
for name, off, ram, ln in WIN:
    img = d[off:off + ln]
    words = struct.unpack("<%dI" % (ln // 4), img)
    lui = {}
    for i, x in enumerate(words):
        op = x >> 26
        if op == 0x0F:
            lui[(x >> 16) & 31] = ((x & 0xFFFF) << 16, i)
            continue
        if op in (0x09, 0x0D):
            rs = (x >> 21) & 31
            if rs in lui and i - lui[rs][1] <= 8:
                lo = x & 0xFFFF
                v = (lui[rs][0] + ((lo ^ 0x8000) - 0x8000)) & 0xFFFFFFFF if op == 0x09 else lui[rs][0] | lo
                if ram <= v < ram + ln and not v & 3:
                    t = (v - ram) // 4
                    if t >= 2 and words[t - 2] == 0x03E00008:
                        r = 0xBFC00000 + off + (v - ram)
                        if r not in have:
                            s["seeds"].append({"address": "0x%08X" % r, "label": "mat_%08X" % v,
                                               "rationale": "%s builds 0x%08X with lui/%s at 0x%08X; target follows a jr $ra epilogue" % (name, v, "addiu" if op == 9 else "ori", ram + i * 4)})
                            have.add(r); added += 1
s["seeds"].sort(key=lambda e: int(e["address"], 16)); s["seed_count"] = len(s["seeds"])
json.dump(s, open(seedf, "w"), indent=2)
print("added", added)
