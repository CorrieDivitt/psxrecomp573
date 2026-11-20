#!/usr/bin/env python3
"""Seed every code pointer in the 700B01 ROM's function-pointer tables
(A0 table, kernel B0/C0 tables, device tables). RAM pointers into the kernel
ramtext (0x500-0x7500, any KSEG) map back to ROM 0xBFC10000 + (x - 0x500)."""
import json, struct, sys
rom, seedf = sys.argv[1], sys.argv[2]
d = open(rom, "rb").read()
w = struct.unpack("<%dI" % (len(d) // 4), d)
TABLES = [(0x1CC, 0x20C), (0x465C, 0x4960), (0x9EA8, 0x9F10), (0xA44C, 0xA490),
          (0xA52C, 0xA574), (0x10174, 0x10790), (0x1674C, 0x167E0), (0x169C0, 0x16CF0)]
# Device tables (0xA44C, 0xA52C) start with a pointer to the device name
# string ("CONS", "MEMO"): data, not code.
EXCLUDE = {0xBFC0A494, 0xBFC0A574}
s = json.load(open(seedf))
have = {int(x["address"], 16) for x in s["seeds"]}
added = 0
for lo, hi in TABLES:
    for off in range(lo, hi, 4):
        x = w[off // 4]
        if 0xBFC00000 <= x < 0xBFC80000:
            r = x
        elif 0x500 <= (x & 0x1FFFFFFF) < 0x7500:
            r = 0xBFC10000 + (x & 0x1FFFFFFF) - 0x500
        else:
            continue
        if r & 3 or r in have or r in EXCLUDE:
            continue
        s["seeds"].append({"address": "0x%08X" % r, "label": "tbl_%08X" % r,
                           "rationale": "code pointer 0x%08X in ROM table 0x%X-0x%X (word at 0x%X)" % (x, lo, hi, off)})
        have.add(r); added += 1
s["seeds"].sort(key=lambda e: int(e["address"], 16)); s["seed_count"] = len(s["seeds"])
json.dump(s, open(seedf, "w"), indent=2)
print("added", added)
