#!/usr/bin/env python3
"""Add the recompiler's 'cross-function target ... in no emitted function'
addresses to the 700B01 seed file, mapped from RAM back to ROM through the
copy windows. These are direct jal/branch targets in already-emitted code."""
import json, re, sys
WIN = [(0x80010000, 0x10000, 0xBFC28800), (0x80380000, 0x18000, 0xBFC60800),
       (0x00000500, 0x7B00, 0xBFC10000)]
log, seedf = sys.argv[1], sys.argv[2]
d = json.load(open(seedf))
have = {int(s["address"], 16) for s in d["seeds"]}
added, outside = 0, set()
for t in sorted({int(m, 16) for m in re.findall(r"cross-function target (0x[0-9A-F]+)", open(log).read())}):
    rom = None
    for ram, ln, src in WIN:
        if ram <= t < ram + ln:
            rom = src + (t - ram)
    if rom is None and any(src <= t < src + ln for ram, ln, src in WIN):
        rom = t  # already a ROM key (jump-table target published in ROM form)
    if rom is None:
        outside.add(t); continue
    if rom not in have:
        d["seeds"].append({"address": "0x%08X" % rom, "label": "xfn_%08X" % t,
                           "rationale": "direct target 0x%08X from emitted code" % t})
        have.add(rom); added += 1
d["seeds"].sort(key=lambda s: int(s["address"], 16))
d["seed_count"] = len(d["seeds"])
json.dump(d, open(seedf, "w"), indent=2)
print("added", added, "outside windows", [hex(x) for x in sorted(outside)])
