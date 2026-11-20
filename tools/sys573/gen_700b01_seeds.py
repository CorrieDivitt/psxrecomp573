#!/usr/bin/env python3
"""Turn Ghidra's 700b01_functions.tsv (from FM_Bios700B01.java) into a psxrecomp
seed file. Seeds are ROM addresses; code that runs from RAM is keyed by its
ROM source through the profile's copy windows.

Rules (see bios/700B01.toml):
 - kernel ramtext ROM 0xBFC10000-0xBFC18000: union of functions Ghidra found in
   the RAM block (0x500+) and in the ROM view. Same bytes, same offsets.
 - GX700 CHECK (ROM 0xBFC28800-0xBFC38800) and boot shell (0xBFC60800-
   0xBFC78800): only the RAM-block analysis counts. The ROM-view functions
   there were analysed at the wrong base and are dropped.
 - everything else in ROM: as found.
"""
import json, sys

KERNEL = (0xBFC10000, 0xBFC18000)
LOADED = [(0xBFC28800, 0xBFC38800), (0xBFC60800, 0xBFC78800)]

def main(tsv, out):
    seeds = {}
    dropped = 0
    for line in open(tsv, encoding="utf-8").read().splitlines()[1:]:
        block, addr, rom, name, size = line.split("\t")
        rom = int(rom, 16)
        if block == "ram" and any(lo <= rom < hi for lo, hi in LOADED):
            dropped += 1
            continue
        if block == "kernel_ram" and not (KERNEL[0] <= rom < KERNEL[1]):
            dropped += 1
            continue
        label = name if not name.startswith("FUN_") else "ghidra_%08X" % rom
        if rom in seeds and not seeds[rom]["label"].startswith("ghidra_"):
            continue
        seeds[rom] = {"address": "0x%08X" % rom, "label": label,
                      "rationale": "Ghidra function start (%s block)" % block}
    for rom, label, why in [(0xBFC00000, "reset", "MIPS R3000A reset vector"),
                            (0xBFC00180, "general_exception_bev1", "BEV=1 general exception vector")]:
        seeds.setdefault(rom, {"address": "0x%08X" % rom, "label": label, "rationale": why})
    data = {"schema": "psxrecomp phase2 seeds",
            "source": "Ghidra 12.1.4 analysis of 700b01.22g via FM_Bios700B01.java (Fighting-Mania-VR)",
            "seed_count": len(seeds),
            "seeds": [seeds[k] for k in sorted(seeds)]}
    json.dump(data, open(out, "w"), indent=2)
    print("seeds", len(seeds), "dropped", dropped)

if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
