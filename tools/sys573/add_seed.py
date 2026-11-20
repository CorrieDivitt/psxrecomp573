#!/usr/bin/env python3
"""add_seed.py SEEDS.json ADDR LABEL RATIONALE"""
import json, sys
f, addr, label, why = sys.argv[1], int(sys.argv[2], 16), sys.argv[3], sys.argv[4]
d = json.load(open(f))
if any(int(s["address"], 16) == addr for s in d["seeds"]):
    print("already seeded"); sys.exit(0)
d["seeds"].append({"address": "0x%08X" % addr, "label": label, "rationale": why})
d["seeds"].sort(key=lambda s: int(s["address"], 16)); d["seed_count"] = len(d["seeds"])
json.dump(d, open(f, "w"), indent=2); print("added", hex(addr))
