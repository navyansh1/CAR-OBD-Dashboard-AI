"""Offline hypothesis search over logs/re_samples.jsonl: which bytes equal a known value under a standard scaling."""
import json, os, sys, collections
import numpy as np
S = collections.defaultdict(list)
import cars
SRC = sys.argv[1] if len(sys.argv) > 1 else os.path.join(cars.car_dir(), "re_samples.jsonl")
for line in open(SRC):
    d = json.loads(line); S[(d["hdr"], d["req"])].append((bytes.fromhex(d["hex"]), d["refs"]))
SCALES = {
  "temp": [(1, -40, "raw-40"), (0.75, -48, "raw*0.75-48"), (0.5, -40, "raw/2-40"), (1, -50, "raw-50")],
  "volt": [(0.1, 0, "raw/10"), (0.08, 0, "raw*0.08"), (0.068235, 0, "raw*0.0682"), (0.0625, 0, "raw/16"), (0.01, 0, "raw/100"), (0.001, 0, "raw/1000"), (1/128, 0, "raw/128")],
  "rpm": [(1, 0, "raw"), (0.25, 0, "raw/4"), (0.5, 0, "raw/2"), (0.125, 0, "raw/8"), (10, 0, "raw*10"), (40, 0, "raw*40")],
  "kpa": [(1, 0, "raw"), (0.5, 0, "raw/2"), (0.1, 0, "raw/10"), (0.390625, 0, "raw*100/256"), (0.003906, 0, "raw/256"), (0.01, 0, "raw/100")],
  "pct": [(100/255, 0, "raw*100/255"), (0.5, 0, "raw/2"), (0.1, 0, "raw/10"), (100/1024, 0, "raw*100/1024"), (0.0015259, 0, "raw*100/65536")],
  "deg": [(0.5, -64, "raw/2-64"), (0.75, -48, "x"), (0.375, -48, "raw*0.375-48"), (0.1, 0, "raw/10"), (0.5, 0, "raw/2"), (1, -64, "raw-64")],
}
KIND = {"COOLANT_TEMP":"temp","INTAKE_TEMP":"temp","AMBIANT_AIR_TEMP":"temp","CATALYST_TEMP_B1S1":"temp",
        "CONTROL_MODULE_VOLTAGE":"volt","ELM_VOLTAGE":"volt","O2_B1S1":"volt","O2_B1S2":"volt",
        "RPM":"rpm","INTAKE_PRESSURE":"kpa","BAROMETRIC_PRESSURE":"kpa","THROTTLE_POS":"pct","ENGINE_LOAD":"pct","ABSOLUTE_LOAD":"pct",
        "ACCELERATOR_POS_D":"pct","TIMING_ADVANCE":"deg"}
TOL = {"temp": 1.6, "volt": 0.25, "rpm": 30, "kpa": 2.5, "pct": 2.5, "deg": 2.0}
out = []
for (hdr, req), rows in S.items():
    L = min(len(r[0]) for r in rows); M = np.array([list(r[0][:L]) for r in rows], dtype=float)
    fields = {}
    for i in range(L):
        fields[(i, "u8")] = M[:, i]
        if i + 1 < L:
            fields[(i, "u16be")] = M[:, i] * 256 + M[:, i + 1]; fields[(i, "u16le")] = M[:, i + 1] * 256 + M[:, i]
    for ref, kind in KIND.items():
        y = np.array([r[1].get(ref, np.nan) for r in rows], dtype=float)
        if np.isnan(y).all(): continue
        ok = ~np.isnan(y)
        for (i, typ), x in fields.items():
            if typ != "u8" and x[ok].max() < 256: continue          # 16-bit only if it uses the high byte
            if x[ok].std() == 0 and kind in ("rpm", "deg"): continue
            for a, b, name in SCALES[kind]:
                v = a * x[ok] + b
                err = np.abs(v - y[ok])
                if np.median(err) <= TOL[kind] and err.max() <= TOL[kind] * 3 and (y[ok].mean() != 0):
                    corr = float(np.corrcoef(x[ok], y[ok])[0, 1]) if x[ok].std() > 0 and y[ok].std() > 0 else None
                    out.append((hdr, req, i, typ, ref, name, round(float(np.median(err)), 3), round(float(v[-1]), 2), round(float(y[ok][-1]), 2), corr, int(x[ok].std() > 0)))
out.sort(key=lambda r: (r[4], r[6]))
print(f"{'blk':8} {'off':>3} {'type':6} {'matches':24} {'formula':14} {'med err':>7} {'decoded':>8} {'pid':>8} {'r':>6} varies")
for r in out:
    print(f"{r[1]:8} {r[2]:>3} {r[3]:6} {r[4]:24} {r[5]:14} {r[6]:>7} {r[7]:>8} {r[8]:>8} {('%.2f'%r[9]) if r[9] is not None else '  —':>6} {'yes' if r[10] else 'no'}")
print(len(out), "hypotheses;", sum(len(v) for v in S.values()), "samples")
