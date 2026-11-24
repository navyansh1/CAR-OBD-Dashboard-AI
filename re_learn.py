"""
Reverse-engineering helper for undocumented manufacturer data blocks.

Every time a block (e.g. engine ECU `21 01`, 32 bytes) is read, it is stored with
the standard OBD values decoded at the same moment. analyse() then:

  1. classifies each byte: constant / counter / bit flags / analog
  2. builds candidate fields at every offset: u8, s8, u16 big/little endian, s16 big endian
  3. correlates every candidate with every standard value (rpm, coolant, MAP, ...)
  4. for strong matches (|r| >= 0.95) fits  value = a * raw + b  and reports it

Static conditions (idling, nothing changing) cannot be correlated. Revving the
engine, warming up, or driving gives the analysis something to lock on to.
"""
import json, os, time
from collections import deque

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
def samples_file():
    import cars
    return os.path.join(cars.car_dir(), "re_samples.jsonl")

REFS = ["RPM", "SPEED", "COOLANT_TEMP", "INTAKE_TEMP", "INTAKE_PRESSURE", "THROTTLE_POS", "ENGINE_LOAD",
        "ABSOLUTE_LOAD", "TIMING_ADVANCE", "O2_B1S1", "O2_B1S2", "CONTROL_MODULE_VOLTAGE", "ELM_VOLTAGE",
        "SHORT_FUEL_TRIM_1", "LONG_FUEL_TRIM_1", "CATALYST_TEMP_B1S1", "AMBIANT_AIR_TEMP", "ACCELERATOR_POS_D",
        "THROTTLE_ACTUATOR", "RELATIVE_THROTTLE_POS", "COMMANDED_EQUIV_RATIO", "RUN_TIME", "EVAPORATIVE_PURGE",
        "BAROMETRIC_PRESSURE", "FUEL_LEVEL"]
MIN_N = 15          # with fewer than 25 samples a stronger correlation is required
MIN_R = 0.95
MIN_R_SMALL = 0.97
LIKELY_R = 0.85      # second tier: shown as "likely"
# Scalings Hyundai uses in its published engine-ECU definitions; a fit close to one of these is snapped to it.
HYUNDAI = {"SPEED": [("u8", 1.250196, 0)], "RPM": [("u16le", 0.25, 0)], "INTAKE_PRESSURE": [("u16le", 0.003906, 0)],
           "ENGINE_LOAD": [("u16le", 0.023438, 0)], "COOLANT_TEMP": [("u8", 0.75, -48)], "INTAKE_TEMP": [("u8", 0.75, -48)],
           "CONTROL_MODULE_VOLTAGE": [("u8", 0.068235, 0)], "BAROMETRIC_PRESSURE": [("u8", 0.5019608, 0)],
           "TIMING_ADVANCE": [("s8", 0.75, 0), ("s8", 0.5, 0)]}
# A reference must move at least this much during the samples before any match counts;
# otherwise tiny idle wobbles produce convincing-looking but meaningless fits.
MIN_SPAN = {"RPM": 600, "SPEED": 15, "COOLANT_TEMP": 6, "INTAKE_TEMP": 6, "INTAKE_PRESSURE": 15, "THROTTLE_POS": 8,
            "ENGINE_LOAD": 15, "ABSOLUTE_LOAD": 15, "TIMING_ADVANCE": 8, "O2_B1S1": 0.5, "O2_B1S2": 0.3,
            "CONTROL_MODULE_VOLTAGE": 0.8, "ELM_VOLTAGE": 0.8, "SHORT_FUEL_TRIM_1": 6, "LONG_FUEL_TRIM_1": 3,
            "CATALYST_TEMP_B1S1": 60, "AMBIANT_AIR_TEMP": 4, "ACCELERATOR_POS_D": 15, "THROTTLE_ACTUATOR": 8,
            "RELATIVE_THROTTLE_POS": 8, "COMMANDED_EQUIV_RATIO": 0.08, "RUN_TIME": 60, "EVAPORATIVE_PURGE": 10,
            "BAROMETRIC_PRESSURE": 3, "FUEL_LEVEL": 10}


class Learner:
    def __init__(self, maxlen=1500, path=None):
        self.path = path or samples_file()
        self.samples = {}        # (hdr, req) -> deque[(t, bytes, {ref: value})]
        self.maxlen = maxlen
        self.result = {"blocks": [], "fields": [], "updated": None, "samples": 0}
        self.load()
        self._fh = open(self.path, "a")

    def load(self):
        """Resume from earlier sessions so learning accumulates across drives."""
        try:
            with open(self.path) as f:
                for line in f:
                    try:
                        d = json.loads(line)
                        key = (d["hdr"], d["req"])
                        dq = self.samples.setdefault(key, deque(maxlen=self.maxlen))
                        dq.append((d["t"], bytes.fromhex(d["hex"]), d["refs"]))
                    except Exception:
                        continue
        except FileNotFoundError:
            pass

    def add(self, hdr, req, payload, refs, t=None):
        t = t or time.time()
        key = (hdr, req)
        dq = self.samples.get(key)
        if dq is None:
            dq = self.samples[key] = deque(maxlen=self.maxlen)
        r = {k: refs[k] for k in REFS if refs.get(k) is not None}
        dq.append((t, bytes(payload), r))
        self._fh.write(json.dumps({"t": round(t, 3), "hdr": hdr, "req": req, "hex": bytes(payload).hex(), "refs": r}) + "\n")

    def flush(self):
        try:
            self._fh.flush()
        except Exception:
            pass

    @staticmethod
    def classify(col):
        vals = np.unique(col)
        if len(vals) == 1:
            return "constant"
        d = np.diff(col.astype(int))
        if len(col) >= 8 and (np.all(d >= 0) or np.all((d >= 0) | (d < -200))) and len(vals) > 3:
            return "counter"
        if len(vals) <= 8:
            x = np.bitwise_or.reduce(vals ^ vals[0])
            if bin(int(x)).count("1") <= 3:
                return "bit flags"
        return "analog"

    def analyse(self):
        blocks, fields, groups = [], [], []
        total = 0
        for (hdr, req), dq in self.samples.items():
            n = len(dq)
            total += n
            L = min(len(s[1]) for s in dq) if n else 0
            if n == 0 or L == 0:
                continue
            M = np.array([list(s[1][:L]) for s in dq], dtype=np.int64)       # n x L
            last = M[-1]
            bstats = []
            for i in range(L):
                col = M[:, i]
                bstats.append({"i": i, "hex": f"{int(last[i]):02X}", "std": round(float(col.std()), 2),
                               "min": int(col.min()), "max": int(col.max()), "distinct": int(len(np.unique(col))),
                               "class": self.classify(col) if n >= 3 else "?"})
            blocks.append({"hdr": hdr, "req": req, "len": L, "n": n, "bytes": bstats})
            if n < MIN_N:
                continue

            # candidate fields
            cands, names = [], []
            for i in range(L):
                u8 = M[:, i]
                if u8.std() == 0:
                    continue
                cands.append(u8); names.append((i, "u8"))
                cands.append(np.where(u8 > 127, u8 - 256, u8)); names.append((i, "s8"))
                if i + 1 < L:
                    be = M[:, i] * 256 + M[:, i + 1]
                    le = M[:, i + 1] * 256 + M[:, i]
                    cands.append(be); names.append((i, "u16be"))
                    cands.append(np.where(be > 32767, be - 65536, be)); names.append((i, "s16be"))
                    cands.append(le); names.append((i, "u16le"))
            if not cands:
                continue
            X = np.array(cands, dtype=float).T                                # n x F
            for ref in REFS:
                y = np.array([s[2].get(ref, np.nan) for s in dq], dtype=float)
                ok = ~np.isnan(y)
                if ok.sum() < MIN_N or np.nanstd(y) < 1e-6:
                    continue
                lo, hi = np.nanpercentile(y, [10, 90])        # robust: one stray sample can't fake a range
                if hi - lo < MIN_SPAN.get(ref, 0):
                    continue
                Xo, yo = X[ok], y[ok]
                xs = Xo.std(axis=0)
                good = xs > 0
                if not good.any():
                    continue
                xz = (Xo[:, good] - Xo[:, good].mean(axis=0)) / xs[good]
                yz = (yo - yo.mean()) / yo.std()
                r = (xz * yz[:, None]).mean(axis=0)
                idx = np.where(good)[0]
                # four neighbouring single bytes that all track the same value = one per cylinder (4-cylinder engine)
                rmap = {names[idx[k]]: r[k] for k in range(len(idx))}
                for off in range(L - 3):
                    for typ in ("s8", "u8"):
                        rs = [rmap.get((off + k, typ), 0) for k in range(4)]
                        if all(abs(v) >= 0.8 for v in rs):
                            fits = []
                            for k in range(4):
                                col = Xo[:, names.index((off + k, typ))]
                                a4, c4 = np.polyfit(col, yo, 1)
                                for ht, ha, hb in HYUNDAI.get(ref, []):
                                    if ht == typ and 0.8 <= a4 / ha <= 1.25:
                                        a4, c4 = ha, hb
                                raw_now = int(X[-1, names.index((off + k, typ))])
                                fits.append({"raw_now": raw_now, "scale": float(a4), "bias": float(c4), "value_now": round(float(a4 * raw_now + c4), 2), "r": round(float(rs[k]), 3)})
                            groups.append({"hdr": hdr, "req": req, "offset": off, "type": typ + "x4", "ref": ref, "per_cylinder": True,
                                           "confidence": "likely", "r": round(float(np.mean(np.abs(rs))), 3), "cylinders": fits, "n": int(ok.sum()),
                                           "ref_now": round(float(yo[-1]), 3)})
                best = np.argsort(-np.abs(r))[:3]
                best = np.argsort(-np.abs(r))[:8]
                for b in best:
                    if abs(r[b]) < LIKELY_R:
                        continue
                    j = idx[b]
                    a, c = np.polyfit(Xo[:, j], yo, 1)
                    off, typ = names[j]
                    strong = abs(r[b]) >= (MIN_R if ok.sum() >= 25 else MIN_R_SMALL)
                    snapped = None
                    for ht, ha, hb in HYUNDAI.get(ref, []):
                        if ht == typ and ha and 0.8 <= a / ha <= 1.25:
                            snapped = (ha, hb)
                    if snapped:
                        a, c = snapped
                    fields.append({"hdr": hdr, "req": req, "offset": off, "type": typ, "ref": ref,
                                   "confidence": "cracked" if strong else "likely", "hyundai_formula": bool(snapped),
                                   "r": round(float(r[b]), 4), "scale": float(a), "bias": float(c), "n": int(ok.sum()),
                                   "raw_now": int(X[-1, j]), "value_now": round(float(a * X[-1, j] + c), 3),
                                   "ref_now": round(float(yo[-1]), 3)})
        # keep the best candidate per (block, offset) and per (block, ref)
        fields.sort(key=lambda f: -(abs(f["r"]) + (0.02 if f["hyundai_formula"] else 0)))
        seen_off, seen_ref, keep = set(), set(), []
        for f in fields:
            k1 = (f["hdr"], f["req"], f["offset"])
            k2 = (f["hdr"], f["req"], f["ref"])
            if k1 in seen_off or k2 in seen_ref:
                continue
            seen_off.add(k1); seen_ref.add(k2)
            keep.append(f)
        # the best match per reference is "the" hidden copy of it; other matches only move with it
        named = set()
        for f in keep:
            f["primary"] = f["ref"] not in named and f["r"] > 0
            named.add(f["ref"])
        # one per-cylinder group per block/ref, skip duplicates that overlap
        seen_g, gkeep = set(), []
        for g in sorted(groups, key=lambda g: -g["r"]):
            k = (g["hdr"], g["req"], g["ref"])
            if k in seen_g:
                continue
            seen_g.add(k)
            gkeep.append(g)
        self.result = {"blocks": blocks, "fields": keep, "groups": gkeep, "updated": time.time(), "samples": total}
        return self.result
