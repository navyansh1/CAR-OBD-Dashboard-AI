"""
Trip history: each finished trip is saved as logs/trips/<id>.json with its summary,
a 1-second time series (for charts) and computed highlights.

    .venv/bin/python trips.py backfill START END     # rebuild a trip from the CSV logs (unix times)
"""
import csv, glob, json, os, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
def tdir():
    import cars
    d = os.path.join(cars.car_dir(), "trips")
    os.makedirs(d, exist_ok=True)
    return d
SERIES = ["SPEED", "RPM", "ENGINE_LOAD", "THROTTLE_POS", "INTAKE_PRESSURE", "COOLANT_TEMP", "TIMING_ADVANCE", "LPH"]
SPEED_BANDS = [(0, 1, "Stopped"), (1, 20, "1–20 km/h"), (20, 40, "20–40"), (40, 60, "40–60"), (60, 80, "60–80"), (80, 999, "80+")]
RPM_BANDS = [(0, 1000, "Idle (<1000)"), (1000, 2000, "1000–2000"), (2000, 3000, "2000–3000"), (3000, 4000, "3000–4000"), (4000, 99999, "4000+")]


def lph(rpm, mapk, iat, lam, disp=1.591e-3, ve=0.85):
    if None in (rpm, mapk, iat) or rpm <= 300:
        return 0.0
    air = (mapk * 1000) * (disp * rpm / 120.0) * ve / (287.05 * (iat + 273.15)) * 1000.0
    return air / (14.7 * (lam or 1.0)) * 3600 / 745.0


def highlights(series):
    t, spd, rpm = series["t"], series["SPEED"], series["RPM"]
    h = {}
    if not t:
        return h
    i = max(range(len(spd)), key=lambda k: spd[k] or 0)
    h["top_speed"] = {"value": spd[i], "at": t[i]}
    i = max(range(len(rpm)), key=lambda k: rpm[k] or 0)
    h["max_rpm"] = {"value": rpm[i], "at": t[i]}
    # hardest acceleration: biggest speed gain over ~3 seconds
    best, at = 0.0, None
    for k in range(len(t)):
        j = k
        while j + 1 < len(t) and t[j + 1] - t[k] <= 3.0:
            j += 1
        if j > k and t[j] > t[k] and spd[j] is not None and spd[k] is not None:
            a = (spd[j] - spd[k]) / (t[j] - t[k])
            if a > best:
                best, at = a, t[k]
    h["hardest_accel"] = {"value": round(best, 2), "unit": "km/h per second", "at": at}
    # hardest braking
    best, at = 0.0, None
    for k in range(len(t)):
        j = k
        while j + 1 < len(t) and t[j + 1] - t[k] <= 3.0:
            j += 1
        if j > k and t[j] > t[k] and spd[j] is not None and spd[k] is not None:
            a = (spd[k] - spd[j]) / (t[j] - t[k])
            if a > best:
                best, at = a, t[k]
    h["hardest_brake"] = {"value": round(best, 2), "unit": "km/h per second", "at": at}
    # longest stop with the engine running
    run, start, longest, lstart = 0.0, None, 0.0, None
    for k in range(1, len(t)):
        if (spd[k] or 0) < 1 and (rpm[k] or 0) > 300:
            if start is None:
                start = t[k - 1]
            if t[k] - start > longest:
                longest, lstart = t[k] - start, start
        else:
            start = None
    h["longest_stop"] = {"value": round(longest), "at": lstart}
    # time spent per band
    def bands(vals, edges):
        out = [0.0] * len(edges)
        for k in range(1, len(t)):
            v, dt = vals[k - 1], min(t[k] - t[k - 1], 5)
            if v is None:
                continue
            for n, (lo, hi, _) in enumerate(edges):
                if lo <= v < hi:
                    out[n] += dt
                    break
        return [{"label": e[2], "seconds": round(s)} for e, s in zip(edges, out)]
    h["speed_bands"] = bands(spd, SPEED_BANDS)
    h["rpm_bands"] = bands([r if (r or 0) > 300 else None for r in rpm], RPM_BANDS)
    # stop-start count
    stops, moving = 0, False
    for v in spd:
        if (v or 0) >= 5:
            moving = True
        elif moving and (v or 0) < 1:
            stops, moving = stops + 1, False
    h["stops"] = stops
    return h


def save(summary, series, started, ended):
    DIR = tdir()
    tid = time.strftime("%Y%m%d-%H%M%S", time.localtime(started))
    doc = {"id": tid, "started": started, "ended": ended, "summary": summary, "series": series, "highlights": highlights(series)}
    with open(os.path.join(DIR, tid + ".json"), "w") as f:
        json.dump(doc, f)
    return tid


def index():
    out = []
    for p in sorted(glob.glob(os.path.join(tdir(), "*.json")), reverse=True):
        try:
            with open(p) as f:
                d = json.load(f)
            out.append({"id": d["id"], "started": d["started"], "ended": d["ended"], "summary": d["summary"],
                        "highlights": {k: v for k, v in d.get("highlights", {}).items() if k in ("top_speed", "max_rpm", "hardest_accel", "stops")}})
        except Exception:
            continue
    return out


def load(tid):
    p = os.path.join(tdir(), os.path.basename(tid) + ".json")
    with open(p) as f:
        return json.load(f)


def backfill(start, end, summary=None):
    """Rebuild a trip's 1-second series from the dashboard's CSV logs."""
    rows = []
    for p in glob.glob(os.path.join(HERE, "logs", "drive-*.csv")):
        with open(p) as f:
            r = csv.reader(f)
            next(r, None)
            for ts, name, val, unit in r:
                t = float(ts)
                if start <= t <= end:
                    rows.append((t, name, float(val)))
    rows.sort()
    cur = {}
    series = {k: [] for k in ["t"] + SERIES}
    nxt = start
    for t, name, v in rows:
        cur[name] = v
        if t >= nxt:
            series["t"].append(round(t, 1))
            for k in SERIES:
                if k == "LPH":
                    series[k].append(round(lph(cur.get("RPM"), cur.get("INTAKE_PRESSURE"), cur.get("INTAKE_TEMP"), cur.get("COMMANDED_EQUIV_RATIO")), 2))
                else:
                    series[k].append(cur.get(k))
            nxt = t + 1.0
    return save(summary or {}, series, start, end)


if __name__ == "__main__":
    if len(sys.argv) >= 4 and sys.argv[1] == "backfill":
        s, e = float(sys.argv[2]), float(sys.argv[3])
        summary = None
        if len(sys.argv) > 4:
            with open(sys.argv[4]) as f:
                summary = json.load(f)
        print("saved", backfill(s, e, summary))
