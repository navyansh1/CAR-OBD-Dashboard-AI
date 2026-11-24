"""Car profiles: cars/<id>.json, the active one named in cars/active.txt. Each car keeps its data in logs/<id>/."""
import json, os, glob

HERE = os.path.dirname(os.path.abspath(__file__))
CARS = os.path.join(HERE, "cars")
LOGS = os.path.join(HERE, "logs")


def ids():
    return sorted(os.path.splitext(os.path.basename(p))[0] for p in glob.glob(os.path.join(CARS, "*.json")))


def load(cid):
    with open(os.path.join(CARS, cid + ".json")) as f:
        return json.load(f)


def active_id():
    try:
        with open(os.path.join(CARS, "active.txt")) as f:
            cid = f.read().strip()
        if cid in ids():
            return cid
    except Exception:
        pass
    return (ids() or ["default"])[0]


def active():
    try:
        return load(active_id())
    except Exception:
        return {}


def set_active(cid):
    if cid not in ids():
        raise ValueError(f"no car profile {cid!r}")
    with open(os.path.join(CARS, "active.txt"), "w") as f:
        f.write(cid)


def save(profile):
    with open(os.path.join(CARS, profile["id"] + ".json"), "w") as f:
        json.dump(profile, f, indent=1, ensure_ascii=False)


def car_dir(cid=None):
    root = os.environ.get("OBD_DATA_ROOT") or LOGS        # replay mode points this at a copy of sample_data/
    d = os.path.join(root, cid or active_id())
    os.makedirs(d, exist_ok=True)
    return d


def match(vin=None, cal_ids=()):
    """Which profile does this car look like? VIN first, then calibration-number prefix."""
    for cid in ids():
        m = load(cid).get("match", {})
        if vin and m.get("vin") == vin:
            return cid
    for cid in ids():
        m = load(cid).get("match", {})
        if m.get("cal_prefix") and any((c or "").startswith(m["cal_prefix"]) for c in cal_ids or ()):
            return cid
    return None


def listing():
    a = active_id()
    out = []
    for cid in ids():
        p = load(cid)
        out.append({"id": cid, "name": p.get("name"), "short": p.get("short"), "variant": p.get("variant"),
                    "year": p.get("year"), "make": p.get("make"), "logo": p.get("logo"), "active": cid == a})
    return out
