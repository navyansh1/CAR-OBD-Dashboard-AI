"""
Read-only manufacturer module scan for Hyundai / Kia cars over an ELM327 (11-bit CAN).

Beyond standard OBD-II, Hyundai modules answer manufacturer requests on their own
CAN addresses (request ID, reply ID = request + 8):
  UDS 0x22  ReadDataByIdentifier      e.g. 22 B002 on the cluster = odometer, fuel litres
  KWP 0x21  ReadDataByLocalIdentifier e.g. 21 01 on older engine ECUs
  KWP 0x1A  ReadECUIdentification
  UDS 0x19  ReadDTCInformation / KWP 0x18 ReadDTCByStatus  (module trouble codes)
  0x3E      TesterPresent (used only to see whether a module exists)

Only the services above are ever sent. Nothing that writes, resets, clears codes,
changes diagnostic session, unlocks security or runs routines is in this file, and
send() refuses anything else.
"""
import json, os, re, time

HERE = os.path.dirname(os.path.abspath(__file__))
ALLOWED_SERVICES = {0x22, 0x21, 0x1A, 0x19, 0x18, 0x3E}

# Common Hyundai/Kia diagnostic request addresses. Names are the usual assignment;
# a given car may use an address for something else or not have the module at all.
MODULES = [
    ("7E0", "Engine control (ECM)"),
    ("7E1", "Transmission (TCU)"),
    ("7E2", "Hybrid / vehicle control (VCU)"),
    ("7E4", "Battery management (BMS)"),
    ("7D1", "ABS / ESC"),
    ("7D2", "Airbag (SRS)"),
    ("7D4", "Power steering (MDPS) / airbag"),
    ("7C6", "Instrument cluster"),
    ("7A0", "Body control / TPMS (BCM)"),
    ("770", "Integrated gateway & power module (IGPM)"),
    ("7A5", "Smart key (SMK)"),
    ("7A6", "TPMS receiver"),
    ("7B3", "Climate control (FATC)"),
    ("7C4", "Front camera / lane assist"),
    ("7D0", "Smart cruise radar (SCC)"),
    ("7C7", "Head unit (AVN)"),
    ("7D6", "Parking assist (SPAS)"),
    ("7B7", "Power seat / tailgate"),
]

UDS_ID = {
    "F187": "Spare part number", "F188": "ECU software number", "F189": "Software version",
    "F18A": "Supplier ID", "F18B": "Manufacture date", "F18C": "Serial number", "F190": "VIN",
    "F191": "Hardware number", "F192": "Supplier hardware number", "F193": "Supplier hardware version",
    "F194": "Supplier software number", "F195": "Supplier software version", "F197": "System name",
    "F19E": "Diagnostic data file", "F1A0": "Hyundai ECU variant", "F100": "Hyundai ECU information",
}
KWP_ID = {
    "87": "Part number", "88": "Original part number", "89": "Hardware version", "8A": "Supplier",
    "8B": "Manufacture date", "8C": "Serial number", "90": "VIN", "91": "Hardware number",
    "92": "Supplier hardware number", "94": "Software number", "95": "Software version",
    "97": "System name", "9B": "Calibration / date",
}
# Chassis / safety modules: identification and trouble-code reads only. Probing unknown data
# identifiers on these has been reported to store ESC (C1555) and lane-assist faults on Kia cars.
SAFETY = {"7D1", "7D2", "7D4", "7D0", "7C4"}
# Narrow, published Hyundai/Kia identifiers per module (never a wildcard sweep).
KNOWN_EXTRA = {
    "7C6": ["22B001", "22B002", "22B003", "22B004", "22B005", "22B00C"],
    "7A0": ["22C00B", "22B001"],
    "770": ["22BC01", "22BC02", "22BC03", "22BC04", "22BC05", "22BC06", "22BC07", "22BC08", "22BC09", "22BC0A"],
    "7E0": ["22E001", "22E002", "22E003", "22E004", "22E00A"],
}
NRC = {0x10: "general reject", 0x11: "service not supported", 0x12: "sub-function not supported",
       0x13: "bad length", 0x14: "response too long", 0x21: "busy", 0x22: "conditions not correct",
       0x31: "request out of range", 0x33: "security access denied", 0x78: "response pending",
       0x7E: "not supported in this session", 0x7F: "service not supported in this session"}


# Per-make setup: which module addresses to try, which published definitions to use.
SUZUKI_MODULES = [("7E0", "Engine control (ECM)")]
MAKES = {"Hyundai": {"modules": MODULES, "defs": "hyundai_pids.json"},
         "Suzuki": {"modules": SUZUKI_MODULES, "defs": "suzuki_pids.json"}}


def load_defs(fname="hyundai_pids.json"):
    try:
        with open(os.path.join(HERE, fname)) as f:
            return json.load(f)["pids"]
    except Exception:
        return []


def bits(payload, bix, length):
    """Big-endian bit field: bit 0 is the most significant bit of the first payload byte."""
    if (bix + length + 7) // 8 > len(payload):
        return None
    v = int.from_bytes(bytes(payload), "big")
    total = len(payload) * 8
    return (v >> (total - bix - length)) & ((1 << length) - 1)


# ---------- Torque-style equations (A = first byte after the response echo) ----------
def col(letters):
    n = 0
    for ch in letters:
        n = n * 26 + (ord(ch) - 64)
    return n - 1


def _int16(a, b):
    v = (a << 8) | b
    return v - 0x10000 if v & 0x8000 else v


def _int24(a, b, c):
    return (a << 16) | (b << 8) | c


def _signed(x):
    return x - 256 if x > 127 else x


def _bit(x, n):
    return (x >> int(n)) & 1


SAFE = re.compile(r"^[A-Z0-9 .+\-*/()<>&|:,]+$")
FUNCS = {"INT16": _int16, "INT24": _int24, "SIGNED": _signed, "BIT": _bit}


def compile_eq(eq):
    if not eq or not SAFE.match(eq):
        return None
    s = re.sub(r"\b([A-Z]{1,2})\b", lambda m: f"_v[{col(m.group(1))}]", eq)
    s = s.replace(":", ",").replace("<", "<<").replace(">", ">>")
    try:
        code = compile(s, "<eq>", "eval")
    except SyntaxError:
        return None
    for name in code.co_names:
        if name not in FUNCS and name != "_v":
            return None
    return code


def evaluate(code, payload):
    try:
        return float(eval(code, {"__builtins__": {}, **FUNCS}, {"_v": payload}))
    except Exception:
        return None


def text(b):
    s = "".join(chr(x) if 32 <= x < 127 else " " for x in b)
    return re.sub(r"\s+", " ", s).strip()


def decode_uds_dtcs(d):
    """59 02 <mask> [DTC_H DTC_M FTB STATUS]..."""
    out = []
    body = d[3:]
    for i in range(0, len(body) - 3, 4):
        h, m, ftb, st = body[i:i + 4]
        code = "PCBU"[h >> 6] + str((h >> 4) & 3) + f"{h & 0xF:X}{m:02X}"
        flags = [n for bit, n in [(0, "failing now"), (2, "pending"), (3, "confirmed"), (7, "warning lamp")] if st & (1 << bit)]
        out.append({"code": f"{code}-{ftb:02X}", "status": f"{st:02X}", "flags": flags})
    return out


def decode_kwp_dtcs(d):
    """58 <count> [DTC_H DTC_L STATUS]..."""
    out = []
    body = d[2:]
    for i in range(0, len(body) - 2, 3):
        h, l, st = body[i:i + 3]
        if h == 0 and l == 0:
            continue
        code = "PCBU"[h >> 6] + str((h >> 4) & 3) + f"{h & 0xF:X}{l:02X}"
        out.append({"code": code, "status": f"{st:02X}", "flags": ["stored"] if st & 0x20 else []})
    return out


class Scanner:
    """`link.at(b"ATSH7E0")` sends an AT command; `link.raw_msgs("22F190")` returns python-obd Messages."""

    def __init__(self, link, log=lambda *_: None, make="Hyundai"):
        self.link = link
        self.log = log
        cfg = MAKES.get(make, MAKES["Hyundai"])
        self.make = make
        self.modules = cfg["modules"]
        self.defs = load_defs(cfg["defs"])
        for d in self.defs:
            d["_code"] = compile_eq(d.get("eq"))
        self.cur = None

    def select(self, hdr):
        if self.cur == hdr:
            return
        resp = f"{int(hdr, 16) + 8:03X}"
        self.link.at(f"ATSH{hdr}".encode())
        self.link.at(f"ATCRA{resp}".encode())
        self.cur = hdr

    def restore(self):
        self.link.at(b"ATCRA")
        self.link.at(b"ATSH7DF")
        self.cur = None

    def send(self, req):
        svc = int(req[:2], 16)
        if svc not in ALLOWED_SERVICES:
            raise ValueError(f"refusing to send service {svc:02X}: not a read-only service")
        msgs = self.link.raw_msgs(req)
        for m in msgs or []:
            d = bytes(m.data)
            if not d:
                continue
            if d[0] == 0x7F and len(d) >= 3 and d[1] == svc:
                if d[2] == 0x78:          # response pending; ELM usually collects the final answer itself
                    continue
                return {"ok": False, "nrc": d[2], "why": NRC.get(d[2], f"NRC {d[2]:02X}"), "hex": d.hex(" ").upper()}
            if d[0] == svc + 0x40:
                return {"ok": True, "data": d, "hex": d.hex(" ").upper()}
        return None

    def probe(self, hdr):
        self.select(hdr)
        for req in ("3E00", "22F190", "1A90", "2101"):
            r = self.send(req)
            if r is not None:
                return True
        return False

    def scan_module(self, hdr, name):
        mod = {"address": hdr, "reply": f"{int(hdr, 16) + 8:03X}", "name": name, "present": False,
               "ident": [], "reads": [], "dtcs": [], "dtc_method": None, "scanned": time.time()}
        if not self.probe(hdr):
            return mod
        mod["present"] = True
        self.log(f"module {hdr} ({name}) answers")

        uds_ok = True
        for did, label in UDS_ID.items():
            r = self.send("22" + did)
            if r and not r["ok"] and r["nrc"] in (0x11, 0x7F):
                uds_ok = False
                break
            if r and r["ok"] and len(r["data"]) > 3:
                raw = r["data"][3:]
                val = text(raw)
                if did == "F18B" and len(raw) >= 3 and all((b >> 4) < 10 and (b & 15) < 10 for b in raw[:4]):
                    h = raw[:4].hex()
                    val = f"{h[:4]}-{h[4:6]}-{h[6:8]}" if len(h) >= 8 else h
                mod["ident"].append({"req": "22 " + did, "label": label, "value": val or r["hex"], "hex": r["hex"]})
        kwp_ok = True
        for lid, label in KWP_ID.items():
            r = self.send("1A" + lid)
            if r and not r["ok"] and r["nrc"] in (0x11, 0x7F):
                kwp_ok = False
                break
            body = r["data"][2:] if r and r["ok"] else b""
            if body and any(b not in (0x00, 0xFF) for b in body):
                val = text(body)
                if val and not any(i["value"] == val for i in mod["ident"]):
                    mod["ident"].append({"req": "1A " + lid, "label": label, "value": val, "hex": r["hex"]})

        reqs = []
        if hdr not in SAFETY:
            known = sorted({d["request"] for d in self.defs if d["header"] == hdr})
            reqs += [r for r in known if r.startswith("21")]               # KWP reads: always worth one try
            if uds_ok:
                reqs += [r for r in known if r.startswith("22")]
                reqs += KNOWN_EXTRA.get(hdr, []) if self.make == "Hyundai" else []
            reqs += [f"21{i:02X}" for i in range(1, 0x10)]
        seen = set()
        for req in reqs:
            if req in seen:
                continue
            seen.add(req)
            r = self.send(req)
            if r is None:
                continue
            if not r["ok"]:
                continue
            if len(r["data"]) > len(req) // 2:          # ignore empty positive replies
                mod["reads"].append(self.decode_read(hdr, req, r["data"]))

        r = self.send("1902FF")
        if r and r["ok"]:
            mod["dtcs"], mod["dtc_method"] = decode_uds_dtcs(r["data"]), "UDS 19 02"
        else:
            r2 = self.send("1802FF00")
            if r2 and r2["ok"]:
                mod["dtcs"], mod["dtc_method"] = decode_kwp_dtcs(r2["data"]), "KWP 18 02"
            elif r or r2:
                mod["dtc_method"] = "not readable without a dealer session"
        return mod

    def decode_read(self, hdr, req, d):
        echo = len(req) // 2
        payload = list(d[echo:])
        vals = []
        for df in self.defs:
            if df["header"] != hdr or df["request"] != req:
                continue
            if "bix" in df:                                  # bit-field definition (OBDb style)
                raw = bits(payload, df["bix"], df["len"])
                if raw is None or (df.get("nullmax") is not None and raw >= df["nullmax"]):
                    continue
                v = raw * df.get("mul", 1) / df.get("div", 1) + df.get("add", 0)
                vals.append({"name": df["name"], "label": df.get("label"), "value": round(v, 4), "unit": df.get("unit") or "",
                             "kind": df.get("kind", "value"), "verified": df.get("verified", True)})
            elif df.get("_code") is not None:
                v = evaluate(df["_code"], payload)
                if v is not None:
                    vals.append({"name": df["name"], "value": round(v, 4), "unit": df.get("unit") or ""})
        return {"req": f"{req[:2]} {req[2:]}", "request": req, "len": len(payload),
                "hex": " ".join(f"{b:02X}" for b in payload), "text": text(bytes(payload)), "values": vals}

    def scan_all(self, progress=lambda *_: None):
        mods = []
        try:
            for i, (hdr, name) in enumerate(self.modules):
                progress(i, len(self.modules), hdr, name)
                mods.append(self.scan_module(hdr, name))
        finally:
            self.restore()
        return mods

    def read_live(self, targets):
        """targets: list of (hdr, req). Returns {(hdr, req): read dict}."""
        out = {}
        try:
            for hdr, req in sorted(targets):
                self.select(hdr)
                r = self.send(req)
                if r and r["ok"] and len(r["data"]) > len(req) // 2:
                    out[(hdr, req)] = self.decode_read(hdr, req, r["data"])
        finally:
            self.restore()
        return out
