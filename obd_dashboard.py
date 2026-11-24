#!/usr/bin/env python3
"""
OBD Live: reads everything a standard ELM327 can get from the car and serves it
as a live dashboard on http://127.0.0.1:8327

  .venv/bin/python obd_dashboard.py                 # via OBD Bridge.app (recommended on macOS)
  .venv/bin/python obd_dashboard.py --port /dev/cu.OBDII
  .venv/bin/python obd_dashboard.py --demo          # fake data, no car needed

What it collects
  * Mode 01  every supported live PID, polled continuously (6 PIDs per request on CAN)
  * Mode 01  monitor status / readiness (since clear and this drive cycle)
  * Mode 02  freeze frame (only when a code is stored)
  * Mode 03  stored trouble codes, Mode 07 pending codes, Mode 0A permanent codes
  * Mode 06  on-board monitor test results (O2, catalyst, VVT, misfire per cylinder, ...)
  * Mode 09  VIN, calibration IDs, CVN, ECU name, any other supported info PID
  * Adapter  ELM version, battery voltage at the OBD port, protocol
  * Raw      every request/response line on the wire
  * Bus      optional passive CAN monitor (ATMA) for a few seconds on demand

Never sent: Mode 04 (clear codes / reset monitors) or anything else that writes to the car.
"""
import argparse, csv, json, logging, math, os, queue, random, re, socket, threading, time, traceback
from collections import deque
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs

HERE = os.path.dirname(os.path.abspath(__file__))
import cars
LOGDIR = os.path.join(HERE, "logs")
BRIDGE_STATUS = os.path.join(LOGDIR, "bridge_status.json")
os.makedirs(LOGDIR, exist_ok=True)

# Polled every cycle; everything else live is polled every SLOW_EVERY cycles.
FAST = {
    "RPM", "SPEED", "ENGINE_LOAD", "ABSOLUTE_LOAD", "THROTTLE_POS", "THROTTLE_POS_B", "THROTTLE_POS_C",
    "RELATIVE_THROTTLE_POS", "ACCELERATOR_POS_D", "ACCELERATOR_POS_E", "ACCELERATOR_POS_F",
    "THROTTLE_ACTUATOR", "INTAKE_PRESSURE", "MAF", "TIMING_ADVANCE", "SHORT_FUEL_TRIM_1",
    "SHORT_FUEL_TRIM_2", "COMMANDED_EQUIV_RATIO", "FUEL_RATE", "FUEL_INJECT_TIMING",
    "O2_B1S1", "O2_B1S2", "O2_B1S3", "O2_B1S4", "O2_B2S1", "O2_B2S2", "O2_B2S3", "O2_B2S4",
    "O2_S1_WR_VOLTAGE", "O2_S1_WR_CURRENT", "RELATIVE_ACCEL_POS", "DEMAND_ENGINE_LOAD",
    "ACTUAL_ENGINE_LOAD", "ENGINE_TORQUE", "FUEL_RAIL_PRESSURE_DIRECT", "FUEL_RAIL_PRESSURE_VAC",
}
SLOW_EVERY = 5
# Reference values kept during "learn focus" mode; everything else pauses so engine blocks are sampled faster.
FOCUS_REFS = {"RPM", "SPEED", "ENGINE_LOAD", "ABSOLUTE_LOAD", "THROTTLE_POS", "RELATIVE_THROTTLE_POS",
              "ACCELERATOR_POS_D", "THROTTLE_ACTUATOR", "INTAKE_PRESSURE", "TIMING_ADVANCE", "SHORT_FUEL_TRIM_1",
              "O2_B1S1", "COMMANDED_EQUIV_RATIO", "COOLANT_TEMP", "INTAKE_TEMP", "CONTROL_MODULE_VOLTAGE",
              "CATALYST_TEMP_B1S1", "LONG_FUEL_TRIM_1"}
NEVER = {"CLEAR_DTC"}
STRING_M1 = {"STATUS", "STATUS_DRIVE_CYCLE", "FUEL_STATUS", "O2_SENSORS", "O2_SENSORS_ALT",
             "OBD_COMPLIANCE", "FUEL_TYPE", "FREEZE_DTC", "AUX_INPUT_STATUS"}

UNIT = {
    "revolutions_per_minute": "rpm", "kilometer_per_hour": "km/h", "degree_Celsius": "°C",
    "percent": "%", "kilopascal": "kPa", "pascal": "Pa", "volt": "V", "millivolt": "mV",
    "second": "s", "minute": "min", "kilometer": "km", "degree": "°", "count": "",
    "ratio": "λ", "dimensionless": "", "millisecond": "ms", "ohm": "Ω", "gps": "g/s",
    "gram_per_second": "g/s", "liters_per_hour": "L/h", "milliampere": "mA", "newton_meter": "N·m",
    "kilogram_per_hour": "kg/h",
}

GROUPS = [
    ("Engine", r"^(RPM|ENGINE_LOAD|ABSOLUTE_LOAD|TIMING_ADVANCE|RUN_TIME|ENGINE_TORQUE|.*ENGINE_LOAD|ENGINE_REF_TORQUE)$"),
    ("Speed & distance", r"^(SPEED|DISTANCE.*|WARMUPS.*|TIME_.*|RUN_TIME_MIL)$"),
    ("Throttle & pedal", r"^(THROTTLE.*|RELATIVE_THROTTLE_POS|ACCELERATOR.*|RELATIVE_ACCEL_POS)$"),
    ("Air & fuel", r"^(INTAKE_PRESSURE|MAF|.*FUEL_TRIM.*|COMMANDED_EQUIV_RATIO|FUEL_.*|EVAPORATIVE.*|EVAP.*|BAROMETRIC_PRESSURE|ETHANOL.*|COMMANDED_EGR|EGR_ERROR)$"),
    ("Oxygen sensors", r"^(O2_.*|LONG_O2_TRIM.*|SHORT_O2_TRIM.*)$"),
    ("Temperatures", r"^(.*TEMP.*)$"),
    ("Electrical", r"^(CONTROL_MODULE_VOLTAGE|ELM_VOLTAGE|HYBRID.*)$"),
]


def group_of(name):
    for g, rx in GROUPS:
        if re.match(rx, name):
            return g
    return "Other"


def unit_label(u):
    s = str(u)
    return UNIT.get(s, s.replace("_", " "))


def num(v):
    """pint Quantity -> (float, unit) or (None, None)."""
    if v is None or not hasattr(v, "magnitude"):
        return None, None
    m = v.magnitude
    try:
        return float(m), unit_label(v.units)
    except (TypeError, ValueError):
        return None, None


def text(b):
    return "".join(chr(x) for x in b if 32 <= x < 127).strip()


# ----------------------------------------------------------------------------------------
class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.version = 0
        self.conn = {"status": "starting", "source": None, "protocol": None, "elm_version": None,
                     "elm_device": None, "voltage": None, "since": None, "error": None,
                     "pids_per_sec": 0.0, "cycle_ms": None, "queries": 0, "failures": 0,
                     "multi_pid": False, "csv": None, "bridge": None, "demo": False}
        self.vehicle = {"profile": car_profile()}
        self.live = {}           # name -> record
        self.history = {}        # name -> deque[(t, v)]
        self.info = {}           # string-ish mode 01 values
        self.status = None       # readiness since clear
        self.status_cycle = None # readiness this drive cycle
        self.dtcs = {"stored": [], "pending": [], "permanent": [], "updated": None}
        self.freeze = []
        self.mode06 = []
        self.mode06_updated = None
        self.supported = []
        self.unknown = {}        # raw hex for supported PIDs python-obd cannot decode
        self.raw = deque(maxlen=800)
        self.raw_seq = 0
        self.sniff = {"running": False, "frames": [], "lines": 0, "started": None, "seconds": 0, "note": None}
        self.trip = Trip()
        self.trip.load_last()
        self.modules = {"running": False, "progress": None, "list": [], "updated": None}
        self.re = {"blocks": [], "fields": [], "updated": None, "samples": 0}
        self.watch = {"running": False, "until": None, "blocks": {}, "events": [], "label": None, "label_t": None}

    def bump(self):
        self.version += 1

    def reset_for_car(self):
        """Caller holds the lock. Clear everything that belongs to one car."""
        self.vehicle = {"profile": car_profile()}
        self.live, self.history, self.info, self.unknown = {}, {}, {}, {}
        self.status = self.status_cycle = None
        self.dtcs = {"stored": [], "pending": [], "permanent": [], "updated": None}
        self.freeze, self.mode06, self.mode06_updated, self.supported = [], [], None, []
        self.modules = {"running": False, "progress": None, "list": [], "updated": None}
        self.re = {"blocks": [], "fields": [], "updated": None, "samples": 0}
        self.watch = {"running": False, "until": None, "blocks": {}, "events": [], "label": None, "label_t": None}
        self.trip = Trip()
        self.trip.load_last()
        self.bump()

    def set_live(self, name, desc, value, unit, mode=1, pid=None, group=None, tag=None):
        now = time.time()
        r = self.live.get(name)
        if r is None:
            r = self.live[name] = {"name": name, "desc": desc, "unit": unit, "mode": mode, "pid": pid,
                                   "group": group or group_of(name), "min": value, "max": value, "tag": tag}
            self.history[name] = deque(maxlen=900)
        r.update(value=value, t=now, unit=unit)
        if value is not None:
            r["min"] = value if r["min"] is None else min(r["min"], value)
            r["max"] = value if r["max"] is None else max(r["max"], value)
            self.history[name].append((round(now, 2), value))

    def add_raw(self, tx, rx, ms):
        self.raw_seq += 1
        self.raw.append({"n": self.raw_seq, "t": time.time(), "tx": tx, "rx": rx, "ms": ms})

    def snapshot(self, history=False):
        with self.lock:
            d = {
                "version": self.version, "now": time.time(), "conn": dict(self.conn),
                "vehicle": self.vehicle, "live": list(self.live.values()), "info": self.info,
                "status": self.status, "status_cycle": self.status_cycle, "dtcs": self.dtcs,
                "freeze": self.freeze, "mode06": self.mode06, "mode06_updated": self.mode06_updated,
                "supported": self.supported, "unknown": self.unknown, "sniff": self.sniff, "modules": self.modules, "re": self.re, "watch": self.watch,
                "derived": derived(self.live), "trip": self.trip.snapshot(),
            }
            if history:
                d["history"] = {k: list(v) for k, v in self.history.items()}
            return json.dumps(d, default=str)


def derived(live):
    g = lambda n: (live.get(n) or {}).get("value")
    out = []
    ml = g("O2_S1_LAMBDA")
    if ml is not None:
        out.append({"name": "AFR_MEASURED", "desc": "Measured air-fuel ratio (wideband sensor, λ × 14.7)", "value": round(ml * 14.7, 2), "unit": ":1"})
    lam = g("COMMANDED_EQUIV_RATIO")
    if lam is not None:
        out.append({"name": "AFR_COMMANDED", "desc": "Commanded air-fuel ratio (petrol, λ × 14.7)", "value": round(lam * 14.7, 2), "unit": ":1"})
    mapk, baro = g("INTAKE_PRESSURE"), g("BAROMETRIC_PRESSURE")
    if mapk is not None and baro is not None:
        out.append({"name": "MANIFOLD_VACUUM", "desc": "Manifold vacuum (baro − MAP)", "value": round(baro - mapk, 1), "unit": "kPa"})
    st, lt = g("SHORT_FUEL_TRIM_1"), g("LONG_FUEL_TRIM_1")
    if st is not None and lt is not None:
        out.append({"name": "TOTAL_FUEL_TRIM_1", "desc": "Total fuel trim bank 1 (short + long)", "value": round(st + lt, 2), "unit": "%"})
    maf, spd = g("MAF"), g("SPEED")
    if maf is not None:
        lph = maf / 14.7 / 740 * 3600
        out.append({"name": "FUEL_FLOW_EST", "desc": "Estimated fuel flow from MAF", "value": round(lph, 2), "unit": "L/h"})
        if spd and spd > 3:
            out.append({"name": "ECONOMY_EST", "desc": "Estimated economy", "value": round(spd / lph, 1) if lph else None, "unit": "km/L"})
    rpm, spd = g("RPM"), g("SPEED")
    if rpm and spd and spd > 5:
        out.append({"name": "RPM_PER_KMH", "desc": "Engine rpm per km/h (indicates gear)", "value": round(rpm / spd, 1), "unit": "rpm/(km/h)"})
    return out


# ----------------------------------------------------------------------------------------
class Trip:
    """Trip computer. Fuel is estimated by speed-density (the Creta has no MAF sensor):
    air g/s = MAP * displacement * rpm/120 * VE / (R * T); fuel = air / (14.7 * lambda)."""
    def __init__(self):
        p = car_profile()
        self.disp = float(p.get("displacement_l", 1.6)) / 1000.0
        self.ve = float(p.get("volumetric_efficiency", 0.85))
        self.price = float(p.get("fuel_price", 100.0))
        self.cur = p.get("currency", "")
        self.car = cars.active_id()
        self.FILE = os.path.join(cars.car_dir(), "trip_last.json")
        self.FILL_FILE = os.path.join(cars.car_dir(), "fuel_calibration.json")
        self.fill = self.load_fill()
        self.fill_t = None
        self.fill_saved = 0
        self.reset()

    # ---- fill-to-fill calibration: real litres pumped vs litres estimated since the last full tank ----

    def load_fill(self):
        try:
            with open(self.FILL_FILE) as f:
                return json.load(f)
        except Exception:
            return {"since": None, "est_l": 0.0, "km": 0.0, "history": []}

    def save_fill(self):
        try:
            with open(self.FILL_FILE, "w") as f:
                json.dump(self.fill, f, indent=1)
        except Exception:
            pass

    def lph_from(self, live):
        g = lambda n: (live.get(n) or {}).get("value")
        rpm, mapk, iat, lam = g("RPM"), g("INTAKE_PRESSURE"), g("INTAKE_TEMP"), g("COMMANDED_EQUIV_RATIO")
        maf = g("MAF")
        if maf is not None and rpm and rpm > 300:          # measured airflow beats the speed-density estimate
            self.method = "airflow sensor (MAF)"
            return maf * (self.ve / 0.85) / (14.7 * (lam or 1.0)) * 3600 / 745.0
        self.method = "intake pressure estimate"
        if None in (rpm, mapk, iat) or rpm <= 300:
            return None
        air = (mapk * 1000) * (self.disp * rpm / 120.0) * self.ve / (287.05 * (iat + 273.15)) * 1000.0  # g/s
        return air / (14.7 * (lam or 1.0)) * 3600 / 745.0

    def fill_update(self, live):
        now = time.time()
        dt = 0 if self.fill_t is None else min(now - self.fill_t, 5.0)
        self.fill_t = now
        if not self.fill.get("since"):
            return
        lph = self.lph_from(live)
        spd = (live.get("SPEED") or {}).get("value")
        if lph:
            self.fill["est_l"] += lph * dt / 3600.0
        if spd:
            self.fill["km"] += spd * dt / 3600.0
        if now - self.fill_saved > 30:
            self.save_fill()
            self.fill_saved = now

    def fill_start(self):
        self.fill.update(since=time.time(), est_l=0.0, km=0.0)
        self.save_fill()

    def fill_done(self, litres):
        """Tank filled to full again: compare litres pumped with the estimate and correct the model."""
        est, km = self.fill.get("est_l", 0.0), self.fill.get("km", 0.0)
        if not self.fill.get("since") or est < 0.3 or litres <= 0:
            return {"ok": False, "why": "Drive a bit more first: at least a few litres since the full tank are needed."}
        factor = max(0.5, min(1.8, litres / est))
        old_ve = self.ve
        self.ve = round(max(0.4, min(1.2, self.ve * factor)), 4)
        try:
            p = car_profile()
            p["volumetric_efficiency"] = self.ve
            cars.save(p)
        except Exception:
            pass
        rec = {"at": time.time(), "pumped_l": litres, "estimated_l": round(est, 2), "km": round(km, 1),
               "real_kmpl": round(km / litres, 2) if litres else None, "estimated_kmpl": round(km / est, 2) if est else None,
               "factor": round(factor, 3), "ve_before": old_ve, "ve_after": self.ve}
        self.fill["history"] = (self.fill.get("history") or []) + [rec]
        self.fill.update(since=time.time(), est_l=0.0, km=0.0)      # tank is full again: next round starts now
        self.save_fill()
        return {"ok": True, **rec}

    def reset(self, active=False):
        self.active = active
        self.ended = None
        import trips
        self.series = {k: [] for k in ["t"] + trips.SERIES}
        self.series_next = 0
        self.t_last = None
        self.d = dict(started=time.time(), distance_km=0.0, fuel_l=0.0, engine_s=0.0, idle_s=0.0, moving_s=0.0,
                      max_speed=0.0, max_rpm=0.0, max_coolant=None, best_0_60=None, best_0_100=None,
                      runs=[], lph=None, kmpl_now=None)
        self.launch, self.run = None, {}


    def end(self):
        was_active = self.active
        self.active, self.ended = False, time.time()
        if was_active and len(self.series["t"]) > 10:
            try:
                import trips
                trips.save(self.snapshot(), self.series, self.d["started"], self.ended)
            except Exception:
                traceback.print_exc()
        try:
            with open(self.FILE, "w") as f:
                json.dump(dict(self.d, active=False, ended=self.ended), f, indent=1)
        except Exception:
            pass

    def load_last(self):
        """Show the last finished trip after a restart, until a new trip starts."""
        try:
            with open(self.FILE) as f:
                d = json.load(f)
            self.ended, self.active = d.pop("ended"), False
            d.pop("active", None)
            self.d.update(d)
        except Exception:
            pass

    def update(self, live):
        self.fill_update(live)
        if self.ended:                      # trip finished: keep the summary frozen
            return
        now_s = time.time()
        if self.active and now_s >= self.series_next:
            import trips
            self.series["t"].append(round(now_s, 1))
            for k in trips.SERIES:
                if k == "LPH":
                    self.series[k].append(round(self.lph_from(live) or 0.0, 2))
                else:
                    self.series[k].append((live.get(k) or {}).get("value"))
            self.series_next = now_s + 1.0
        g = lambda n: (live.get(n) or {}).get("value")
        rpm, spd, mapk, iat, lam = g("RPM"), g("SPEED"), g("INTAKE_PRESSURE"), g("INTAKE_TEMP"), g("COMMANDED_EQUIV_RATIO")
        now = time.time()
        dt = 0 if self.t_last is None else min(now - self.t_last, 5.0)
        self.t_last = now
        d = self.d
        if rpm is None:
            return
        if rpm > 300:
            d["engine_s"] += dt
        if spd is not None:
            d["distance_km"] += spd * dt / 3600.0
            d["max_speed"] = max(d["max_speed"], spd)
            if spd < 1 and rpm > 300:
                d["idle_s"] += dt
            elif spd >= 1:
                d["moving_s"] += dt
        d["max_rpm"] = max(d["max_rpm"], rpm)
        ct = g("COOLANT_TEMP")
        if ct is not None:
            d["max_coolant"] = ct if d["max_coolant"] is None else max(d["max_coolant"], ct)
        lph = self.lph_from(live)
        if lph is not None:
            d["lph"] = round(lph, 2)
            d["fuel_l"] += lph * dt / 3600.0
            if spd is not None and spd < 1:
                d["idle_fuel_l"] = d.get("idle_fuel_l", 0.0) + lph * dt / 3600.0
            d["kmpl_now"] = round(spd / lph, 1) if spd and spd > 3 and lph > 0 else None
        # acceleration timer: the clock starts when the car leaves a standstill
        if spd is not None:
            if spd < 1:
                self.launch, self.run = now, {}
            elif self.launch is not None:
                el = now - self.launch
                for target, key in ((60, "best_0_60"), (100, "best_0_100")):
                    if spd >= target and key not in self.run:
                        self.run[key] = el
                        d[key] = round(el if d[key] is None else min(d[key], el), 1)
                        d["runs"] = (d["runs"] + [{"to": target, "s": round(el, 1), "at": now}])[-10:]
                if spd >= 100 or el > 40:
                    self.launch = None

    def snapshot(self):
        d = dict(self.d)
        d["active"], d["ended"] = self.active, self.ended
        d["method"] = getattr(self, "method", None)
        f = self.fill
        d["fill"] = {"since": f.get("since"), "est_l": round(f.get("est_l", 0.0), 2), "km": round(f.get("km", 0.0), 1),
                     "last": (f.get("history") or [None])[-1], "ve": self.ve}
        d["cost"] = round(d["fuel_l"] * self.price, 2)
        d["currency"] = self.cur
        d["price"] = self.price
        d["avg_kmpl"] = round(d["distance_km"] / d["fuel_l"], 1) if d["fuel_l"] > 0.02 and d["distance_km"] > 0.2 else None
        d["fuel_l"] = round(d["fuel_l"], 3)
        d["distance_km"] = round(d["distance_km"], 2)
        return d


class CarOff(RuntimeError):
    """The adapter answers but the car doesn't: engine/ignition off."""


WAKE_VOLTS = 13.2     # alternator charging = engine running
PARK_POLL_S = 15


class CarReader(threading.Thread):
    daemon = True

    def __init__(self, state, port, baud):
        super().__init__(name="car")
        self.s, self.port, self.baud = state, port, baud
        self.jobs = queue.Queue()
        self.c = None
        self.csv = None
        self.bridge_gen = None

    # ------------- plumbing -------------
    def run(self):
        import obd
        self.obd = obd
        logging.getLogger("obd").setLevel(logging.CRITICAL)
        self.init_car()
        backoff = 2
        while True:
            try:
                if self.switch_pending:
                    self.init_car()
                self.wait_for_bridge()
                self.connect()
                backoff = 2
                self.session()
            except CarOff as e:
                self.drop_obd()
                self.parked(str(e))
                backoff = 1
                continue
            except Exception as e:
                with self.s.lock:
                    self.s.conn.update(status="reconnecting", error=f"{type(e).__name__}: {e}")
                    self.s.bump()
                traceback.print_exc()
            finally:
                self.drop_obd()
            time.sleep(backoff)
            backoff = min(backoff * 2, 20)

    def drop_obd(self):
        if self.c is not None:
            try:
                # Do not use OBD.close(): it sends ATZ and closes the link. Just drop the socket.
                self.c.interface._ELM327__port.close()
            except Exception:
                pass
            self.c = None

    # ------------- parked mode & automatic trips -------------
    auto_armed = True
    rpm_low_since = None
    wake_now = False

    def adapter_voltage(self):
        """Battery voltage measured by the adapter itself (ATRV). Sends nothing onto the car's network."""
        if not self.port.startswith("socket://"):
            return None
        host, port = self.port[len("socket://"):].rsplit(":", 1)
        try:
            with socket.create_connection((host, int(port)), timeout=4) as sk:
                sk.settimeout(4)
                sk.sendall(b"ATRV\r")
                buf, end = b"", time.time() + 4
                while b">" not in buf and time.time() < end:
                    chunk = sk.recv(256)
                    if not chunk:
                        break
                    buf += chunk
            m = re.findall(rb"(\d+\.\d+)\s*V", buf)
            return float(m[-1]) if m else None
        except OSError:
            return None

    def parked(self, why):
        """Car is off: stay quiet (no requests to the car, so its computers can sleep) and watch the
        battery voltage. When it rises above WAKE_VOLTS the engine has started: reconnect."""
        self.auto_armed = True
        while True:
            if self.switch_pending or self.wake_now:
                self.wake_now = False
                return
            b = self.bridge()
            v = self.adapter_voltage() if (b and b.get("state") == "up") else None
            with self.s.lock:
                self.s.conn.update(status="parked", error=None, bridge=b,
                                   parked_note=why if v is not None else "adapter not reachable (unplugged, or the car's socket has no power)")
                if v is not None:
                    self.s.conn["voltage"] = v
                    self.s.set_live("ELM_VOLTAGE", "Battery voltage at OBD port (adapter)", v, "V", mode=0)
                self.s.bump()
            if v is not None and v >= WAKE_VOLTS:
                print(f"engine start detected ({v} V)", flush=True)
                return
            for _ in range(PARK_POLL_S * 2):
                if self.switch_pending or self.wake_now:
                    break
                time.sleep(0.5)

    def auto_trip(self, rpm):
        """Start a trip when the engine starts, end it when the engine stops."""
        trip = self.s.trip
        now = time.time()
        if rpm is None:
            return
        if rpm < 300:
            self.rpm_low_since = self.rpm_low_since or now
            self.auto_armed = True
            if trip.active and now - self.rpm_low_since > 30:
                self.auto_end_trip("engine stopped")
            return
        self.rpm_low_since = None
        if self.auto_armed and not trip.active:
            self.auto_armed = False
            with self.s.lock:
                trip.reset(active=True)
                self.s.bump()
            self.focus_until = now + 60 * 60
            print("trip started automatically", flush=True)

    def auto_end_trip(self, why):
        trip = self.s.trip
        if trip.active:
            with self.s.lock:
                trip.end()
                self.s.bump()
            self.focus_until = 0
            print(f"trip ended automatically ({why})", flush=True)

    def init_car(self):
        """(Re)load everything that belongs to the active car profile."""
        import re_learn
        self.mfr_targets, self.learner, self.switch_pending = [], None, False
        with self.s.lock:
            self.s.reset_for_car()
        try:
            self.learner = re_learn.Learner()
            self.run_learner()
        except Exception:
            traceback.print_exc()

    def switch_car(self, cid, why="manual"):
        cars.set_active(cid)
        print(f"car profile -> {cid} ({why})", flush=True)
        self.switch_pending = True

    def bridge(self):
        try:
            with open(BRIDGE_STATUS) as f:
                b = json.load(f)
            if time.time() - b.get("updated", 0) > 10:
                b["state"] = "not running"
            return b
        except Exception:
            return None

    def wait_for_bridge(self):
        if not self.port.startswith("socket://"):
            return
        while True:
            b = self.bridge()
            with self.s.lock:
                self.s.conn["bridge"] = b
                if not b or b.get("state") != "up":
                    self.s.conn.update(status="waiting for bluetooth",
                                       error=None if b else "OBD Bridge.app is not running. Start it with ./start.sh")
                    self.s.bump()
            if b and b.get("state") == "up":
                self.bridge_gen = b.get("link_generation")
                return
            time.sleep(1)

    def connect(self):
        obd = self.obd
        with self.s.lock:
            self.s.conn.update(status="connecting", source=self.port, error=None)
            self.s.bump()
        c = obd.OBD(self.port, baudrate=self.baud, fast=False, timeout=3, check_voltage=True)
        st = c.status()
        if st != obd.OBDStatus.CAR_CONNECTED:
            msg = {"ELM Connected": "Adapter answers but the car does not. Is the ignition on?",
                   "OBD Connected": "Adapter is plugged in but ignition is off. Turn the key to ON or start the engine."}.get(str(st), str(st))
            try:
                c.interface._ELM327__port.close()
            except Exception:
                pass
            raise CarOff(msg)
        self.c = c
        self.hook_raw()
        with self.s.lock:
            self.s.conn.update(status="connected", protocol=c.protocol_name(), protocol_id=c.protocol_id(),
                               since=time.time(), error=None, queries=0, failures=0)
            self.s.bump()

    def hook_raw(self):
        iface = self.c.interface
        orig = iface.send_and_parse
        s = self.s

        def logged(cmd):
            t = time.time()
            msgs = orig(cmd)
            if msgs and cmd[:2] in (b"01", b"02", b"03", b"06", b"07", b"09", b"0A"):
                good = [m for m in msgs if not (m.data and m.data[0] == 0x7F)]
                if len(good) != len(msgs):
                    with s.lock:
                        s.add_raw(cmd.decode(errors="replace") + " (refused, ignored)", [m.raw() for m in msgs if m not in good], 0)
                msgs = good
            ms = round((time.time() - t) * 1000)
            rx = []
            for m in msgs or []:
                rx += [f.raw for f in m.frames] if m.frames else [m.raw()]
            with s.lock:
                s.add_raw(cmd.decode(errors="replace"), rx, ms)
                s.conn["queries"] += 1
                if not msgs:
                    s.conn["failures"] += 1
            return msgs
        iface.send_and_parse = logged

    def at(self, cmd):
        t = time.time()
        lines = self.c.interface._ELM327__send(cmd)
        with self.s.lock:
            self.s.add_raw(cmd.decode(), lines, round((time.time() - t) * 1000))
        return " ".join(l for l in lines if l and l != ">").strip()

    def q(self, cmd, force=False):
        return self.c.query(cmd, force=force)

    def raw_cmd(self, hexstr, name="RAW"):
        o = self.obd
        return o.OBDCommand(name, name, hexstr.encode(), 0, o.decoders.raw_string, o.ECU.ALL, False)

    def raw_msgs(self, hexstr):
        return self.q(self.raw_cmd(hexstr), force=True).messages

    # ------------- session -------------
    def session(self):
        self.adapter_info()
        self.discover()
        self.vehicle_info()
        self.diagnostics()
        self.open_csv()
        if not self.s.modules["list"] and not self.load_modules():
            self.scan_modules()
        self.live_loop()

    measured_lambda = None

    def wideband_decoder(self, messages):
        d = messages[0].data
        if len(d) >= 6:
            self.measured_lambda = ((d[2] << 8) | d[3]) * 2 / 65536.0
        return self.obd.decoders.current_centered(messages)

    def publish_lambda(self):
        """Caller holds the lock."""
        if self.measured_lambda is not None and 0.5 < self.measured_lambda < 2.5:
            self.s.set_live("O2_S1_LAMBDA", "Measured air-fuel ratio (wideband lambda)", round(self.measured_lambda, 3), "λ", 1, 0x34)

    def adapter_info(self):
        v = self.at(b"ATI")
        dev = self.at(b"AT@1")
        dp = self.at(b"ATDP")
        rv = self.at(b"ATRV")
        with self.s.lock:
            self.s.conn.update(elm_version=v, elm_device=dev, protocol_desc=dp)
            vv = re.findall(r"[\d.]+", rv)
            if vv:
                self.s.set_live("ELM_VOLTAGE", "Battery voltage at OBD port (adapter)", float(vv[0]), "V", mode=0)
            self.s.bump()

    def bitmap(self, mode, maxpid=0xE0):
        """Return the set of PIDs the ECU says it supports for a mode (01 / 06 / 09)."""
        found = set()
        base = 0
        while base <= maxpid:
            msgs = self.raw_msgs(f"{mode:02X}{base:02X}")
            bits = 0
            for m in msgs or []:
                d = m.data
                if len(d) >= 6 and d[0] == mode + 0x40 and d[1] == base:
                    bits |= int.from_bytes(bytes(d[2:6]), "big")
            if not bits:
                break
            for i in range(32):
                if bits & (1 << (31 - i)):
                    found.add(base + i + 1)
            if not (bits & 1):
                break
            base += 0x20
        return found

    def discover(self):
        obd = self.obd
        m1 = self.bitmap(1)
        m9 = self.bitmap(9, 0x20)
        self.m1, self.m9 = m1, m9
        known1 = {c.pid: c for c in obd.commands[1] if c is not None}
        self.live_cmds, self.string_cmds, self.unknown_m1 = [], [], []
        for pid in sorted(m1):
            if pid % 0x20 == 0:
                continue
            cmd = known1.get(pid)
            if cmd is None:
                self.unknown_m1.append(pid)
            elif cmd.name in STRING_M1:
                self.string_cmds.append(cmd)
            elif cmd.name not in NEVER:
                if pid == 0x34:                      # wideband O2: python-obd keeps only the current; keep lambda too
                    cmd = obd.OBDCommand("O2_S1_WR_CURRENT", cmd.desc, b"0134", 6, self.wideband_decoder, obd.ECU.ENGINE, True)
                self.live_cmds.append(cmd)
        sup = []
        for cmd in sorted(self.c.supported_commands, key=lambda x: (x.mode if x.mode is not None else 99, x.pid or 0)):
            if cmd.mode is None or cmd.name.startswith(("PIDS_", "MIDS_")):
                continue
            sup.append({"mode": cmd.mode, "pid": cmd.pid, "name": cmd.name, "desc": cmd.desc,
                        "polled": cmd.name not in NEVER and cmd.mode != 2,
                        "note": "never sent (would clear codes)" if cmd.name in NEVER else
                                ("freeze frame, read only when a code is stored" if cmd.mode == 2 else "")})
        for pid in self.unknown_m1:
            sup.append({"mode": 1, "pid": pid, "name": f"PID_01_{pid:02X}", "desc": "Not decoded by python-obd (shown raw)", "polled": True, "note": ""})
        for pid in sorted(m9):
            if not any(x["mode"] == 9 and x["pid"] == pid for x in sup):
                sup.append({"mode": 9, "pid": pid, "name": f"INFO_09_{pid:02X}", "desc": "Vehicle information (raw)", "polled": True, "note": ""})

        # Multi-PID requests (CAN only): up to 6 PIDs per request.
        self.multi = False
        if str(self.c.protocol_id()) in ("6", "7", "8", "9") and len(self.live_cmds) >= 2:
            a, b = self.live_cmds[0], self.live_cmds[1]
            got = self.multi_query([a, b])
            self.multi = len(got) == 2
        with self.s.lock:
            self.s.supported = sup
            self.s.conn["multi_pid"] = self.multi
            self.s.bump()

    def vehicle_info(self):
        v = {}
        for pid in sorted(self.m9):
            msgs = self.raw_msgs(f"09{pid:02X}")
            if not msgs:
                continue
            payloads = [bytes(m.data[3:]) for m in msgs if len(m.data) > 3]
            hexs = " ".join(bytes(m.data).hex(" ").upper() for m in msgs)
            ecus = len(msgs)
            if pid == 0x02:
                v["vin"] = text(payloads[0]) if payloads else None
            elif pid == 0x04:
                ids = []
                for p in payloads:
                    ids += [text(p[i:i + 16]) for i in range(0, len(p), 16) if text(p[i:i + 16])]
                v["calibration_ids"] = ids
            elif pid == 0x06:
                cv = []
                for p in payloads:
                    cv += [p[i:i + 4].hex().upper() for i in range(0, len(p) - 3, 4)]
                v["cvn"] = cv
            elif pid == 0x0A:
                v["ecu_names"] = [re.sub(r"\s*-\s*", " - ", p.replace(b"\x00", b" ").decode(errors="replace")).strip(" -") for p in payloads]
            elif pid == 0x08 or pid == 0x0B:
                d = payloads[0] if payloads else b""
                counters = [int.from_bytes(d[i:i + 2], "big") for i in range(0, len(d) - 1, 2)]
                names = ["OBD monitoring conditions encountered", "Ignition cycles", "Catalyst B1 completions", "Catalyst B1 conditions",
                         "Catalyst B2 completions", "Catalyst B2 conditions", "O2 B1 completions", "O2 B1 conditions",
                         "O2 B2 completions", "O2 B2 conditions", "EGR/VVT completions", "EGR/VVT conditions",
                         "AIR completions", "AIR conditions", "EVAP completions", "EVAP conditions",
                         "Secondary O2 B1 completions", "Secondary O2 B1 conditions", "Secondary O2 B2 completions", "Secondary O2 B2 conditions"]
                v["in_use_performance"] = [{"name": names[i] if i < len(names) else f"Counter {i}", "value": n} for i, n in enumerate(counters)]
            v.setdefault("raw", {})[f"09 {pid:02X}"] = hexs
        if v.get("vin"):
            v.update(decode_vin(v["vin"]))
        v["profile"] = car_profile()
        guess = cars.match(v.get("vin"), v.get("calibration_ids") or [])
        if guess and guess != cars.active_id():
            self.switch_car(guess, f"recognised by {'VIN' if v.get('vin') else 'calibration number'}")
            raise RuntimeError(f"this is the {cars.load(guess).get('short', guess)}; switching profile")
        with self.s.lock:
            self.s.vehicle = v
            self.s.bump()

    def diagnostics(self):
        obd = self.obd
        out = {}
        r = self.q(obd.commands.GET_DTC)
        if r.value is None:                  # no answer (ignition off / link lost): keep the last good results
            return
        out["stored"] = [list(x) for x in (r.value or [])]
        r = self.q(obd.commands.GET_CURRENT_DTC)
        out["pending"] = [list(x) for x in (r.value or [])]
        perm = obd.OBDCommand("PERMANENT_DTC", "Permanent DTCs", b"0A", 0, obd.decoders.dtc, obd.ECU.ALL, False)
        r = self.q(perm, force=True)
        out["permanent"] = [list(x) for x in (r.value or [])]
        out["updated"] = time.time()

        status, cycle = self.s.status, self.s.status_cycle
        if 0x01 in self.m1:
            status = ser_status(self.q(obd.commands.STATUS).value) or status
        if 0x41 in self.m1:
            cycle = ser_status(self.q(obd.commands.STATUS_DRIVE_CYCLE).value) or cycle

        info = {}
        for cmd in self.string_cmds:
            if cmd.name in ("STATUS", "STATUS_DRIVE_CYCLE"):
                continue
            val = self.q(cmd).value
            if cmd.name == "O2_SENSORS" and val:
                # python-obd lists the 8 bits most-significant first; bit 0 (LSB) is bank 1 sensor 1
                flat = list(val[1] or ()) + list(val[2] or ())
                present = [f"Bank {1 if (7 - i) < 4 else 2} sensor {(7 - i) % 4 + 1}" for i, b in enumerate(flat) if b]
                val = ", ".join(sorted(present)) or "none"
            elif isinstance(val, (tuple, list)):
                val = " / ".join(str(x) for x in val if x)
            info[cmd.name] = {"desc": cmd.desc, "value": None if val is None else str(val)}

        freeze = []
        if out["stored"] or (status and status.get("mil")):
            for cmd in self.c.supported_commands:
                if cmd.mode == 2 and not cmd.name.startswith("DTC_PIDS"):
                    val = self.q(cmd).value
                    if val is None:
                        continue
                    n, u = num(val)
                    freeze.append({"name": cmd.name[4:], "desc": cmd.desc, "value": round(n, 3) if n is not None else str(val), "unit": u or ""})

        m06 = []
        for cmd in sorted(self.c.supported_commands, key=lambda x: x.pid or 0):
            if cmd.mode != 6 or cmd.name.startswith("MIDS_"):
                continue
            mon = self.q(cmd).value
            if not mon:
                continue
            tests = []
            for t in mon.tests:
                v, u = num(t.value)
                lo, _ = num(t.min)
                hi, _ = num(t.max)
                tests.append({"tid": t.tid, "name": t.name, "desc": t.desc if t.desc and t.desc != "Unknown" else f"Test 0x{t.tid:02X}",
                              "value": v, "min": lo, "max": hi, "unit": u or "", "passed": bool(t.passed)})
            m06.append({"monitor": cmd.name.replace("MONITOR_", ""), "desc": cmd.desc, "tests": tests})

        with self.s.lock:
            self.s.dtcs = out
            self.s.status, self.s.status_cycle = status, cycle
            self.s.info = info
            self.s.freeze = freeze
            if m06:
                self.s.mode06, self.s.mode06_updated = m06, time.time()
            self.s.bump()

    # ------------- live -------------
    def multi_query(self, cmds):
        from obd.protocols.protocol import Message
        req = "01" + "".join(f"{c.pid:02X}" for c in cmds)
        msgs = self.raw_msgs(req)
        by = {c.pid: c for c in cmds}
        out = {}
        for m in msgs or []:
            d = m.data
            if not d or d[0] != 0x41:
                continue
            i = 1
            while i < len(d):
                cmd = by.get(d[i])
                if cmd is None or cmd.bytes <= 2:
                    break
                n = cmd.bytes - 2
                chunk = d[i + 1:i + 1 + n]
                if len(chunk) < n:
                    break
                fake = Message(m.frames)
                fake.ecu = m.ecu
                fake.data = bytearray([0x41, d[i]]) + chunk
                try:
                    out[cmd.name] = (cmd, cmd.decode([fake]))
                except Exception:
                    pass
                i += 1 + n
        return out

    def read(self, cmds):
        results = {}
        multi_ok = [c for c in cmds if c.bytes > 2] if self.multi else []
        singles = [c for c in cmds if c not in multi_ok]
        for i in range(0, len(multi_ok), 6):
            batch = multi_ok[i:i + 6]
            got = self.multi_query(batch)
            results.update(got)
            for c in batch:
                if c.name not in got:
                    singles.append(c)
        for c in singles:
            results[c.name] = (c, self.q(c).value)
        return results

    def open_csv(self):
        path = os.path.join(LOGDIR, time.strftime("drive-%Y%m%d-%H%M%S.csv"))
        self.csv_f = open(path, "w", newline="")
        self.csv = csv.writer(self.csv_f)
        self.csv.writerow(["timestamp", "pid", "value", "unit"])
        with self.s.lock:
            self.s.conn["csv"] = os.path.relpath(path, HERE)

    def live_loop(self):
        obd = self.obd
        if self.watch_until - time.time() > 5:          # a switch-finder run was cut short by a link drop: resume it
            self.watch_resume = True
            self.jobs.put(("watch", int(self.watch_until - time.time())))
        cycle = 0
        last_diag = time.time()
        last_learn = time.time()
        last_rv = 0
        rate_t, rate_n = time.time(), 0
        empty_streak = 0
        fast = [c for c in self.live_cmds if c.name in FAST]
        slow = [c for c in self.live_cmds if c.name not in FAST]
        while True:
            self.handle_jobs()
            if self.switch_pending:
                raise RuntimeError("car profile changed")
            if self.port.startswith("socket://"):
                b = self.bridge()
                with self.s.lock:
                    self.s.conn["bridge"] = b
                if not b or b.get("state") != "up":
                    raise RuntimeError("Bluetooth link dropped; waiting for the bridge to restore it")
                if b.get("link_generation") != self.bridge_gen:
                    raise RuntimeError("Bluetooth link was re-established; re-initialising the adapter")

            t0 = time.time()
            focus = time.time() < self.focus_until
            if focus and self.learner is not None and any(h == "7E0" for h, _ in self.mfr_targets):
                self.focus_burst(fast, 6.0)
                with self.s.lock:
                    self.s.conn["focus_until"] = self.focus_until
                    self.s.conn["status"] = "connected"
                    self.s.bump()
                if self.learner and time.time() - last_learn > 8:
                    self.run_learner()
                    last_learn = time.time()
                cycle += 1
                continue
            if focus:
                batch = [c for c in fast if c.name in FOCUS_REFS] + ([c for c in slow if c.name in FOCUS_REFS] if cycle % 3 == 0 else [])
            else:
                batch = fast + (slow if cycle % SLOW_EVERY == 0 else [])
            res = self.read(batch)
            if cycle % SLOW_EVERY == 0:
                for pid in self.unknown_m1:
                    msgs = self.raw_msgs(f"01{pid:02X}")
                    with self.s.lock:
                        self.s.unknown[f"01 {pid:02X}"] = " ".join(bytes(m.data).hex(" ").upper() for m in msgs) if msgs else None
            got = 0
            now = time.time()
            with self.s.lock:
                for name, (cmd, val) in res.items():
                    v, u = num(val)
                    if v is not None:
                        got += 1
                        self.s.set_live(name, cmd.desc, v, u, 1, cmd.pid)
                        self.csv.writerow([f"{now:.3f}", name, f"{v:.4g}", u])
                self.publish_lambda()
                dt = time.time() - t0
                self.s.conn["cycle_ms"] = round(dt * 1000)
                rate_n += got
                if now - rate_t >= 2:
                    self.s.conn["pids_per_sec"] = round(rate_n / (now - rate_t), 1)
                    rate_t, rate_n = now, 0
                self.s.conn["status"] = "connected"
                self.s.trip.update(self.s.live)
                self.s.bump()
            if now - last_rv > 3:
                rv = self.at(b"ATRV")
                vv = re.findall(r"\d+\.\d+|\d+", rv)
                with self.s.lock:
                    if vv:
                        self.s.set_live("ELM_VOLTAGE", "Battery voltage at OBD port (adapter)", float(vv[0]), "V", mode=0)
                        self.s.conn["voltage"] = float(vv[0])
                last_rv = now
                self.csv_f.flush()

            empty_streak = empty_streak + 1 if got == 0 and batch else 0
            if empty_streak >= 4:
                raise CarOff("Car stopped answering (ignition off?)")
            if time.time() - last_diag > 30:
                self.diagnostics()
                last_diag = time.time()
            if self.mfr_targets:
                self.poll_manufacturer(per_cycle=4 if focus else 2)
            with self.s.lock:
                self.s.conn["focus_until"] = self.focus_until if focus else None
            if self.learner and time.time() - last_learn > 8:
                self.run_learner()
                last_learn = time.time()
            cycle += 1

    # ------------- Hyundai / manufacturer modules -------------
    mfr_targets = []

    def scanner(self):
        import hyundai
        return hyundai.Scanner(self, log=lambda m: print(m, flush=True), make=car_profile().get("make", "Hyundai"))

    def scan_modules(self):
        import hyundai
        sc = self.scanner()
        with self.s.lock:
            self.s.modules.update(running=True, progress={"i": 0, "n": len(sc.modules)})
            self.s.conn["status_note"] = "scanning vehicle modules"
            self.s.bump()

        def prog(i, n, hdr, name):
            with self.s.lock:
                self.s.modules["progress"] = {"i": i, "n": n, "address": hdr, "name": name}
                self.s.bump()
        try:
            mods = sc.scan_all(prog)
        finally:
            with self.s.lock:
                self.s.modules["running"] = False
                self.s.conn["status_note"] = None
        # keep re-reading everything a module answered (decoded or raw), capped to keep live data fast
        self.mfr_targets = [(m["address"], r["request"]) for m in mods if m["present"] for r in m["reads"]][:24]
        if self.learner is None:
            import re_learn
            self.learner = re_learn.Learner()
        try:
            with open(self.modules_file(), "w") as f:
                json.dump(mods, f, indent=1, default=str)
        except Exception:
            pass
        with self.s.lock:
            self.s.modules.update(list=mods, updated=time.time(), progress=None)
            for m in mods:
                for r in m["reads"]:
                    self.publish_mfr(m, r)
            self.s.bump()

    def modules_file(self):
        return os.path.join(cars.car_dir(), "modules.json")

    def load_modules(self):
        try:
            with open(self.modules_file()) as f:
                mods = json.load(f)
        except Exception:
            return False
        self.mfr_targets = [(m["address"], r["request"]) for m in mods if m["present"] for r in m["reads"]][:24]
        if self.learner is None:
            import re_learn
            self.learner = re_learn.Learner()
        with self.s.lock:
            self.s.modules.update(list=mods, updated=os.path.getmtime(self.modules_file()), progress=None)
            self.s.bump()
        return True

    def publish_mfr(self, mod, read):
        """Caller holds the lock."""
        group = car_profile().get("make", "Maker") + " secret data"
        for v in read["values"]:
            if v.get("verified") is False:          # layout not confirmed for this car: keep it off the main screens
                continue
            label = v.get("label") or v["name"].replace("_", " ").title()
            self.s.set_live("HY_" + v["name"].upper(), label, v["value"], v["unit"],
                            mode=0x22, pid=read["request"], group=group, tag=f"{mod['address']} {read['req']}")

    learner = None
    rr = 0
    focus_until = 0

    def poll_manufacturer(self, per_cycle=3):
        """Round-robin a few manufacturer blocks per cycle and feed the reverse-engineering learner."""
        sc = self.scanner()
        # Prioritise blocks that actually change; static ones are re-checked 1 time in 8.
        static = set()
        if self.learner is not None:
            for b in self.learner.result.get("blocks", []):
                if b["n"] >= 5 and all(y["class"] == "constant" for y in b["bytes"]):
                    static.add((b["hdr"], b["req"]))
        self.rr += 1
        pool = [t for t in self.mfr_targets if t not in static] or self.mfr_targets
        if static and self.rr % 8 == 0:
            pool = sorted(static)
        n = len(pool)
        k0 = (self.rr * per_cycle) % n
        pick = [pool[(k0 + k) % n] for k in range(min(per_cycle, n))]
        got = sc.read_live(pick)
        mods = {m["address"]: m for m in self.s.modules["list"]}
        now = time.time()
        with self.s.lock:
            refs = {k: r.get("value") for k, r in self.s.live.items()}
        if self.learner is not None:
            for (hdr, req), read in got.items():
                self.learner.add(hdr, req, bytes.fromhex(read["hex"].replace(" ", "")), refs, now)
        with self.s.lock:
            for (hdr, req), read in got.items():
                m = mods.get(hdr)
                if not m:
                    continue
                for i, old in enumerate(m["reads"]):
                    if old["request"] == req:
                        m["reads"][i] = read
                self.publish_mfr(m, read)
                for v in read["values"]:
                    self.csv.writerow([f"{now:.3f}", "HY_" + v["name"].upper(), f"{v['value']:.4g}", v["unit"]])
            self.s.bump()

    QUICK = ["RPM", "INTAKE_PRESSURE", "THROTTLE_POS", "ENGINE_LOAD", "TIMING_ADVANCE", "SPEED"]

    def focus_burst(self, fast, seconds):
        """Learn mode: stay on the engine ECU's address; read quick references, one block, quick references again.
        The ECU needs ~0.8 s per 21xx block, so bracketing keeps references aligned with the block."""
        quick = [c for c in fast if c.name in self.QUICK]
        sc = self.scanner()
        static = set()
        for b in self.learner.result.get("blocks", []):
            if b["n"] >= 5 and all(y["class"] == "constant" for y in b["bytes"]):
                static.add((b["hdr"], b["req"]))
        pool = [t for t in self.mfr_targets if t[0] == "7E0" and t not in static] or [t for t in self.mfr_targets if t[0] == "7E0"]
        end = time.time() + seconds
        try:
            # slower values (temperatures, voltages, pedal, trims) once per burst so the screen never freezes
            others = [c for c in self.live_cmds if c.name in FOCUS_REFS or c.name in FAST]
            others = [c for c in others if c.name not in self.QUICK]
            got = self.read(others)
            now = time.time()
            with self.s.lock:
                for name, (cmd, val) in got.items():
                    v, u = num(val)
                    if v is not None:
                        self.s.set_live(name, cmd.desc, v, u, 1, cmd.pid)
                        self.csv.writerow([f"{now:.3f}", name, f"{v:.4g}", u])
                self.publish_lambda()
            rv = re.findall(r"\d+\.\d+|\d+", self.at(b"ATRV"))
            if rv:
                with self.s.lock:
                    self.s.set_live("ELM_VOLTAGE", "Battery voltage at OBD port (adapter)", float(rv[0]), "V", mode=0)
                    self.s.conn["voltage"] = float(rv[0])
            before = self.read(quick)
            while time.time() < end:
                self.rr += 1
                hdr, req = pool[self.rr % len(pool)]
                sc.select(hdr)                      # private block on the module's own address...
                r = sc.send(req)
                sc.restore()                        # ...standard values back on the broadcast address (Suzuki refuses them on 7E0)
                after = self.read(quick)
                now = time.time()
                with self.s.lock:
                    for name, (cmd, val) in after.items():
                        v, u = num(val)
                        if v is not None:
                            self.s.set_live(name, cmd.desc, v, u, 1, cmd.pid)
                            self.csv.writerow([f"{now:.3f}", name, f"{v:.4g}", u])
                    refs = {k: rec.get("value") for k, rec in self.s.live.items()}
                    self.s.trip.update(self.s.live)
                for name in self.QUICK:            # average of the readings either side of the block
                    a, b = num(before.get(name, (None, None))[1])[0], num(after.get(name, (None, None))[1])[0]
                    if a is not None and b is not None:
                        refs[name] = (a + b) / 2
                if r and r["ok"] and len(r["data"]) > len(req) // 2:
                    self.learner.add(hdr, req, bytes(r["data"][len(req) // 2:]), refs, now)
                before = after
        finally:
            sc.restore()

    DEC_LABEL = {"SPEED": "Hidden road speed", "RPM": "Hidden engine speed", "RUN_TIME": "Hidden running timer",
                 "INTAKE_PRESSURE": "Hidden intake pressure", "TIMING_ADVANCE": "Hidden spark timing",
                 "ENGINE_LOAD": "Hidden engine load", "THROTTLE_POS": "Hidden throttle", "COOLANT_TEMP": "Hidden engine temperature"}
    DEC_UNIT = {"SPEED": "km/h", "RPM": "rpm", "RUN_TIME": "s", "INTAKE_PRESSURE": "kPa", "TIMING_ADVANCE": "°",
                "ENGINE_LOAD": "%", "THROTTLE_POS": "%", "COOLANT_TEMP": "°C"}

    def latest_blocks(self):
        return {k: dq[-1][1] for k, dq in self.learner.samples.items() if dq}

    def run_learner(self, analyse=True):
        if analyse or not getattr(self, "last_res", None):
            self.last_res = self.learner.analyse()
            self.learner.flush()
        res = self.last_res
        with self.s.lock:
            self.s.re = res
            # publish decoded values as live readings (from the newest sample of each block)
            latest = self.latest_blocks()
            def raw_at(hdr, req, off, typ):
                b = latest.get((hdr, req))
                if b is None or off + (2 if "16" in typ else 1) > len(b):
                    return None
                if typ == "u8": return b[off]
                if typ == "s8": return b[off] - 256 if b[off] > 127 else b[off]
                le, be = b[off] | (b[off + 1] << 8), (b[off] << 8) | b[off + 1]
                v = {"u16le": le, "u16be": be, "s16le": le, "s16be": be}[typ]
                return v - 65536 if typ.startswith("s16") and v > 32767 else v
            for f in res.get("fields", []):
                if not f.get("primary") or f["ref"] == "RUN_TIME":
                    continue
                raw = raw_at(f["hdr"], f["req"], f["offset"], f["type"])
                if raw is None:
                    continue
                self.s.set_live("DEC_" + f["ref"], self.DEC_LABEL.get(f["ref"], "Hidden " + f["ref"].lower()),
                                round(f["scale"] * raw + f["bias"], 2), self.DEC_UNIT.get(f["ref"], ""), mode=0x21,
                                pid=f["req"], group=car_profile().get("make", "Maker") + " secret data", tag=f"{f['req'][:2]} {f['req'][2:]} byte {f['offset']}")
            for g in res.get("groups", []):
                for k, c in enumerate(g["cylinders"]):
                    raw = raw_at(g["hdr"], g["req"], g["offset"] + k, g["type"][:2])
                    if raw is None:
                        continue
                    self.s.set_live(f"DEC_{g['ref']}_CYL{k + 1}", f"{self.DEC_LABEL.get(g['ref'], g['ref'])}, cylinder {k + 1}",
                                    round(c["scale"] * raw + c["bias"], 2), self.DEC_UNIT.get(g["ref"], ""), mode=0x21,
                                    pid=g["req"], group=car_profile().get("make", "Maker") + " secret data", tag=f"{g['req'][:2]} {g['req'][2:]} byte {g['offset'] + k}")
            self.s.bump()

    watch_stop = False
    watch_until = 0

    def switch_watch(self, seconds):
        """Switch finder: re-read the private blocks as fast as the ECU allows and log every bit that flips,
        tagged with the action the user said they were doing. Read-only (service 21/22)."""
        static = set()
        if self.learner is not None:
            for b in self.learner.result.get("blocks", []):
                if b["n"] >= 20 and all(y["class"] == "constant" for y in b["bytes"]):
                    static.add((b["hdr"], b["req"]))
        targets = [t for t in self.mfr_targets if t not in static] or list(self.mfr_targets)
        if not targets:
            return
        self.watch_until = time.time() + seconds
        sc = self.scanner()
        prev = {}
        end = time.time() + seconds
        resumed = self.s.watch.get("events") and getattr(self, "watch_resume", False)
        self.watch_stop, self.watch_resume = False, False
        with self.s.lock:
            if resumed:                                 # carry on after a Bluetooth drop, keep earlier results
                self.s.watch.update(running=True, until=end)
            else:
                self.s.watch.update(running=True, until=end, events=[], blocks={}, label=None, label_t=None)
            self.s.bump()
        try:
            i = 0
            while time.time() < end and not self.watch_stop and not self.switch_pending:
                hdr, req = targets[i % len(targets)]
                i += 1
                sc.select(hdr)
                r = sc.send(req)
                if not (r and r["ok"]):
                    continue
                payload = bytes(r["data"][len(req) // 2:])
                key = f"{hdr} {req}"
                now = time.time()
                with self.s.lock:
                    w = self.s.watch
                    old = prev.get(key)
                    if old is not None:
                        for bi, (a, b) in enumerate(zip(old, payload)):
                            if a != b:
                                for bit in range(8):
                                    if (a ^ b) & (1 << bit):
                                        lab = w["label"] if w["label_t"] and now - w["label_t"] < 8 else None
                                        w["events"].append({"t": now, "block": key, "byte": bi, "bit": bit,
                                                            "from": (a >> bit) & 1, "to": (b >> bit) & 1, "label": lab})
                        w["events"] = w["events"][-200:]
                    prev[key] = payload
                    w["blocks"][key] = payload.hex()
                    self.s.bump()
            self.watch_until = 0                       # finished normally or stopped
        finally:
            try:
                sc.restore()
            except Exception:
                pass
            with self.s.lock:
                self.s.watch.update(running=False, until=None)
                self.s.bump()

    def handle_jobs(self):
        while not self.jobs.empty():
            job, arg = self.jobs.get_nowait()
            if job == "refresh":
                self.diagnostics()
                self.vehicle_info()
            elif job == "sniff":
                self.sniff(arg)
            elif job == "modules":
                self.scan_modules()
            elif job == "watch":
                self.switch_watch(arg)

    def sniff(self, seconds):
        """Passive CAN bus monitor (ATMA). The adapter only listens; nothing is sent to the car."""
        port = self.c.interface._ELM327__port
        frames = {}
        lines = 0
        with self.s.lock:
            self.s.sniff = {"running": True, "frames": [], "lines": 0, "started": time.time(), "seconds": seconds, "note": None}
            self.s.bump()
        note = None
        old_to = port.timeout
        port.timeout = 0.05
        try:
            port.reset_input_buffer()
            port.write(b"ATMA\r")
            port.flush()
            buf = b""
            end = time.time() + seconds
            last_pub = 0
            while time.time() < end:
                buf += port.read(4096)
                *done, buf = buf.split(b"\r")
                now = time.time()
                for raw in done:
                    line = raw.decode(errors="replace").strip().strip(">").strip()
                    if not line or line == "ATMA":
                        continue
                    if line in ("BUFFER FULL", "STOPPED", "?", "CAN ERROR", "NO DATA"):
                        note = line
                        continue
                    toks = line.split()
                    if len(toks) == 1 and re.fullmatch(r"[0-9A-Fa-f]+", toks[0]):
                        s = toks[0]
                        idl = 3 if self.c.protocol_id() in ("6", "8") else 8
                        toks = [s[:idl]] + [s[i:i + 2] for i in range(idl, len(s), 2)]
                    if not all(re.fullmatch(r"[0-9A-Fa-f]{1,8}", t) for t in toks):
                        continue
                    lines += 1
                    fid, data = toks[0].upper(), [t.upper() for t in toks[1:]]
                    f = frames.get(fid)
                    if f is None:
                        f = frames[fid] = {"id": fid, "count": 0, "first": now, "data": data, "changed": [0] * len(data)}
                    if len(f["changed"]) < len(data):
                        f["changed"] += [0] * (len(data) - len(f["changed"]))
                    for i, b in enumerate(data):
                        if i >= len(f["data"]) or f["data"][i] != b:
                            f["changed"][i] += 1
                    f["data"], f["count"], f["last"] = data, f["count"] + 1, now
                if now - last_pub > 0.4:
                    self.publish_sniff(frames, lines, note, True)
                    last_pub = now
        finally:
            port.write(b"\r")
            port.flush()
            t = time.time()
            tail = b""
            while time.time() - t < 3 and b">" not in tail:
                tail += port.read(256)
            port.timeout = old_to
            port.reset_input_buffer()
        self.publish_sniff(frames, lines, note or ("no frames seen; this car's OBD port may be behind a gateway" if not frames else None), False)

    def publish_sniff(self, frames, lines, note, running):
        rows = []
        for f in sorted(frames.values(), key=lambda x: int(x["id"], 16)):
            span = max(f.get("last", f["first"]) - f["first"], 0.001)
            rows.append({"id": f["id"], "count": f["count"], "hz": round((f["count"] - 1) / span, 1) if f["count"] > 1 else None,
                         "data": f["data"], "changed": f["changed"]})
        with self.s.lock:
            self.s.sniff.update(running=running, frames=rows, lines=lines, note=note)
            self.s.bump()


def ser_status(s):
    if s is None:
        return None
    tests = []
    for k, t in vars(s).items():
        if k and hasattr(t, "available") and hasattr(t, "complete") and k not in ("MIL",):
            tests.append({"name": k.replace("_MONITORING", "").replace("_", " ").title(),
                          "available": bool(t.available), "complete": bool(t.complete) if t.available else None})
    tests.sort(key=lambda x: (not x["available"], x["name"]))
    return {"mil": bool(s.MIL), "dtc_count": int(s.DTC_count), "ignition": s.ignition_type, "tests": tests}


WMI = {"MAL": "Hyundai (India)", "MA3": "Maruti Suzuki", "MAT": "Tata Motors", "MAJ": "Ford India", "MA1": "Mahindra",
       "MAK": "Honda Cars India", "MBH": "Suzuki (India)", "MBJ": "Toyota Kirloskar", "MEX": "Volkswagen India",
       "MZB": "Kia (India)", "MZ7": "MG Motor India", "MBL": "Hero MotoCorp", "MC2": "Volvo Eicher",
       "KMH": "Hyundai (Korea)", "KNA": "Kia (Korea)", "JHM": "Honda (Japan)", "JT": "Toyota (Japan)",
       "WVW": "Volkswagen", "WBA": "BMW", "WDD": "Mercedes-Benz", "1HG": "Honda (USA)", "SAL": "Land Rover"}
YEARS = "ABCDEFGHJKLMNPRSTVWXY123456789"


def car_profile():
    return cars.active()


def decode_vin(vin):
    out = {}
    wmi = vin[:3].upper()
    out["maker"] = WMI.get(wmi) or WMI.get(wmi[:2])
    if len(vin) == 17:
        c = vin[9].upper()
        if c in YEARS:
            i = YEARS.index(c)
            out["model_year_candidates"] = [1980 + i, 2010 + i] if 2010 + i <= time.localtime().tm_year + 1 else [1980 + i]
        out["vin_parts"] = {"WMI (maker)": vin[:3], "VDS (model/body/engine)": vin[3:9], "Check digit / region": vin[8],
                            "Model year code": vin[9], "Plant": vin[10], "Serial": vin[11:]}
    return out


# ----------------------------------------------------------------------------------------
class DemoReader(threading.Thread):
    """Fake data with the same shape as a real car, for trying the dashboard without one."""
    daemon = True

    def __init__(self, state):
        super().__init__(name="demo")
        self.s = state
        self.jobs = queue.Queue()

    def run(self):
        s = self.s
        spec = [("RPM", "Engine speed", "rpm"), ("SPEED", "Vehicle speed", "km/h"), ("ENGINE_LOAD", "Calculated engine load", "%"),
                ("COOLANT_TEMP", "Engine coolant temperature", "°C"), ("INTAKE_TEMP", "Intake air temperature", "°C"),
                ("INTAKE_PRESSURE", "Intake manifold pressure", "kPa"), ("TIMING_ADVANCE", "Timing advance", "°"),
                ("THROTTLE_POS", "Throttle position", "%"), ("SHORT_FUEL_TRIM_1", "Short term fuel trim - bank 1", "%"),
                ("LONG_FUEL_TRIM_1", "Long term fuel trim - bank 1", "%"), ("O2_B1S1", "O2: bank 1 - sensor 1 voltage", "V"),
                ("O2_B1S2", "O2: bank 1 - sensor 2 voltage", "V"), ("CONTROL_MODULE_VOLTAGE", "Control module voltage", "V"),
                ("BAROMETRIC_PRESSURE", "Barometric pressure", "kPa"), ("COMMANDED_EQUIV_RATIO", "Commanded equivalence ratio", "λ"),
                ("CATALYST_TEMP_B1S1", "Catalyst temperature: bank 1 - sensor 1", "°C"), ("AMBIANT_AIR_TEMP", "Ambient air temperature", "°C"),
                ("RUN_TIME", "Engine run time", "s"), ("DISTANCE_SINCE_DTC_CLEAR", "Distance traveled since codes cleared", "km")]
        with s.lock:
            s.conn.update(status="connected", source="demo", demo=True, protocol="ISO 15765-4 (CAN 11/500) [DEMO]",
                          elm_version="ELM327 v2.1 [DEMO]", since=time.time(), multi_pid=True)
            s.vehicle = {"profile": car_profile(), "vin": "DEMO0000000000000", "maker": "Demo", "calibration_ids": ["DEMO-CAL"], "cvn": ["00000000"], "ecu_names": ["ECM - EngineControl"]}
            s.status = {"mil": False, "dtc_count": 0, "ignition": "spark", "tests": [{"name": n, "available": True, "complete": True} for n in ["Misfire", "Fuel System", "Components", "Catalyst", "Oxygen Sensor"]]}
            s.supported = [{"mode": 1, "pid": i, "name": n, "desc": d, "polled": True, "note": ""} for i, (n, d, u) in enumerate(spec)]
            s.dtcs = {"stored": [], "pending": [], "permanent": [], "updated": time.time()}
        t0 = time.time()
        while True:
            t = time.time() - t0
            rpm = 750 + 1800 * max(0, math.sin(t / 9)) + random.uniform(-20, 20)
            vals = {"RPM": rpm, "SPEED": max(0, 60 * math.sin(t / 9)), "ENGINE_LOAD": 20 + rpm / 120, "COOLANT_TEMP": 88,
                    "INTAKE_TEMP": 45, "INTAKE_PRESSURE": 35 + rpm / 60, "TIMING_ADVANCE": 8 + 10 * math.sin(t / 3),
                    "THROTTLE_POS": 13 + 30 * max(0, math.sin(t / 9)), "SHORT_FUEL_TRIM_1": random.uniform(-4, 4),
                    "LONG_FUEL_TRIM_1": 3.1, "O2_B1S1": 0.45 + 0.4 * math.sin(t * 2.5), "O2_B1S2": 0.7,
                    "CONTROL_MODULE_VOLTAGE": 14.1 + random.uniform(-0.05, 0.05), "BAROMETRIC_PRESSURE": 100,
                    "COMMANDED_EQUIV_RATIO": 1.0, "CATALYST_TEMP_B1S1": 600, "AMBIANT_AIR_TEMP": 31, "RUN_TIME": t,
                    "DISTANCE_SINCE_DTC_CLEAR": 1863}
            with s.lock:
                for n, d, u in spec:
                    s.set_live(n, d, round(vals[n], 3), u)
                s.set_live("ELM_VOLTAGE", "Battery voltage at OBD port (adapter)", 14.2, "V", mode=0)
                s.conn.update(pids_per_sec=24.0, cycle_ms=410, voltage=14.2)
                s.add_raw("010C0D04050B0E", ["7E8 10 0E 41 0C 0A FA 0D 00", "7E8 21 04 3B 05 7C 0B 28 0E"], 135)
                s.bump()
            while not self.jobs.empty():
                self.jobs.get_nowait()
            time.sleep(0.4)


# ----------------------------------------------------------------------------------------
class ReplayReader(CarReader):
    """Plays back a recorded drive (sample_data/<car>/drive.csv) through the whole dashboard, with the
    car's saved health, computer scan, hidden-block samples and trips. No car or adapter needed."""

    def __init__(self, state, root, skip=0):
        super().__init__(state, "replay://", 0)
        self.root, self.skip = root, skip

    def run(self):
        import obd
        self.obd = obd
        logging.getLogger("obd").setLevel(logging.CRITICAL)
        while True:
            self.switch_pending = False
            try:
                self.load_car()
                self.play()
            except Exception:
                traceback.print_exc()
                time.sleep(3)

    def load_car(self):
        import re_learn
        d = cars.car_dir()
        with self.s.lock:
            self.s.reset_for_car()
        h = json.load(open(os.path.join(d, "health.json")))
        mods = json.load(open(os.path.join(d, "modules.json")))
        with self.s.lock:
            v = h.get("vehicle", {})
            v["profile"] = car_profile()
            self.s.vehicle = v
            self.s.status, self.s.status_cycle = h.get("status"), h.get("status_cycle")
            self.s.dtcs = dict(h.get("dtcs", {"stored": [], "pending": [], "permanent": []}), updated=time.time())
            self.s.mode06 = h.get("mode06", [])
            self.s.mode06_updated = time.time() if self.s.mode06 else None
            self.s.info, self.s.supported = h.get("info", {}), h.get("supported", [])
            self.s.modules.update(list=mods, updated=time.time(), progress=None)
            self.s.conn.update(status="connected", source="replay", replay=True, demo=False, error=None,
                               protocol=h.get("protocol"), elm_version="ELM327 v2.1 (recorded)",
                               since=time.time(), multi_pid=True, csv=None, bridge=None)
            self.s.bump()
        self.mfr_targets = [(m["address"], r["request"]) for m in mods if m["present"] for r in m["reads"]]
        self.learner = re_learn.Learner(path=os.path.join(d, "re_samples.jsonl"))
        self.seed = h.get("seed", {})
        self.block_rows = sorted((x[0], (h_, r_), x[1]) for (h_, r_), dq in self.learner.samples.items() for x in dq)
        self.replay_latest = {}
        self.run_learner()

    def latest_blocks(self):
        return self.replay_latest or super().latest_blocks()

    def play(self):
        obd = self.obd
        path = os.path.join(cars.car_dir(), "drive.csv")
        rows = []
        with open(path) as f:
            r = csv.reader(f)
            next(r, None)
            for ts, name, val, unit in r:
                if name.startswith(("HY_", "DEC_")):
                    continue
                rows.append((float(ts), name, float(val), unit))
        rows.sort()
        # readings that the recording doesn't repeat often (or at all) start from their last known value
        before = {}
        for ts, name, val, unit in rows:
            if ts < rows[0][0] + self.skip:
                before[name] = (val, unit)
        for name, (val, unit) in self.seed.items():
            before.setdefault(name, (val, unit))
        later = {name for ts, name, val, unit in rows if ts >= rows[0][0] + self.skip}
        # fast-changing values the recording stops updating would show a stale idle value: leave them out
        before = {k: v for k, v in before.items() if k not in FAST or k in later}
        with self.s.lock:
            for name, (val, unit) in before.items():
                c = obd.commands[name] if obd.commands.has_name(name) else None
                self.s.set_live(name, c.desc if c else "Battery voltage at OBD port (adapter)" if name == "ELM_VOLTAGE" else name,
                                val, unit, mode=c.mode if c else 0, pid=c.pid if c else None)
            self.s.bump()
        first = True
        while not self.switch_pending:
            start = rows[0][0] + (self.skip if first else 0)       # --replay-skip jumps into the drive once
            first = False
            t0, wall0 = start, time.time()
            last_bump, last_dec, n, bi = 0, 0, 0, 0
            for ts, name, val, unit in rows:
                if ts < start:
                    continue
                while bi < len(self.block_rows) and self.block_rows[bi][0] <= ts:   # hidden blocks on the same clock
                    _, key, raw = self.block_rows[bi]
                    if self.block_rows[bi][0] >= start - 30:
                        self.replay_latest[key] = raw
                    bi += 1
                if time.time() - last_dec > 2 and self.replay_latest:
                    self.run_learner(analyse=False)
                    last_dec = time.time()
                while time.time() < wall0 + (ts - t0):
                    if self.switch_pending:
                        return
                    self.handle_jobs()
                    time.sleep(0.02)
                if obd.commands.has_name(name):
                    c = obd.commands[name]
                    desc, mode, pid = c.desc, c.mode, c.pid
                elif name == "O2_S1_LAMBDA":
                    desc, mode, pid = "Measured air-fuel ratio (wideband lambda)", 1, 0x34
                else:
                    desc, mode, pid = name.replace("_", " ").title(), 0, None
                with self.s.lock:
                    self.s.set_live(name, desc, val, unit, mode=mode, pid=pid)
                    if name == "ELM_VOLTAGE":
                        self.s.conn["voltage"] = val
                    n += 1
                    if time.time() - last_bump > 0.25:
                        self.s.trip.update(self.s.live)
                        el = max(time.time() - wall0, 1)
                        self.s.conn.update(status="connected", pids_per_sec=round(n / el, 1),
                                           focus_until=self.focus_until if time.time() < self.focus_until else None)
                        self.s.bump()
                        last_bump = time.time()

    def handle_jobs(self):
        while not self.jobs.empty():
            job, arg = self.jobs.get_nowait()
            if job == "refresh":
                with self.s.lock:
                    self.s.dtcs["updated"] = time.time()
                    self.s.bump()


STATE = State()
READER = None


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass

    def send(self, code, body, ctype="application/json"):
        b = body.encode() if isinstance(body, str) else body
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(b)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(b)

    def do_GET(self):
        u = urlparse(self.path)
        qs = parse_qs(u.query)
        if u.path in ("/", "/index.html"):
            with open(os.path.join(HERE, "dashboard.html"), "rb") as f:
                return self.send(200, f.read(), "text/html; charset=utf-8")
        if u.path == "/api/state":
            return self.send(200, STATE.snapshot(history="history" in qs))
        if u.path == "/api/raw":
            since = int(qs.get("since", ["0"])[0])
            with STATE.lock:
                rows = [r for r in STATE.raw if r["n"] > since]
            return self.send(200, json.dumps(rows))
        if u.path == "/api/stream":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            last = -1
            try:
                while True:
                    if STATE.version != last:
                        last = STATE.version
                        self.wfile.write(b"data: " + STATE.snapshot().encode() + b"\n\n")
                        self.wfile.flush()
                    time.sleep(0.25)
            except (BrokenPipeError, ConnectionResetError, OSError):
                return
        if u.path == "/api/cars":
            return self.send(200, json.dumps(cars.listing()))
        if u.path == "/api/trips":
            import trips
            return self.send(200, json.dumps(trips.index()))
        if u.path.startswith("/api/trips/"):
            import trips
            try:
                return self.send(200, json.dumps(trips.load(u.path.rsplit("/", 1)[1])))
            except Exception:
                return self.send(404, '{"error":"no such trip"}')
        if u.path == "/api/csv":
            with STATE.lock:
                p = STATE.conn.get("csv")
            if p and os.path.exists(os.path.join(HERE, p)):
                with open(os.path.join(HERE, p), "rb") as f:
                    data = f.read()
                self.send_response(200)
                self.send_header("Content-Type", "text/csv")
                self.send_header("Content-Disposition", f'attachment; filename="{os.path.basename(p)}"')
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                return self.wfile.write(data)
            return self.send(404, '{"error":"no log yet"}')
        self.send(404, '{"error":"not found"}')

    def do_POST(self):
        u = urlparse(self.path)
        qs = parse_qs(u.query)
        if u.path == "/api/refresh":
            READER.jobs.put(("refresh", None))
            return self.send(200, '{"ok":true}')
        if u.path in ("/api/trip/reset", "/api/trip/start"):
            with STATE.lock:
                STATE.trip.reset(active=True)
            READER.focus_until = time.time() + 60 * 60      # decode hidden data for the whole trip (max 1 h)
            return self.send(200, '{"ok":true}')
        if u.path == "/api/watch":
            READER.jobs.put(("watch", max(20, min(300, int(qs.get("seconds", ["120"])[0])))))
            return self.send(200, '{"ok":true}')
        if u.path == "/api/watch/label":
            with STATE.lock:
                STATE.watch.update(label=qs.get("name", [""])[0][:40] or None, label_t=time.time())
                STATE.bump()
            return self.send(200, '{"ok":true}')
        if u.path == "/api/watch/stop":
            READER.watch_stop = True
            return self.send(200, '{"ok":true}')
        if u.path == "/api/wake":
            READER.wake_now = True
            return self.send(200, '{"ok":true}')
        if u.path == "/api/car":
            cid = qs.get("id", [""])[0]
            try:
                READER.switch_car(cid)
            except ValueError as e:
                return self.send(400, json.dumps({"error": str(e)}))
            return self.send(200, '{"ok":true}')
        if u.path == "/api/fill/start":
            with STATE.lock:
                STATE.trip.fill_start()
            return self.send(200, '{"ok":true}')
        if u.path == "/api/fill/done":
            try:
                litres = float(qs.get("litres", ["0"])[0])
            except ValueError:
                litres = 0
            with STATE.lock:
                res = STATE.trip.fill_done(litres)
            return self.send(200, json.dumps(res))
        if u.path == "/api/trip/end":
            with STATE.lock:
                STATE.trip.end()
            READER.focus_until = 0
            return self.send(200, '{"ok":true}')
        if u.path == "/api/focus":
            mins = max(0, min(30, int(qs.get("minutes", ["10"])[0])))
            READER.focus_until = time.time() + mins * 60 if mins else 0
            return self.send(200, '{"ok":true}')
        if u.path == "/api/modules":
            READER.jobs.put(("modules", None))
            return self.send(200, '{"ok":true}')
        if u.path == "/api/sniff":
            secs = max(2, min(60, int(qs.get("seconds", ["10"])[0])))
            READER.jobs.put(("sniff", secs))
            return self.send(200, '{"ok":true}')
        self.send(404, '{"error":"not found"}')


def main():
    global READER
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="socket://127.0.0.1:35000", help="adapter: socket://host:port (bridge) or /dev/cu.*")
    ap.add_argument("--baud", type=int, default=38400)
    ap.add_argument("--http", type=int, default=8327)
    ap.add_argument("--demo", action="store_true", help="fake data, no car needed")
    ap.add_argument("--replay", nargs="?", const=os.path.join(HERE, "sample_data"), default=None,
                    help="play back recorded drives from this folder (default: sample_data/)")
    ap.add_argument("--replay-skip", type=float, default=0, help="seconds to skip into the recorded drive")
    a = ap.parse_args()
    if a.replay:
        import shutil
        work = os.path.join(LOGDIR, "replay")             # play from a copy so the samples never change
        shutil.rmtree(work, ignore_errors=True)
        shutil.copytree(a.replay, work)
        os.environ["OBD_DATA_ROOT"] = work
        if cars.active_id() not in [d for d in os.listdir(work) if os.path.isdir(os.path.join(work, d))]:
            cars.set_active(sorted(os.listdir(work))[0])
        with STATE.lock:
            STATE.reset_for_car()
        READER = ReplayReader(STATE, work, a.replay_skip)
    else:
        READER = DemoReader(STATE) if a.demo else CarReader(STATE, a.port, a.baud)
    READER.start()
    srv = ThreadingHTTPServer(("127.0.0.1", a.http), Handler)
    srv.daemon_threads = True
    print(f"OBD Live dashboard: http://127.0.0.1:{a.http}  (source: {'replay of ' + a.replay if a.replay else 'demo' if a.demo else a.port})", flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
