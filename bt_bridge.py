"""
ELM327 Bluetooth -> TCP bridge for macOS.

Why this exists
  macOS (Apple Silicon, Sonoma and later) has a long-standing bug with classic
  Bluetooth serial ports: once a program closes /dev/cu.<device>, reopening it
  "succeeds" but no bytes ever flow again until bluetoothd is restarted or the
  device is re-paired. Cheap ELM327 adapters hit this constantly.

What it does
  * Talks to the adapter's RFCOMM channel directly through IOBluetooth
    (no /dev/cu driver involved).
  * Keeps that channel open for as long as the bridge runs.
  * If the channel closes, or the adapter stops answering, it drops the
    Bluetooth baseband link itself and re-opens it (no root, no re-pairing).
  * Serves the raw byte stream on 127.0.0.1:35000, so any OBD software can use
    it (python-obd: obd.OBD("socket://127.0.0.1:35000")). Clients may connect
    and disconnect as often as they like; the Bluetooth link is unaffected.

It must run inside "OBD Bridge.app" so macOS grants it Bluetooth permission.
"""
import json, os, queue, signal, socket, sys, threading, time, traceback

import objc
from Foundation import NSObject, NSRunLoop, NSDate
import IOBluetooth

HERE = os.path.dirname(os.path.abspath(__file__))
CONF = {
    "address": os.environ.get("OBD_BT_ADDR", ""),      # e.g. 01-23-45-67-89-BA; blank = find by name
    "name": os.environ.get("OBD_BT_NAME", "OBDII"),
    "tcp_port": int(os.environ.get("OBD_TCP_PORT", "35000")),
}
STATUS_FILE = os.path.join(HERE, "logs", "bridge_status.json")
STALL_SECONDS = 6.0      # sent bytes, nothing back for this long -> link is stuck
os.makedirs(os.path.join(HERE, "logs"), exist_ok=True)


def log(msg):
    print(time.strftime("%H:%M:%S"), msg, flush=True)


def pump(seconds):
    """Run the Cocoa run loop so IOBluetooth can deliver callbacks."""
    end = time.time() + seconds
    while True:
        left = end - time.time()
        if left <= 0:
            break
        NSRunLoop.currentRunLoop().runUntilDate_(NSDate.dateWithTimeIntervalSinceNow_(min(left, 0.02)))


class ChannelDelegate(NSObject):
    def initWithBridge_(self, bridge):
        self = objc.super(ChannelDelegate, self).init()
        self.bridge = bridge
        return self

    def rfcommChannelData_data_length_(self, channel, data, length):
        if isinstance(data, (bytes, bytearray)):
            chunk = bytes(data[:length])
        elif hasattr(data, "as_buffer"):
            chunk = bytes(data.as_buffer(length))
        else:
            chunk = bytes(data)[:length]
        self.bridge.on_rx(chunk)

    def rfcommChannelClosed_(self, channel):
        self.bridge.on_closed()


class Bridge:
    def __init__(self):
        self.dev = None
        self.ch = None
        self.chan_id = None
        self.delegate = ChannelDelegate.alloc().initWithBridge_(self)
        self.tx = queue.Queue()
        self.client = None
        self.client_lock = threading.Lock()
        self.probe_buf = None          # when not None, rx goes here (link self-test)
        self.last_tx = 0.0
        self.last_rx = 0.0
        self.pending = False           # bytes sent, waiting for any reply
        self.backoff = 1.0
        self.reset_baseband = False
        self.st = {
            "state": "starting", "device": None, "address": None, "rfcomm_channel": None,
            "link_generation": 0, "reconnects": 0, "stalls": 0, "client": None,
            "bytes_tx": 0, "bytes_rx": 0, "last_error": None, "since": None,
            "pid": os.getpid(), "tcp_port": CONF["tcp_port"], "updated": time.time(),
        }
        self.write_status()

    # ---------- status ----------
    def write_status(self, **kw):
        self.st.update(kw)
        self.st["updated"] = time.time()
        tmp = STATUS_FILE + ".tmp"
        with open(tmp, "w") as f:
            json.dump(self.st, f)
        os.replace(tmp, STATUS_FILE)

    # ---------- bluetooth ----------
    def find_device(self):
        if CONF["address"]:
            d = IOBluetooth.IOBluetoothDevice.deviceWithAddressString_(CONF["address"])
            if d is not None:
                return d
        for d in IOBluetooth.IOBluetoothDevice.pairedDevices() or []:
            if (d.name() or "") == CONF["name"] or (d.name() or "").upper().startswith(("OBD", "ELM", "V-LINK", "VLINK")):
                return d
        raise RuntimeError(f"No paired Bluetooth device named {CONF['name']!r}. Pair the adapter in System Settings first.")

    def open_link(self):
        self.write_status(state="connecting")
        if self.dev is None:
            self.dev = self.find_device()
            self.write_status(device=self.dev.name(), address=self.dev.addressString())
        dev = self.dev

        if self.reset_baseband and dev.isConnected():
            log("resetting Bluetooth baseband link")
            dev.closeConnection()
            pump(2.0)
        self.reset_baseband = False

        if not dev.isConnected():
            r = dev.openConnection()
            if r != 0:
                raise RuntimeError(f"Bluetooth openConnection failed (IOReturn {r}). Is the adapter powered and in range?")
            pump(0.3)

        if self.chan_id is None:
            spp = IOBluetooth.IOBluetoothSDPUUID.uuid16_(0x1101)
            rec = dev.getServiceRecordForUUID_(spp)
            if rec is None:
                dev.performSDPQuery_(None)
                pump(4.0)
                rec = dev.getServiceRecordForUUID_(spp)
            if rec is None:
                raise RuntimeError("Adapter does not advertise a Serial Port (SPP) service")
            st, cid = rec.getRFCOMMChannelID_(None)
            if st != 0:
                raise RuntimeError(f"Could not read RFCOMM channel id ({st})")
            self.chan_id = cid

        st, ch = dev.openRFCOMMChannelSync_withChannelID_delegate_(None, self.chan_id, self.delegate)
        if st != 0 or ch is None:
            raise RuntimeError(f"openRFCOMMChannel failed (IOReturn {st})")
        self.ch = ch
        pump(0.3)

        # Self-test: make sure bytes really flow both ways (the macOS bug is a silent channel).
        reply = self.probe(b"ATI\r", 4.0)
        if b">" not in reply:
            self.close_channel()
            self.reset_baseband = True
            raise RuntimeError("Channel opened but adapter is silent (stale macOS RFCOMM state); will reset link")
        ident = reply.replace(b"\r", b" ").replace(b">", b"").strip().decode(errors="replace")
        self.backoff = 1.0
        self.fails = 0
        gen = self.st["link_generation"] + 1
        log(f"link up: {dev.name()} {dev.addressString()} rfcomm ch {self.chan_id} -> {ident!r} (generation {gen})")
        self.write_status(state="up", rfcomm_channel=self.chan_id, link_generation=gen,
                          since=time.time(), last_error=None, adapter=ident)

    def probe(self, data, timeout):
        self.probe_buf = bytearray()
        self.write_raw(data)
        end = time.time() + timeout
        while time.time() < end and b">" not in self.probe_buf:
            pump(0.02)
        out, self.probe_buf = bytes(self.probe_buf), None
        self.pending = False
        return out

    def write_raw(self, data):
        r = self.ch.writeSync_length_(data, len(data))
        if r != 0:
            raise RuntimeError(f"RFCOMM write failed (IOReturn {r})")
        now = time.time()
        if not self.pending:
            self.pending = True
            self.last_tx = now
        self.st["bytes_tx"] += len(data)

    def drop_client(self):
        """Tell the TCP client the link is gone so it fails fast instead of waiting on a dead adapter."""
        with self.client_lock:
            c, self.client = self.client, None
        if c is not None:
            try:
                c.shutdown(socket.SHUT_RDWR)
                c.close()
            except OSError:
                pass
            self.st["client"] = None

    def close_channel(self):
        self.drop_client()
        ch, self.ch = self.ch, None
        if ch is not None:
            try:
                ch.closeChannel()
                pump(0.5)
            except Exception:
                pass

    def on_rx(self, chunk):
        self.last_rx = time.time()
        self.pending = False
        self.st["bytes_rx"] += len(chunk)
        if self.probe_buf is not None:
            self.probe_buf += chunk
            return
        with self.client_lock:
            c = self.client
        if c is not None:
            try:
                c.sendall(chunk)
            except OSError:
                pass

    def on_closed(self):
        if self.ch is not None:
            log("RFCOMM channel closed by adapter/OS")
            self.ch = None
            self.drop_client()
            self.write_status(state="down", last_error="channel closed")

    # ---------- tcp ----------
    def tcp_server(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("127.0.0.1", CONF["tcp_port"]))
        srv.listen(4)
        log(f"serving ELM327 on tcp://127.0.0.1:{CONF['tcp_port']}")
        while True:
            conn, addr = srv.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with self.client_lock:
                old, self.client = self.client, conn
            if old is not None:
                try:
                    old.close()
                except OSError:
                    pass
            self.st["client"] = f"{addr[0]}:{addr[1]}"
            log(f"client connected {addr}")
            threading.Thread(target=self.client_reader, args=(conn,), daemon=True).start()

    def client_reader(self, conn):
        try:
            while True:
                data = conn.recv(4096)
                if not data:
                    break
                self.tx.put(data)
        except OSError:
            pass
        with self.client_lock:
            if self.client is conn:
                self.client = None
                self.st["client"] = None
        log("client disconnected")

    # ---------- main loop (must be the main thread: IOBluetooth callbacks) ----------
    def run(self):
        threading.Thread(target=self.tcp_server, daemon=True).start()
        last_status = 0
        while True:
            if self.ch is None:
                try:
                    if self.st["state"] == "up":
                        self.write_status(state="down")
                    self.open_link()
                    if self.st["link_generation"] > 1:
                        self.write_status(reconnects=self.st["reconnects"] + 1)
                except Exception as e:
                    log(f"connect failed: {e}")
                    # don't retry the same stale state: drop the link, and every 3rd failure forget the device too
                    self.fails = getattr(self, "fails", 0) + 1
                    self.reset_baseband = True
                    if self.fails % 3 == 0:
                        self.dev, self.chan_id = None, None
                    self.write_status(state="down", last_error=str(e))
                    # drop queued client bytes; they were meant for a dead link
                    while not self.tx.empty():
                        self.tx.get_nowait()
                    pump(self.backoff)
                    self.backoff = min(self.backoff * 2, 15.0)
                    continue

            pump(0.01)
            try:
                while not self.tx.empty() and self.ch is not None:
                    self.write_raw(self.tx.get_nowait())
            except Exception as e:
                log(f"write error: {e}")
                self.close_channel()
                self.reset_baseband = True
                continue

            if self.pending and time.time() - self.last_tx > STALL_SECONDS:
                log("adapter stopped answering; recycling Bluetooth link")
                self.write_status(stalls=self.st["stalls"] + 1, last_error="adapter stopped answering")
                self.pending = False
                self.close_channel()
                self.reset_baseband = True
                continue

            if time.time() - last_status > 1:
                self.write_status()
                last_status = time.time()

    def shutdown(self, *_):
        log("shutting down")
        self.write_status(state="stopped")
        self.close_channel()
        os._exit(0)


if __name__ == "__main__":
    b = Bridge()
    signal.signal(signal.SIGTERM, b.shutdown)
    signal.signal(signal.SIGINT, b.shutdown)
    try:
        b.run()
    except Exception:
        traceback.print_exc()
        b.write_status(state="crashed", last_error=traceback.format_exc(limit=2))
        raise
