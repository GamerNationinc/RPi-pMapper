"""Drive NebulaCam's queue pipeline, UDP bridge and status stack without hardware."""
import sys, os, types, time, threading, csv, tempfile, socket, json, urllib.request, importlib.util

import pathlib; REPO = str(pathlib.Path(__file__).resolve().parents[1])

# ---- stubs for Pi-only modules -------------------------------------------
class _Consts:
    def __getattr__(self, name):          # any MAV_* constant -> stable int
        return abs(hash(name)) % 1000

mavutil = types.ModuleType("pymavlink.mavutil"); mavutil.mavlink = _Consts()
pym = types.ModuleType("pymavlink"); pym.mavutil = mavutil
sys.modules["pymavlink"] = pym; sys.modules["pymavlink.mavutil"] = mavutil
for n in ("picamera2", "libcamera"):
    sys.modules[n] = types.ModuleType(n)
sys.modules["picamera2"].Picamera2 = object
sys.modules["libcamera"].controls = types.SimpleNamespace()
sys.modules["simplejpeg"] = None          # force the PIL path (import raises -> None)

sys.path.insert(0, REPO)
try:
    import nebula_cam as nc
except ImportError:
    sys.modules.pop("simplejpeg"); import nebula_cam as nc

import numpy as np

# ---- Windows shims (all exist on the Pi) ----------------------------------
nc.boottime = time.monotonic
nc.free_mb = lambda p: 20000.0
os.sync = getattr(os, "sync", lambda: None)
if not hasattr(os, "O_DIRECTORY"):
    os.O_DIRECTORY = 0

# ---- fakes ------------------------------------------------------------------
class FakeRequest:
    def __init__(self):
        self.meta = {"SensorTimestamp": int(time.monotonic() * 1e9),
                     "ExposureTime": 800, "AnalogueGain": 2.0}
    def get_metadata(self): return self.meta
    def make_array(self, _): return np.full((48, 64, 3), 90, np.uint8)
    def release(self): pass

class FakeCam:
    def __init__(self): self.gate = threading.Event(); self.gate.set(); self.camera_controls = {}
    def capture_request(self): self.gate.wait(); return FakeRequest()
    def capture_metadata(self): return FakeRequest().meta
    def set_controls(self, c): pass
    def stop(self): pass

class FakeMsg:
    def __init__(self, name, a): self.name, self.a = name, a
    def get_msgbuf(self): return b"\xfd" + self.name.encode()

class FakeTx:
    """pymavlink-shaped: *_encode() -> msg, send(msg). Asserts the tx lock is
    held and no send overlaps."""
    def __init__(self, svc): self.svc = svc; self.sent = []; self.inflight = 0; self.overlap = 0
    def __getattr__(self, name):
        if not name.endswith("_encode"): raise AttributeError(name)
        return lambda *a: FakeMsg(name, a)
    def send(self, msg):
        assert self.svc.tx_lock.locked(), f"{msg.name} sent without tx_lock"
        self.inflight += 1
        if self.inflight > 1: self.overlap += 1
        time.sleep(0.0005)
        self.sent.append((msg.name, msg.a)); self.inflight -= 1

root = tempfile.mkdtemp(prefix="nebtest_")

def make_svc(root):
    cfg = nc.deep_merge(nc.DEFAULTS, {"storage": {"root": root}, "system": {"status_period_s": 0.5},
                                      "camera": {"ae_settle_s": 0.0},
                                      "telemetry": {"udp_enabled": False},
                                      "status": {"file": os.path.join(root, "status.json"), "http_port": 0}})
    svc = nc.NebulaCam(cfg)
    svc.picam = FakeCam(); svc.cam_ready = True
    svc.mav = types.SimpleNamespace(target_system=1, target_component=1)
    svc.mav.mav = FakeTx(svc)
    svc.linked = True
    now = time.monotonic()
    for i in range(40):                      # 4 s of telemetry either side of now
        t = now - 2.0 + i * 0.1
        svc.buf.add_pos(t, 43.0 + i*1e-5, -80.0, 300.0, 50.0)
        svc.buf.add_gps(t, 43.0 + i*1e-5, -80.0, 264.0, 3, 14, 1.2)
        svc.buf.add_att(t, 0.0, 0.0, 0.0)
    svc.gps_epoch_offset = time.time() - time.monotonic()
    return svc

def start_workers(svc):
    ths = [threading.Thread(target=svc.capture_loop, daemon=True),
           threading.Thread(target=svc.write_loop, daemon=True)]
    for t in ths: t.start()
    return ths

# ---- 1. arm -> 3 triggers -> disarm: ordered close, own filenames -----------
svc = make_svc(root); start_workers(svc)
svc.on_arm(); sdir = svc.session.dir
svc.enqueue_capture(time.monotonic(), 7)      # FC idx 7
svc.enqueue_capture(time.monotonic(), 8)
svc.enqueue_capture(time.monotonic(), None)   # DIGICAM / fallback trigger
svc.on_disarm()                               # must return immediately
assert svc.session is None
deadline = time.monotonic() + 5
while time.monotonic() < deadline and not (svc.capture_q.empty() and svc.write_q.empty()): time.sleep(0.05)
time.sleep(0.3)
files = sorted(p.name for p in sdir.glob("*.jpg"))
assert files == ["NEB_00001.jpg", "NEB_00002.jpg", "NEB_00003.jpg"], files
rows = list(csv.reader(open(sdir / "geotags.csv")))
assert rows[0][-1] == "fc_img_idx" and [r[-1] for r in rows[1:]] == ["7", "8", ""], rows
assert rows[1][2].startswith("43.000") and rows[1][4] == "264.000", rows[1]
assert not list(sdir.glob("*.tmp"))
assert svc.written_bytes > 0 and len(svc.write_times) == 3
print("1 OK: ordered close, filenames from own counter, FC idx in CSV:", files)

# ---- 2. queue full -> counted drop, no crash --------------------------------
svc = make_svc(root); start_workers(svc)
svc.picam.gate.clear()                        # capture blocks -> capture_q fills
svc.on_arm()
for i in range(6): svc.enqueue_capture(time.monotonic(), i)
assert svc.dropped >= 1, svc.dropped
svc.picam.gate.set(); svc.on_disarm(); time.sleep(1.5)
assert svc.capture_q.empty() and svc.write_q.empty()
print("2 OK: dropped =", svc.dropped, "of 6 with capture_q(4)+in-flight")

# ---- 3. concurrent TX is serialised -----------------------------------------
svc = make_svc(root)
def hammer():
    for i in range(150): svc.statustext(6, f"CAM t{i}"); svc.heartbeat()
ths = [threading.Thread(target=hammer) for _ in range(4)]
[t.start() for t in ths]; [t.join() for t in ths]
assert svc.mav.mav.overlap == 0 and len(svc.mav.mav.sent) == 1200, (svc.mav.mav.overlap, len(svc.mav.mav.sent))
print("3 OK: 1200 sends from 4 threads, 0 overlaps")

# ---- 4. READY without an FC link --------------------------------------------
class NoLink:
    target_system = 0
    def wait_heartbeat(self, timeout): time.sleep(0.05)
    def close(self): pass
mavutil.mavlink_connection = lambda *a, **k: NoLink()
svc = make_svc(root); svc.mav = None; svc.linked = False
svc.camera_start = lambda: setattr(svc, "cam_ready", True)
ready = threading.Event(); svc.notify.ready = ready.set
pings = []; svc.notify.ping = lambda: pings.append(1)
th = threading.Thread(target=svc.run, daemon=True); th.start()
assert ready.wait(3.0), "READY never sent while FC absent"
time.sleep(1.2)
assert pings, "watchdog not fed while unlinked"
assert not svc.linked
sf = json.load(open(os.path.join(root, "status.json")))
assert sf["state"] == "NOLINK" and sf["code"] == "S1", sf["state"]
svc.stop.set(); th.join(5); assert not th.is_alive()
print("4 OK: READY + watchdog pings with no FC; status file says S1 NOLINK; clean shutdown")

# ---- 5. UDP bridge: serial->UDP mirror, UDP->serial under lock, peers ------
svc = make_svc(root)
written = []
svc.mav.write = lambda d: written.append(d)
svc.udp = nc.UdpBridge(0, svc.udp_rx)           # ephemeral port
port = svc.udp.sock.getsockname()[1]
gcs = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); gcs.bind(("127.0.0.1", 0)); gcs.settimeout(2)
gcs.sendto(b"\xfd\x00GCS", ("127.0.0.1", port)); time.sleep(0.2)
assert written == [b"\xfd\x00GCS"], written            # GCS -> serial
assert len(svc.udp.live_peers()) == 1
svc.statustext(6, "CAM hello")                            # our TX mirrored to the peer
data, _ = gcs.recvfrom(4096); assert data == b"\xfdstatustext_encode", data
svc.udp.send(b"\xfdFC")                                   # FC -> UDP path
data, _ = gcs.recvfrom(4096); assert data == b"\xfdFC"
print("5 OK: udp bridge both directions, peer learned on port", port)

# ---- 6. snapshot + status file + http page + nebula-top render ------------
svc.hb_last = time.monotonic(); svc.last_fix = (3, 14, 1.2); svc.dropped = 2
svc.write_times.extend([1.5, 1.4])
svc.health = {"throttle_flags": ["UNDERVOLT"], "throttled": "0x50005", "cpu_temp_c": 61.0,
              "mem_free_mb": 210.0, "wifi": {"ssid": "NEBULA-LAPTOP", "rssi_dbm": -61, "ip": "192.168.137.5"}}
snap = svc.snapshot()
assert snap["code"] == "S2" and snap["state"] == "READY", (snap["code"], snap["state"])
assert set(snap["flags"]) == {"DROP", "UNDERVOLT", "SLOW"}, snap["flags"]
json.dumps(snap)                                           # must be serialisable
nc.write_status_file(os.path.join(root, "status.json"), snap)
assert json.load(open(os.path.join(root, "status.json")))["code"] == "S2"
srv = nc.start_status_http(0, lambda: snap); hport = srv.server_address[1]
page = urllib.request.urlopen(f"http://127.0.0.1:{hport}/", timeout=2).read().decode()
js = json.load(urllib.request.urlopen(f"http://127.0.0.1:{hport}/status.json", timeout=2))
assert "<title>nebula-cam" in page and js["flags"] == snap["flags"]
import importlib.machinery; spec = importlib.util.spec_from_loader("ntop", importlib.machinery.SourceFileLoader("ntop", os.path.join(REPO, "nebula-top")))
ntop = importlib.util.module_from_spec(spec); spec.loader.exec_module(ntop)
lines = ntop.render(ntop.fetch(f"http://127.0.0.1:{hport}"), width=100)
text = "\n".join(t for t, _ in lines)
assert "S2 READY" in text and "UNDERVOLT" in text and "NEBULA-LAPTOP" in text and "-61 dBm" in text, text
assert any(lv == "bad" for _, lv in lines)
print("6 OK: snapshot/file/http/nebula-top render")
print(text)
print("ALL PIPELINE TESTS PASSED")
