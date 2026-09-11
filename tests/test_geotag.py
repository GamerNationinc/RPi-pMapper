import sys, types, math, io
# stub hardware-only modules
for name in ("picamera2", "libcamera", "pymavlink", "pymavlink.mavutil", "simplejpeg"):
    sys.modules[name] = types.ModuleType(name)
sys.modules["picamera2"].Picamera2 = object
sys.modules["libcamera"].controls = types.SimpleNamespace()
sys.modules["pymavlink"].mavutil = sys.modules["pymavlink.mavutil"]
sys.modules["simplejpeg"] = None and None
import pathlib; sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import importlib; sys.modules.pop("simplejpeg")
import nebula_cam as nc
from datetime import datetime, timezone
from PIL import Image

# --- StateBuffer interpolation ---
b = nc.StateBuffer()
for i in range(10):
    t = 100.0 + i * 0.1
    b.add_pos(t, 43.0 + i*1e-5, -80.0 + i*1e-5, 300.0 + i, 50.0 + i)
    b.add_gps(t, 43.0 + i*1e-5, -80.0 + i*1e-5, 264.0 + i, 3, 14, 1.2)
    b.add_att(t, 0.0, 0.0, math.radians(350.0 + i*2))   # yaw crosses 360
f = b.sample(100.45)
assert abs(f["lat"] - 43.000045) < 1e-9, f["lat"]
assert abs(f["alt_ellip"] - 268.5) < 1e-9
assert abs(f["yaw"] - 359.0) < 1e-6, f["yaw"]
f2 = b.sample(100.75); assert abs(f2["yaw"] - 5.0) < 1e-6, f2["yaw"]
assert b.sample(999.0)["lat"] == 43.00009   # clamps to last
print("StateBuffer OK", f)

# --- EXIF + XMP into a real JPEG ---
img = Image.new("RGB", (64, 48), (120, 200, 30))
blob = io.BytesIO(); img.save(blob, "JPEG", quality=90); data = blob.getvalue()
meta = {"ExposureTime": 800, "AnalogueGain": 2.0}
utc = datetime(2026, 9, 11, 17, 5, 7, 250000, tzinfo=timezone.utc)
sink = io.BytesIO()
nc.piexif.insert(nc.build_exif(f, utc, meta), data, sink)
out = nc.insert_xmp(sink.getvalue(), nc.build_xmp(f))
Image.open(io.BytesIO(out)).load()          # still decodes
ex = nc.piexif.load(out)
gps = ex["GPS"]
lat = gps[nc.piexif.GPSIFD.GPSLatitude]; print("EXIF lat", lat, gps[nc.piexif.GPSIFD.GPSLatitudeRef])
assert gps[nc.piexif.GPSIFD.GPSLongitudeRef] == b"W"
assert gps[nc.piexif.GPSIFD.GPSAltitude] == (268500, 1000)
assert ex["Exif"][nc.piexif.ExifIFD.ExposureTime] == (1, 1250)
assert b"Camera:Yaw=\"359.00\"" in out and b"drone-dji:GimbalPitchDegree=\"-90.00\"" in out
assert out.count(b"http://ns.adobe.com/xap/1.0/\x00") == 1
print("EXIF/XMP OK", len(data), "->", len(out), "bytes")
