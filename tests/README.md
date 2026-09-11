# tests

Run from anywhere, no Pi needed. The hardware modules (`picamera2`,
`libcamera`, `pymavlink`, `simplejpeg`) are stubbed in `sys.modules`; only
`piexif`, `pillow` and `numpy` are required (`pip install piexif pillow numpy`).

    python tests/test_geotag.py      # StateBuffer interpolation, EXIF, XMP, JPEG splice
    python tests/test_pipeline.py    # queues, ordered close, drops, TX lock, READY w/o FC,
                                     # UDP bridge, status snapshot/file/http, nebula-top render

Both print `... OK` per case and exit non-zero on the first failure. They
cannot exercise real capture, the serial port, NetworkManager or curses.
