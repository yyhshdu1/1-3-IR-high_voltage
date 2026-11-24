"""Read-only driver for the GQ GMC-800 Geiger counter (GQ-RFC1201 serial protocol).

Only measurement/identification *query* commands are ever sent. No command that
changes settings, time, power, or configuration is implemented, and `_query`
refuses anything not in the whitelist below.

Verified on a real unit: USB-SERIAL CH340 on COM8, 115200 8N1,
firmware "GMC-800Re1.13". On this firmware <GETCPM>> and <GETCPS>> answer with
4 bytes (big-endian uint32); older models answer with 2 bytes (14-bit value).
Both are handled. <GETVOLT>> gets no answer from this unit, so it is not used.

Usage (library):
    from gmc800 import GMC800
    with GMC800("COM8") as g:
        print(g.version, g.serial_number)
        print(g.cpm(), g.cps())

Usage (command line):
    python gmc800.py                    # one reading on COM8
    python gmc800.py -i 1               # print CPS/CPM every 1 s
    python gmc800.py -i 1 --csv out.csv # also append to a CSV file
"""

import argparse
import csv
import struct
import sys
import threading
import time
from datetime import datetime

import serial

# Only these (read-only) commands may be sent to the device.
_ALLOWED = {b"<GETVER>>", b"<GETSERIAL>>", b"<GETCPM>>", b"<GETCPS>>", b"<GETCFG>>"}

# Calibration table inside the 512-byte <GETCFG>> block, as found on the GMC-800
# (fw 1.13): 6 x uint32 CPM (big-endian) followed by 6 x float32 uSv/h (big-endian).
_CAL_OFFSET = 73
_CAL_POINTS = 6


class GMC800Error(RuntimeError):
    pass


class GMC800:
    def __init__(self, port="COM8", baudrate=115200, timeout=1.0, usv_per_cpm=None):
        """
        usv_per_cpm: optional manual override (uSv/h per CPM). By default the
        device's own calibration table (read via <GETCFG>>) is used.
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.usv_per_cpm = usv_per_cpm
        self._ser = None
        self._cal = None
        self._lock = threading.Lock()

    # ------------------------------------------------------------ connection
    def open(self):
        if self._ser is None or not self._ser.is_open:
            self._ser = serial.Serial(
                self.port, self.baudrate, bytesize=8, parity="N", stopbits=1,
                timeout=self.timeout, write_timeout=self.timeout,
            )
            time.sleep(0.1)
            self._ser.reset_input_buffer()
        return self

    def close(self):
        if self._ser is not None:
            self._ser.close()
            self._ser = None

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()

    # --------------------------------------------------------------- low level
    def _query(self, cmd: bytes, nbytes: int, min_bytes=None) -> bytes:
        """Send a whitelisted command and read up to `nbytes` of reply."""
        if cmd not in _ALLOWED:
            raise GMC800Error(f"command {cmd!r} is not allowed (driver is read-only)")
        if self._ser is None or not self._ser.is_open:
            raise GMC800Error("port is not open")
        min_bytes = nbytes if min_bytes is None else min_bytes
        with self._lock:
            self._ser.reset_input_buffer()
            self._ser.write(cmd)
            self._ser.flush()
            data = self._ser.read(nbytes)  # returns early only when nbytes arrive
        if len(data) < min_bytes:
            raise GMC800Error(
                f"{cmd.decode()}: expected {min_bytes}-{nbytes} bytes, got {len(data)}: {data!r}"
            )
        return data

    def _query_count(self, cmd: bytes) -> int:
        # 4 bytes on GMC-800 (uint32), 2 bytes on older models (14-bit).
        data = self._query(cmd, 4, min_bytes=2)
        if len(data) == 4:
            return int.from_bytes(data, "big")
        if len(data) == 2:
            return int.from_bytes(data, "big") & 0x3FFF
        raise GMC800Error(f"{cmd.decode()}: unexpected reply length {len(data)}: {data!r}")

    # ------------------------------------------------------------ public API
    @property
    def version(self) -> str:
        """Model + firmware string, e.g. 'GMC-800Re1.13'."""
        data = self._query(b"<GETVER>>", 14, min_bytes=1)  # length varies by model
        return data.decode("ascii", errors="replace").strip()

    @property
    def serial_number(self) -> str:
        """7-byte serial number as a hex string."""
        return self._query(b"<GETSERIAL>>", 7).hex().upper()

    def cpm(self) -> int:
        """Counts per minute (device's rolling value)."""
        return self._query_count(b"<GETCPM>>")

    def cps(self) -> int:
        """Counts in the last second."""
        return self._query_count(b"<GETCPS>>")

    @property
    def calibration(self):
        """Device calibration table [(cpm, uSv/h), ...], read once from <GETCFG>>."""
        if self._cal is None:
            cfg = self._query(b"<GETCFG>>", 512, min_bytes=256)
            n = _CAL_POINTS
            cpms = [int.from_bytes(cfg[_CAL_OFFSET + 4 * i:_CAL_OFFSET + 4 * i + 4], "big")
                    for i in range(n)]
            o = _CAL_OFFSET + 4 * n
            usvs = [struct.unpack(">f", cfg[o + 4 * i:o + 4 * i + 4])[0] for i in range(n)]
            ok = (all(c > 0 for c in cpms) and all(u > 0 and u == u for u in usvs)
                  and cpms == sorted(cpms) and usvs == sorted(usvs))
            if not ok:
                raise GMC800Error(f"calibration table not recognised: {cpms} {usvs}")
            self._cal = list(zip(cpms, usvs))
        return self._cal

    def cpm_to_usv_h(self, cpm: float) -> float:
        """Convert CPM to uSv/h: `usv_per_cpm` if given, else the device's own
        calibration table (linear through the origin below the first point,
        piecewise-linear between points)."""
        if self.usv_per_cpm is not None:
            return cpm * self.usv_per_cpm
        pts = [(0, 0.0)] + self.calibration
        for (c0, u0), (c1, u1) in zip(pts, pts[1:]):
            if cpm <= c1:
                return u0 + (u1 - u0) * (cpm - c0) / (c1 - c0)
        (c0, u0), (c1, u1) = pts[-2], pts[-1]  # extrapolate beyond last point
        return u1 + (u1 - u0) * (cpm - c1) / (c1 - c0)

    def usv_h(self) -> float:
        """Dose rate in uSv/h, from the device's rolling CPM."""
        return self.cpm_to_usv_h(self.cpm())

    def read(self) -> dict:
        """One timestamped reading: {'time', 'cps', 'cpm', 'usv_h'}."""
        cps = self.cps()
        cpm = self.cpm()
        return {"time": datetime.now().isoformat(timespec="milliseconds"),
                "cps": cps, "cpm": cpm, "usv_h": self.cpm_to_usv_h(cpm)}


# ------------------------------------------------------------------------ CLI
def main(argv=None):
    ap = argparse.ArgumentParser(description="Read-only GMC-800 Geiger counter reader")
    ap.add_argument("-p", "--port", default="COM8")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    ap.add_argument("-i", "--interval", type=float, default=None,
                    help="poll every N seconds (default: single reading)")
    ap.add_argument("--csv", help="append readings to this CSV file")
    args = ap.parse_args(argv)

    with GMC800(args.port, args.baud) as g:
        print(f"# {g.version}  serial {g.serial_number}  on {args.port}")
        writer = fh = None
        if args.csv:
            fh = open(args.csv, "a", newline="")
            writer = csv.DictWriter(fh, fieldnames=["time", "cps", "cpm", "usv_h"])
            if fh.tell() == 0:
                writer.writeheader()
        try:
            while True:
                t0 = time.monotonic()
                r = g.read()
                print(f"{r['time']}  CPS={r['cps']:<5d} CPM={r['cpm']:<6d} {r['usv_h']:.4f} uSv/h")
                if writer:
                    writer.writerow(r)
                    fh.flush()
                if args.interval is None:
                    break
                time.sleep(max(0.0, args.interval - (time.monotonic() - t0)))
        except KeyboardInterrupt:
            pass
        finally:
            if fh:
                fh.close()


if __name__ == "__main__":
    sys.exit(main())
