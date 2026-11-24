"""
xray_guard.py - read a RadEye G20 dose-rate meter and forward it to the HV_control firmware.

The firmware (FW 2.3+) does the actual back-off: this module only has to send
"x <level>" about once per second, level in units of 0.01 uSv/h.  If the reports stop
(RadEye unplugged, serial error, this program crashed) the firmware's own monitor-lost
check (config key "xtimeout") ramps the voltage down, so this module deliberately sends
nothing when it has no valid reading.

Firmware configuration (set once, then "c save"):
    c xthr <level>      alarm level, 0.01 uSv/h units  (e.g. 40 = 0.40 uSv/h; background is ~0.1 uSv/h)
    c xback <units>     back-off per alarm step in host units (3936 ~ 2 kV, 0 = ramp down + latch)
    c xtimeout <s>      monitor-lost timeout (e.g. 5)

RadEye G20 serial settings and parsing are taken from the lab's earlier RadEyeG20.py
(9600 baud, 7 data bits, even parity, 2 stop bits, "@" handshake, "X1" = auto-send mode,
value = 10 nSv/h * first integer after the first two characters of the line).
THE LINE FORMAT HAS NOT BEEN CHECKED ON THIS PC - run  python xray_guard.py --port COM4 --raw
once and compare with what you expect before relying on it.

GMC-800 (Geiger counter, used while the RadEye is lent out):  python xray_guard.py --source gmc800 --port COM8 --raw
prints the identification, the device's calibration table and a few windowed readings.  See Gmc800Source
for why the dose is a 10 s average of the one-second counts, and re-tune c xthr for it.

Fake readings: the RadEye's first sample after a (re)start can be a stale 843-862 uSv/h (seen in
8 files of the Sep-Oct 2025 lab data).  DoseFilter drops the first sample and holds back any
sample above 200 uSv/h until the next one confirms it, so the firmware never sees these values.
"""
import argparse
import threading
import time

import serial


def parse_radeye_line(line):
    """RadEye auto-send line -> dose rate in uSv/h, or None if it cannot be parsed."""
    line = line.strip()
    if len(line) < 3:
        return None
    try:
        nsv_h = 10 * int(line[2:].split()[0])
    except (ValueError, IndexError):
        return None
    return nsv_h / 1000.0


class RadEyeG20:
    """pyserial version of the lab's RadEyeG20.py (no pyvisa needed)."""

    def __init__(self, port, timeout=2.0):
        self.port = port
        self.ser = serial.Serial(port, 9600, bytesize=serial.SEVENBITS, parity=serial.PARITY_EVEN,
                                 stopbits=serial.STOPBITS_TWO, timeout=timeout)
        self.last_raw = ""

    def setup(self):
        self.ser.reset_input_buffer()
        self.ser.write(b"@")
        self.ser.read(1)
        time.sleep(0.1)
        self.ser.write(b"X1\r\n")        # auto-send mode on
        self.ser.readline()

    def read_raw(self):
        raw = self.ser.readline()
        self.last_raw = raw.decode("ascii", "ignore").strip()
        return self.last_raw

    def read_dose(self):
        """Dose rate in uSv/h, or None if no valid line arrived within the timeout."""
        return parse_radeye_line(self.read_raw())

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


class Gmc800Source:
    """GQ GMC-800 Geiger counter as the dose source (uses gmc800.py, read-only commands only).

    The GMC-800 counts pulses, it does not give a fast dose rate.  <GETCPM>> is a rolling
    one-minute value (far too slow for a few-second X-ray burst) and <GETCPS>> is the count of
    the last second (very noisy: at the 0.1 uSv/h background a second has about 0.25 counts,
    and ONE count in one second would read about 0.4 uSv/h, the usual alarm level).  So:

      dose = mean of the last `window_s` one-second counts  ->  CPM  ->  uSv/h,

    using the device's own calibration table (or `usv_per_cpm` if given).  The window is
    pre-filled from <GETCPM>> at the first read so that the very first values are not noisy.
    With the default 10 s window one count is worth about 0.04 uSv/h, the background scatters by
    about 0.06 uSv/h (1 sigma) and the reaction to a burst is delayed by roughly half a window.

    CPS is the device's own one-second counter, polled about once per second, so an occasional
    second can be read twice or skipped; over a 10 s window this averages out.

    device : optional object with cps(), cpm(), cpm_to_usv_h(cpm) (tests); default is gmc800.GMC800.
    """

    def __init__(self, port="COM8", window_s=10, usv_per_cpm=None, baudrate=115200, device=None):
        if window_s < 1:
            raise ValueError("window_s must be at least 1")
        if device is None:
            from gmc800 import GMC800
            device = GMC800(port, baudrate, usv_per_cpm=usv_per_cpm).open()
        self.dev = device
        self.window_s = int(window_s)
        self._win = []
        self.last_raw = ""

    def setup(self):
        """Read the identification and force the calibration table to be read now (fails loudly)."""
        info = ""
        if hasattr(self.dev, "version"):
            info = "%s serial %s" % (self.dev.version, self.dev.serial_number)
        self.dev.cpm_to_usv_h(60.0)
        return info

    def read_dose(self):
        if not self._win:                                   # first call: start from the rolling CPM
            self._win = [self.dev.cpm() / 60.0] * self.window_s
        cps = self.dev.cps()
        self._win.append(float(cps))
        del self._win[:-self.window_s]
        mean_cps = sum(self._win) / len(self._win)
        dose = self.dev.cpm_to_usv_h(mean_cps * 60.0)
        self.last_raw = "cps=%d window_mean=%.2f cps  ->  %.3f uSv/h" % (cps, mean_cps, dose)
        return dose

    def close(self):
        if hasattr(self.dev, "close"):
            self.dev.close()


class DoseFilter:
    """Drops the fake readings the RadEye gives right after it is (re)started.

    In the 2025 lab data (Sep-Oct, about 11000 files) 8 files start with a first sample of
    843-862 uSv/h while the next sample is back at the 0.09 uSv/h background: stale data from the
    meter's buffer, not a dose.  Real X-ray bursts there were 1-65 uSv/h.  Two protections:

      skip_first   the first sample after start() / reset() (also after a read gap) is thrown away.
      glitch_uSv   any other sample at or above this level is held back for one period and only
                   forwarded if the NEXT sample is also high (at least 20 % of glitch_uSv);
                   otherwise it is counted as a glitch and dropped.  A real dose that high costs
                   one extra period (about 1 s) of reaction time, far shorter than the firmware's
                   xtimeout.

    feed(dose) returns the value to forward to the firmware, or None to send nothing this period.
    """

    def __init__(self, skip_first=1, glitch_uSv=200.0, confirm_fraction=0.2):
        self.skip_first = max(0, int(skip_first))
        self.glitch_uSv = float(glitch_uSv)
        self.confirm_fraction = float(confirm_fraction)
        self.n_skipped = 0
        self.n_glitch = 0
        self.last_reason = ""
        self.reset()

    def reset(self):
        """Call after any gap in the readings (error, reconnect): the next sample is suspect again."""
        self._skip = self.skip_first
        self._suspect = None
        self._in_high = False           # last forwarded sample was already above glitch_uSv (a confirmed episode)

    def feed(self, dose):
        self.last_reason = ""
        if self._skip > 0:
            self._skip -= 1
            self.n_skipped += 1
            self.last_reason = "first dose-meter sample after (re)start ignored (%.2f uSv/h)" % dose
            return None
        if self.glitch_uSv > 0 and self._suspect is None and not self._in_high and dose >= self.glitch_uSv:
            self._suspect = dose
            self.last_reason = "implausible sample %.1f uSv/h held back, waiting for the next one" % dose
            return None
        if self._suspect is not None:
            first, self._suspect = self._suspect, None
            if dose < self.confirm_fraction * self.glitch_uSv:
                self.n_glitch += 1
                self.last_reason = "sample %.1f uSv/h dropped as a glitch (next sample %.2f)" % (first, dose)
        self._in_high = self.glitch_uSv > 0 and dose >= self.glitch_uSv
        return dose


class XrayGuard(threading.Thread):
    """Background thread: read the dose meter, send 'x N' to the firmware, keep the last reading.

    source  : object with read_dose() -> uSv/h or None   (RadEyeG20, or any fake for tests)
    link    : HVLink, or a list of HVLinks (needs .query(cmd)).  With two supplies on one
              set of electrodes BOTH boards must get every reading, so pass both links.
    period  : seconds between reports (the RadEye delivers about one value per second)
    skip_first, glitch_uSv : see DoseFilter (set skip_first=0 and glitch_uSv=0 to forward everything)
    """

    def __init__(self, source, link, period=1.0, log=print, on_alarm=None, xthr_uSv=None,
                 skip_first=1, glitch_uSv=200.0):
        super().__init__(daemon=True)
        self.links = list(link) if isinstance(link, (list, tuple)) else [link]
        self.source, self.link, self.period, self.log = source, self.links[0], period, log
        self.on_alarm = on_alarm
        self.xthr_uSv = xthr_uSv              # only used to flag the alarm in the GUI
        self.filter = DoseFilter(skip_first=skip_first, glitch_uSv=glitch_uSv)
        self.stop_evt = threading.Event()
        self.dose = None
        self.t_dose = 0.0
        self.n_ok = 0
        self.n_bad = 0
        self.last_error = ""
        self.alarm = False

    @property
    def n_glitch(self):
        return self.filter.n_glitch + self.filter.n_skipped

    def stop(self):
        self.stop_evt.set()

    def run(self):
        while not self.stop_evt.is_set():
            t0 = time.time()
            try:
                dose = self.source.read_dose()
            except Exception as e:          # serial error: send nothing, firmware times out
                dose = None
                self.last_error = "读 RadEye 出错: %s" % e
            if dose is None:
                self.n_bad += 1
                self.filter.reset()             # after a gap the next sample may be stale data again
                if not self.last_error:
                    self.last_error = "RadEye 无有效数据"
            else:
                self.last_error = ""
                value = self.filter.feed(dose)
                if self.filter.last_reason:
                    self.log("X 射线读数过滤: " + self.filter.last_reason)
                if value is None:               # first sample / suspected glitch: send nothing this period
                    self.stop_evt.wait(max(0.0, self.period - (time.time() - t0)))
                    continue
                dose = value
                self.n_ok += 1
                self.dose, self.t_dose = dose, time.time()
                level = max(0, min(65535, int(round(dose * 100))))
                for i, lk in enumerate(self.links):   # one board failing must not stop the others
                    try:
                        reply = lk.query("x %d" % level, quiet=True)
                        if reply != "OK":
                            self.log("X 射线上报被拒(板 %d): %r" % (i + 1, reply))
                    except Exception as e:
                        self.last_error = "发送 x 失败(板 %d): %s" % (i + 1, e)
                alarm = bool(self.xthr_uSv is not None and dose >= self.xthr_uSv)
                if alarm and not self.alarm and self.on_alarm:
                    self.on_alarm(dose)
                self.alarm = alarm
            self.stop_evt.wait(max(0.0, self.period - (time.time() - t0)))


class FakeDose:
    """Test source: constant or scripted dose rates (uSv/h)."""

    def __init__(self, values, delay=0.0):
        self.values, self.i, self.delay = list(values), 0, delay

    def read_dose(self):
        time.sleep(self.delay)
        v = self.values[min(self.i, len(self.values) - 1)]
        self.i += 1
        return v


def main():
    ap = argparse.ArgumentParser(description="RadEye G20 / GMC-800 reader and forwarder for HV_control")
    ap.add_argument("--port", required=True, help="dose meter serial port, e.g. COM4 (RadEye) or COM8 (GMC-800)")
    ap.add_argument("--source", choices=("radeye", "gmc800"), default="radeye", help="which dose meter is on --port")
    ap.add_argument("--gmc-window", type=int, default=10,
                    help="GMC-800: seconds of one-second counts averaged for the dose (default 10)")
    ap.add_argument("--gmc-usv-per-cpm", type=float, default=None,
                    help="GMC-800: override the device's own calibration (uSv/h per CPM)")
    ap.add_argument("--raw", action="store_true", help="only print raw lines and parsed values (no HV link)")
    ap.add_argument("--hv-port", nargs="+",
                    help="HV_control serial port(s); give BOTH boards, e.g. --hv-port COM5 COM11")
    ap.add_argument("--count", type=int, default=20)
    args = ap.parse_args()

    if args.source == "gmc800":
        rad = Gmc800Source(args.port, window_s=args.gmc_window, usv_per_cpm=args.gmc_usv_per_cpm)
        print("# GMC-800: " + rad.setup())
        if hasattr(rad.dev, "calibration") and args.gmc_usv_per_cpm is None:
            print("# calibration table (CPM, uSv/h): %s" % (rad.dev.calibration,))
        print("# 1 CPM = %.5f uSv/h,  1 count/s = %.3f uSv/h" % (rad.dev.cpm_to_usv_h(1.0), rad.dev.cpm_to_usv_h(60.0)))
    else:
        rad = RadEyeG20(args.port)
        rad.setup()
    if args.raw or not args.hv_port:
        for _ in range(args.count):
            if args.source == "gmc800":
                t0 = time.time()
                rad.read_dose()
                print(rad.last_raw)
                time.sleep(max(0.0, 1.0 - (time.time() - t0)))
            else:
                raw = rad.read_raw()
                print("raw=%r  ->  %s uSv/h" % (raw, parse_radeye_line(raw)))
        rad.close()
        return

    from hv_link import HVLink
    links = []
    for p in args.hv_port:
        lk = HVLink(log=lambda m: None)
        lk.open(p)
        links.append(lk)
    g = XrayGuard(rad, links)
    g.start()
    try:
        while True:
            time.sleep(1.0)
            print("dose=%s uSv/h  ok=%d bad=%d filtered=%d  %s" % (g.dose, g.n_ok, g.n_bad, g.n_glitch, g.last_error))
    except KeyboardInterrupt:
        pass
    finally:
        g.stop()
        rad.close()
        for lk in links:
            lk.close()


if __name__ == "__main__":
    main()
