"""No-hardware test of the RadEye glitch filter in xray_guard.py:  python test_xray_guard_filter.py"""
import sys
import time

from xray_guard import DoseFilter, FakeDose, XrayGuard

fails = []


def check(name, ok, info=""):
    print(("PASS  " if ok else "FAIL  ") + name + ("   " + info if info else ""))
    if not ok:
        fails.append(name)


# ---- DoseFilter alone ------------------------------------------------------------------------
f = DoseFilter()
out = [f.feed(v) for v in (853.2, 0.09, 0.10, 44.1, 0.09)]
check("first sample 853 is dropped, the rest pass unchanged", out == [None, 0.09, 0.10, 44.1, 0.09], str(out))
check("first-sample counter", f.n_skipped == 1 and f.n_glitch == 0)

f = DoseFilter()
f.feed(0.09)
out = [f.feed(v) for v in (0.09, 862.0, 0.10, 0.09)]
check("mid-stream 862 followed by background is dropped as a glitch", out == [0.09, None, 0.10, 0.09], str(out))
check("glitch counter", f.n_glitch == 1)

f = DoseFilter()
f.feed(0.09)
out = [f.feed(v) for v in (900.0, 700.0, 300.0)]
check("a real high dose is confirmed by the next sample (one period late)", out == [None, 700.0, 300.0], str(out))
check("confirmed high dose is not counted as a glitch", f.n_glitch == 0)

f = DoseFilter()
f.feed(0.09)
out = [f.feed(v) for v in (1.2, 12.0, 64.4, 150.0, 0.5)]
check("real X-ray values up to 150 uSv/h pass at once", out == [1.2, 12.0, 64.4, 150.0, 0.5], str(out))

f = DoseFilter()
f.feed(0.09)
f.feed(0.09)
f.reset()
check("after reset the next sample is dropped again", f.feed(0.5) is None and f.feed(0.5) == 0.5)

f = DoseFilter(skip_first=0, glitch_uSv=0)
check("filter can be switched off", [f.feed(v) for v in (850.0, 0.1)] == [850.0, 0.1])


# ---- XrayGuard thread with a fake meter and fake links -----------------------------------------
class FakeLink:
    def __init__(self):
        self.sent = []

    def query(self, cmd, quiet=False):
        self.sent.append(cmd)
        return "OK"


class Gappy(FakeDose):
    """None (read error) at the given call indices."""

    def __init__(self, values, gaps):
        super().__init__(values)
        self.gaps, self.n = set(gaps), 0

    def read_dose(self):
        k, self.n = self.n, self.n + 1
        if k in self.gaps:
            return None
        return super().read_dose()


la, lb = FakeLink(), FakeLink()
g = XrayGuard(FakeDose([850.0, 0.09, 0.09, 5.5, 900.0, 0.10, 0.11]), [la, lb], period=0.02, log=lambda m: None)
g.start()
time.sleep(0.5)
g.stop()
g.join(1)
sent = la.sent
check("both boards get identical reports", la.sent == lb.sent)
check("850 and 900 uSv/h never reach the firmware", all(int(c.split()[1]) < 20000 for c in sent), str(sent[:12]))
check("first report is the first good value (9 = 0.09 uSv/h)", sent[:3] == ["x 9", "x 9", "x 550"], str(sent[:6]))
check("filter statistics", g.n_glitch == 2, "n_glitch=%d" % g.n_glitch)

lc = FakeLink()
g = XrayGuard(Gappy([0.09, 0.09, 700.0, 0.09, 0.09], gaps=[2]), lc, period=0.02, log=lambda m: None)
g.start()
time.sleep(0.4)
g.stop()
g.join(1)
check("after a read gap the next sample (700) is dropped as stale", all(int(c.split()[1]) < 20000 for c in lc.sent), str(lc.sent))

# ---- GMC-800 source with a fake device (no hardware) -------------------------------------------
from xray_guard import Gmc800Source


class FakeGmc:
    """cpm() = rolling minute; cps() = scripted one-second counts; 1 CPM = 0.0065 uSv/h (about 154 CPM per uSv/h)."""

    def __init__(self, cpm, counts):
        self._cpm, self.counts, self.i = cpm, list(counts), 0

    def cpm(self):
        return self._cpm

    def cps(self):
        v = self.counts[min(self.i, len(self.counts) - 1)]
        self.i += 1
        return v

    def cpm_to_usv_h(self, cpm):
        return cpm * 0.0065


# background about 15 CPM = 0.25 counts/s: the first value is pre-filled from CPM, so it is already sensible
src = Gmc800Source(device=FakeGmc(15, [0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0]), window_s=10)
d0 = src.read_dose()
check("first GMC value comes from the rolling CPM (0.0975 uSv/h)", abs(d0 - 15 * 0.0065) < 0.01, "%.3f" % d0)
vals = [src.read_dose() for _ in range(11)]
check("a single count in a second does not reach the 0.40 uSv/h alarm level", max(vals) < 0.4, "max %.3f" % max(vals))

src = Gmc800Source(device=FakeGmc(15, [0] * 5 + [5] * 10 + [0] * 10), window_s=10)
vals = [src.read_dose() for _ in range(25)]
check("a 10 s burst of 5 counts/s reaches the alarm level", max(vals) > 0.4, "max %.2f uSv/h" % max(vals))
check("... and the window average falls back to background afterwards", vals[-1] < 0.1, "last %.3f" % vals[-1])

g = XrayGuard(Gmc800Source(device=FakeGmc(15, [0] * 3 + [30, 30] + [0] * 30), window_s=5), FakeLink(), period=0.02, log=lambda m: None)
g.start()
time.sleep(0.8)
g.stop()
g.join(1)
check("GMC source works inside XrayGuard (values forwarded, dose tracked)", g.n_ok >= 10 and g.dose is not None, "n_ok=%d dose=%s" % (g.n_ok, g.dose))

try:
    Gmc800Source(device=FakeGmc(15, [0]), window_s=0)
    check("window_s = 0 rejected", False)
except ValueError:
    check("window_s = 0 rejected", True)

print()
print("ALL PASS" if not fails else "FAILED: %s" % fails)
sys.exit(1 if fails else 0)
