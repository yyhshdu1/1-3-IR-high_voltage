"""
test_psu_watchdog.py - offline test of psu_watchdog.py (no hardware: simulated SPD3303X and
simulated Arduino link, real UDP on 127.0.0.1 with a random port).

    python test_psu_watchdog.py
"""
import sys
import threading
import time

import psu_watchdog as pw
from hv_link import HVError, HVLink, HVTimeout, raises_hv

passed = failed = 0


def check(name, cond, extra=""):
    global passed, failed
    passed += bool(cond)
    failed += not cond
    print(("PASS " if cond else "FAIL ") + name + (("   [%s]" % extra) if extra else ""))


def wait_until(cond, timeout=3.0):
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        if cond():
            return True
        time.sleep(0.02)
    return cond()


class FakePSU(object):
    def __init__(self):
        self.on, self.v, self.i, self.cc, self.alive = True, 24.0, 0.2, False, True
        self.idn = "FAKE SPD3303X"
        self.n_off = 0

    def _chk(self):
        if not self.alive:
            raise RuntimeError("VISA timeout (simulated)")

    def status(self):
        self._chk()
        return {"CH1_on": self.on, "CH1_cc": self.cc, "mode": "independent"}

    def measure(self, ch):
        self._chk()
        return {"v": self.v if self.on else 0.0, "i": 0.05 if self.on else 0.0, "p": 0.0}

    def voltage_setpoint(self, ch):
        self._chk()
        return self.v

    def current_setpoint(self, ch):
        self._chk()
        return self.i

    def output(self, ch, on):
        self._chk()
        self.on = bool(on)
        if not on:
            self.n_off += 1

    def close(self):
        pass


class FakeLink(object):
    """Just enough of HVLink for WatchdogClient."""

    def __init__(self, port="COMX"):
        self.lock = threading.RLock()
        self.port = port
        self._open = True
        self.alive = True
        self.generation = 1
        self.last_ok = 0.0
        self.last_status = None
        self.t_boot = time.monotonic() - 100
        self.st = {"hv": 0, "tgt": 0, "cur": 0, "raw": 0, "rst": 0x20, "n_por": 1, "n_bor": 0, "n_wdt": 0}
        self.n_z = 0

    @property
    def is_open(self):
        return self._open

    def status(self, quiet=True):
        with self.lock:
            if not self._open:
                raise HVError("serial port not open")
            if not self.alive:
                raise HVTimeout("no reply (simulated)")
            st = dict(self.st, up=int(time.monotonic() - self.t_boot))
            self.last_ok = time.monotonic()
            self.last_status = (self.last_ok, self.generation, st)
            return st

    def emergency_stop(self):
        with self.lock:
            if not self._open or not self.alive:
                raise HVError("emergency stop got no reply (simulated)")
            self.n_z += 1
            self.st.update(hv=0, tgt=0, cur=0, raw=0)
            return "OK"

    def reopen(self):            # intentional reopen = DTR reset, new generation
        with self.lock:
            self.generation += 1
            self.t_boot = time.monotonic()
            self.st.update(hv=0, tgt=0, cur=0, raw=0, rst=0x20)
            self._open = True


logs = []


def wlog(msg):
    logs.append(msg)


psu = FakePSU()


def factory():
    psu._chk()
    return psu


def start_server(**kw):
    opts = dict(hb_timeout=1.0, ok_age_max=1.5, poll_s=0.05, reconnect_s=0.2, log=wlog)
    opts.update(kw)
    wd = pw.PsuWatchdog(factory, **opts)
    stop, ready = threading.Event(), threading.Event()
    th = threading.Thread(target=wd.serve, args=(("127.0.0.1", 0), stop, ready), daemon=True)
    th.start()
    ready.wait(2)
    return wd, stop, th


def new_client(wd, link, name="TOF+"):
    c = pw.WatchdogClient(link, name, addr=wd.addr, period=0.1, reply_timeout=0.6, log=wlog)
    c.start()
    return c


# ------------------------------------------------------------------ start / arm
wd, stop, th = start_server()
check("start: IDLE, CH1 left ON as found", wait_until(lambda: wd.expected_on is True) and
      wd.state == pw.IDLE and psu.on)
link = FakeLink()
cl = new_client(wd, link)
check("client not armed: permit refuses e 1", cl.permit("e 1") is not None)
r = cl.arm()
check("arm -> ARMED", r["ok"] and r["state"] == pw.ARMED, r.get("error"))
check("armed: permit allows e 1", wait_until(lambda: cl.permit("e 1") is None), cl.permit("e 1"))
time.sleep(1.5)
check("healthy heartbeats for 1.5 s: still ARMED, CH1 ON", wd.state == pw.ARMED and psu.on)

# ------------------------------------------------------------------ intentional reopen
link.reopen()
time.sleep(0.8)
check("intentional reopen (new generation): no trip", wd.state == pw.ARMED and psu.on)

# ------------------------------------------------------------------ power_on refusals
r = cl.power_on()
check("power_on while already on: allowed (idempotent)", r["ok"], r.get("error"))
link.st["hv"] = 1
time.sleep(0.25)
r = cl.power_on()
check("power_on refused while HV on", not r["ok"] and "HV" in r["error"], r.get("error"))
link.st["hv"] = 0
psu.v = 12.0
time.sleep(0.25)
r = cl.power_on()
check("power_on refused with 12 V setpoint", not r["ok"] and "setpoint" in r["error"], r.get("error"))
psu.v = 24.0

# ------------------------------------------------------------------ 1. control program stops
cl.stop()
cl.join(1)
t0 = time.monotonic()
check("heartbeat lost -> TRIPPED", wait_until(lambda: wd.state == pw.TRIPPED, 3))
check("heartbeat lost -> CH1 OFF", not psu.on, "%.1f s, reason: %s" % (time.monotonic() - t0, wd.reason))
check("trip reason names the heartbeat", "heartbeat" in wd.reason, wd.reason)

# restart of the control program: same name, arm while tripped, reset, power on
link2 = FakeLink()
cl = new_client(wd, link2)
r = cl.arm()
check("arm while TRIPPED: accepted with warning", r["ok"] and "TRIPPED" in r["error"], r.get("error"))
check("tripped watchdog -> client sends z once", wait_until(lambda: link2.n_z == 1), "n_z=%d" % link2.n_z)
check("tripped: permit refuses d 100", cl.permit("d 100") is not None)
time.sleep(0.3)
r = cl.power_on()
check("power_on refused while TRIPPED", not r["ok"], r.get("error"))
r = cl.reset()
check("reset accepted (fresh, HV off) -> ARMED, CH1 still OFF", r["ok"] and r["state"] == pw.ARMED and not psu.on,
      r.get("error"))
r = cl.power_on()
check("power_on after reset -> CH1 ON", r["ok"] and psu.on, r.get("error"))
time.sleep(0.3)
check("z sent only once for that trip", link2.n_z == 1, "n_z=%d" % link2.n_z)


def recover(link_, what):
    r1 = cl.reset()
    r2 = cl.power_on()
    check("recover after %s: reset + power_on" % what, r1["ok"] and r2["ok"] and psu.on,
          "%s / %s" % (r1.get("error"), r2.get("error")))


# ------------------------------------------------------------------ 2. Arduino stops answering
link2.alive = False
t0 = time.monotonic()
check("Arduino silent -> TRIPPED, CH1 OFF", wait_until(lambda: wd.state == pw.TRIPPED and not psu.on, 4),
      "%.1f s, %s" % (time.monotonic() - t0, wd.reason))
check("reason: no reply from the Arduino", "no reply from the Arduino" in wd.reason, wd.reason)
r = cl.reset()
check("reset refused while Arduino still silent", not r["ok"], r.get("error"))
link2.alive = True
time.sleep(0.3)
recover(link2, "silent Arduino")

# ------------------------------------------------------------------ 3. serial port lost
link2._open = False
t0 = time.monotonic()
check("serial port closed -> TRIPPED, CH1 OFF", wait_until(lambda: wd.state == pw.TRIPPED and not psu.on, 2),
      "%.2f s, %s" % (time.monotonic() - t0, wd.reason))
link2.reopen()
time.sleep(0.3)
recover(link2, "serial loss")

# ------------------------------------------------------------------ 4. spontaneous resets
# 4a: watchdog reset right after a reopen ('up' is still 0, so only rst / n_wdt show it)
link2.t_boot = time.monotonic()
link2.st.update(rst=0x08, n_wdt=link2.st["n_wdt"] + 1)
check("spontaneous WDT reset (up unchanged) -> TRIPPED", wait_until(lambda: wd.state == pw.TRIPPED and not psu.on, 2),
      wd.reason)
check("reason: reset by itself", "reset by itself" in wd.reason, wd.reason)
time.sleep(0.3)
recover(link2, "spontaneous WDT reset")
# 4b: same cause again later: only 'up' going back and the counter show it
time.sleep(1.2)
link2.t_boot = time.monotonic()
link2.st.update(n_wdt=link2.st["n_wdt"] + 1)
check("second WDT reset (same rst) -> TRIPPED", wait_until(lambda: wd.state == pw.TRIPPED and not psu.on, 2),
      wd.reason)
time.sleep(0.3)
recover(link2, "second WDT reset")
# 4c: intentional reopen after all this: new generation, rst back to 0x20, no trip
link2.reopen()
time.sleep(0.6)
check("intentional reopen with changed rst: no trip", wd.state == pw.ARMED and psu.on, wd.reason)

# ------------------------------------------------------------------ 5. CH1 switched off at the supply
nz = link2.n_z
psu.on = False
check("CH1 found OFF while armed -> TRIPPED", wait_until(lambda: wd.state == pw.TRIPPED, 2), wd.reason)
check("... and the client sends z", wait_until(lambda: link2.n_z == nz + 1, 2), "n_z=%d" % link2.n_z)
time.sleep(0.3)
recover(link2, "CH1 off at the supply")

# ------------------------------------------------------------------ 6. SPD3303X lost
nz = link2.n_z
psu.alive = False
check("SPD3303X lost -> TRIPPED", wait_until(lambda: wd.state == pw.TRIPPED and not wd.psu_ok, 2), wd.reason)
check("... client sends z", wait_until(lambda: link2.n_z == nz + 1, 2), "n_z=%d" % link2.n_z)
check("... permit refuses e 1", cl.permit("e 1") is not None, cl.permit("e 1"))
check("CH1 still ON (supply unreachable, off pending)", psu.on and wd.off_pending)
psu.alive = True
check("SPD3303X back -> CH1 switched OFF at once", wait_until(lambda: not psu.on and not wd.off_pending, 2))
time.sleep(0.3)
recover(link2, "SPD3303X loss")

# ------------------------------------------------------------------ 7. disarm
link2.st["hv"] = 1
time.sleep(0.25)
r = cl.disarm()
check("disarm while HV on -> TRIPPED, CH1 OFF", wd.state == pw.TRIPPED and not psu.on, wd.reason)
link2.st["hv"] = 0
r = cl.arm()
time.sleep(0.3)
recover(link2, "unsafe disarm")
r = cl.disarm()
check("disarm with HV off -> IDLE, CH1 stays ON", r["state"] == pw.IDLE and psu.on, r.get("error"))
time.sleep(1.5)
check("IDLE: no trip without heartbeats from a disarmed client", wd.state == pw.IDLE and psu.on)
r = cl.arm()
check("re-arm", r["ok"] and r["state"] == pw.ARMED, r.get("error"))

# ------------------------------------------------------------------ 8. second watchdog on the same port
try:
    s = pw.bind_socket(wd.addr)
    s.close()
    check("second watchdog cannot bind the same port", False)
except OSError as e:
    check("second watchdog cannot bind the same port", True, type(e).__name__)

# ------------------------------------------------------------------ 9. watchdog process dies
nz = link2.n_z
stop.set()
th.join(2)
check("watchdog gone -> client sends z", wait_until(lambda: link2.n_z == nz + 1, 2), "n_z=%d" % link2.n_z)
check("watchdog gone -> permit refuses e 1", cl.permit("e 1") is not None, cl.permit("e 1"))
time.sleep(1.5)
check("watchdog gone: z sent once, not every heartbeat", link2.n_z == nz + 1, "n_z=%d" % link2.n_z)
cl.stop()
cl.join(1)

# ------------------------------------------------------------------ HVLink permit hook
check("raises_hv", raises_hv("e 1") and raises_hv("d 100") and not raises_hv("d 0") and
      not raises_hv("e 0") and not raises_hv("s") and not raises_hv("d abc") and not raises_hv("x 5"))


class FakeSerial(object):
    is_open = True
    timeout = 1

    def __init__(self):
        self.sent = []

    def reset_input_buffer(self):
        pass

    def write(self, b):
        self.sent.append(b)

    def readline(self):
        return b"OK\n"

    def close(self):
        pass


lk = HVLink(log=wlog)
lk.ser = FakeSerial()
lk.permit = lambda cmd: "not armed"
check("HVLink: blocked e 1 -> ERR PSU_WD, nothing sent", lk.query("e 1") == "ERR PSU_WD" and not lk.ser.sent)
check("HVLink: d 0 still sent", lk.query("d 0") == "OK" and len(lk.ser.sent) == 1)
lk.permit = lambda cmd: None
check("HVLink: allowed e 1 sent", lk.query("e 1") == "OK" and len(lk.ser.sent) == 2)
check("HVLink: last_ok updated on reply", lk.last_ok > 0)

print("\n%d passed, %d failed" % (passed, failed))
if failed:
    print("\n".join(logs[-40:]))
sys.exit(1 if failed else 0)
