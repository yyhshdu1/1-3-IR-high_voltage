"""
stage7_tests.py - stage 7 tests with the TOF3000 connected, kept far below 1 kV:
HV_enable shutdown (1b), firmware / GUI voltage caps, low staircase with v_mon cross-check,
X-ray interlock in the firmware, and the 24 V outer interlock at HV.

Runs inside the bring-up GUI (tab "Stage 7 tests"), driven through a small context object
(see GuiCtx in hv_bringup_gui.py, FakeCtx in test_stage7_sim.py):

    ctx.link              HVLink (owned by the GUI, its poll loop keeps running)
    ctx.log(msg), ctx.emit(name, ok, detail), ctx.ask(msg) -> bool, ctx.stop (Event)
    ctx.step(text)        show the current test in the GUI
    ctx.wd()              current WatchdogClient or None
    ctx.wd_status()       watchdog status dict (raises psu_watchdog.WatchdogError)
    ctx.wd_restart()      start the watchdog process again and re-arm; True if ARMED
    ctx.wd_kill()         kill the watchdog process (like Task Manager)
    ctx.wd_trip(reason)   ask the watchdog to trip (cuts 24 V)
    ctx.xray_start(src), ctx.xray_stop(), ctx.xray_set_source(src), ctx.make_gmc(port)

Voltage safety (every wait loop samples v_mon and aborts with 'z'):
    - targets never above test_max_kv (default 0.8 kV)
    - GUI permit cap and firmware 'c dacmax' (RAM only) at cap_kv (default 0.9 kV)
    - v_mon above abort_kv (default 0.95 kV), or v_mon off the target by more than 25 %,
      or any firmware fault -> z, abort
Conversion (首次上电步骤.md): 1 kV = 1969 host units, v_prog ~ v_mon ~ kV/3 V.
"""
import datetime
import os
import time

UNITS_PER_KV = 59055 / 30.0          # 1968.5
VMON_KV_PER_V = 3.0
IMON_UA_PER_V = 40.0                  # 400 uA = 10 V (to be checked against the TOF3000 manual)
X_LEVEL_SIM_HIGH = 1.00               # uSv/h, above the 0.40 uSv/h test threshold
X_LEVEL_SIM_LOW = 0.10


class Abort(Exception):
    pass


class SimDose(object):
    """Simulated dose source for XrayGuard; the test sets .dose (uSv/h)."""

    def __init__(self, dose=X_LEVEL_SIM_LOW):
        self.dose = dose
        self.last_raw = ""

    def read_dose(self):
        time.sleep(0.05)
        self.last_raw = "simulated %.2f uSv/h" % self.dose
        return self.dose


def kv_units(kv):
    return int(round(kv * UNITS_PER_KV))


class Stage7(object):
    def __init__(self, ctx, test_max_kv=0.8, cap_kv=0.9, abort_kv=0.95, hold_s=10.0, xport="COM5",
                 sim_source=None):
        if not (0 < test_max_kv < cap_kv < abort_kv <= 1.0):
            raise ValueError("need 0 < test_max_kv < cap_kv < abort_kv <= 1.0 kV")
        self.ctx, self.link = ctx, ctx.link
        self.test_max_kv, self.cap_kv, self.abort_kv = test_max_kv, cap_kv, abort_kv
        self.cap_units = int(cap_kv * UNITS_PER_KV)
        self.hold_s, self.xport = hold_s, xport
        self.sim = sim_source
        self.off_v = {"v_mon": 0.0, "i_mon": 0.0}
        self.orig_cfg = {}
        self.max_kv_seen = 0.0
        self.rows = []                    # (name, ok, detail) for the report
        self.t0 = time.time()

    # ------------------------------------------------------------- helpers
    def log(self, msg):
        self.ctx.log(msg)

    def emit(self, name, ok, detail=""):
        self.rows.append((name, ok, detail))
        self.ctx.emit(name, ok, detail)
        return ok

    def q(self, cmd, quiet=True):
        return self.link.query(cmd, quiet=quiet)

    def must(self, cmd, want="OK"):
        r = self.link.query(cmd, quiet=False)
        if r != want:
            raise Abort("%s -> %r (expected %r)" % (cmd, r, want))
        return r

    def set_target(self, kv):
        if kv > self.test_max_kv + 1e-9:
            raise Abort("refusing target %.3f kV above the test maximum %.2f kV" % (kv, self.test_max_kv))
        return self.must("d %d" % kv_units(kv))

    def zero(self):
        try:
            r = self.link.emergency_stop()
            self.log("z -> %s" % r)
        except Exception as e:  # noqa: BLE001
            self.log("!! z failed: %s" % e)

    def sample(self, check=True):
        """Status + corrected monitor readings; enforces the voltage limits."""
        st = self.link.status()
        v = self.link.volts()
        vmon = v[0] - self.off_v["v_mon"]
        kv = vmon * VMON_KV_PER_V
        s = {"st": st, "v": v, "kv": kv, "vmon": vmon, "vprog": v[2],
             "iua": (v[1] - self.off_v["i_mon"]) * IMON_UA_PER_V}
        if v[0] < 12.0:
            self.max_kv_seen = max(self.max_kv_seen, kv)
        if check:
            if v[0] < 12.0 and kv > self.abort_kv:
                self.zero()
                raise Abort("v_mon %.3f V = %.3f kV is above the %.2f kV abort level" % (vmon, kv, self.abort_kv))
            if v[2] < 12.0 and v[2] * VMON_KV_PER_V > self.cap_kv * 1.05:   # 12.29 V = ADC dead (no 24 V)
                self.zero()
                raise Abort("v_prog %.3f V (= %.3f kV) is above the cap" % (v[2], v[2] * VMON_KV_PER_V))
        return s

    def wait(self, seconds, until=None, check=True, interval=0.3):
        """Sample every `interval` s for up to `seconds`; stop early when until(sample) is true."""
        t_end = time.monotonic() + seconds
        s = self.sample(check)
        while True:
            if self.ctx.stop.is_set():
                self.zero()
                raise Abort("stopped by the user")
            if until is not None and until(s):
                return s, True
            if time.monotonic() >= t_end:
                return s, until is None
            time.sleep(interval)
            s = self.sample(check)

    def settle(self, timeout=20.0):
        """Wait until the DAC reached its target (D: tgt == cur)."""
        def done(_s):
            t, c, _r = self.link.dstate()
            return t == c
        s, ok = self.wait(timeout, done)
        if not ok:
            raise Abort("DAC did not settle within %.0f s (D=%s)" % (timeout, self.q("D")))
        return s

    def no_fault(self, s, where):
        flt = s["st"].get("flt", 0)
        if flt:
            self.zero()
            raise Abort("%s: firmware fault flt=0x%X" % (where, flt))

    def decay_time(self, below_v=0.02, timeout=60.0):
        t0 = time.monotonic()
        s, ok = self.wait(timeout, lambda s: s["vmon"] < below_v)
        return (time.monotonic() - t0) if ok else None, s

    def supply(self):
        try:
            m = self.ctx.wd_status().get("meas") or {}
            return "24 V supply %.3f V %.3f A" % (m.get("v") or 0.0, m.get("i") or 0.0)
        except Exception as e:  # noqa: BLE001
            return "24 V supply: %s" % e

    def hv_off_and_zero(self):
        self.q("d 0")
        try:
            self.settle(30)
        except Abort:
            self.zero()
        self.q("e 0")

    # --------------------------------------------------------------- tests
    def prepare(self):
        self.ctx.step("0. Preparation")
        lk = self.link
        banner = self.q("?")
        if "Pol +" not in banner:
            raise Abort("board is not Pol +: %s" % banner)
        wd = self.ctx.wd()
        if wd is None:
            raise Abort("no watchdog client (connect in the GUI first)")
        st = self.ctx.wd_status()
        if not wd.armed or st.get("state") != "ARMED":
            r = wd.arm()
            st = self.ctx.wd_status()
            if st.get("state") != "ARMED":
                raise Abort("watchdog not ARMED: %s %s" % (st.get("state"), r.get("error") or st.get("reason")))
        if not (st.get("meas") or {}).get("on"):
            r = wd.power_on()
            if not r.get("ok") or r.get("error"):
                raise Abort("24 V ON refused: %s" % r.get("error"))
            time.sleep(2.0)
        self.q("C")
        time.sleep(0.5)
        s = self.sample(check=False)
        ok = s["st"].get("flt") == 0 and s["st"].get("hv") == 0 and self.q("D") == "0,0,0"
        self.emit("0 start: 24 V on, no fault, HV off, DAC 0", ok, "%s; %s" % (self.supply(), _st(s["st"])))
        if not ok:
            raise Abort("board not in a clean state")

        self.orig_cfg = lk.config()
        for k, v in (("cable", 1), ("dacmax", self.cap_units)):
            self.must("c %s %d" % (k, v))
        cfg = lk.config()
        self.emit("0 firmware cap in RAM: dacmax=%d (%.2f kV), cable=1 (not saved)" % (self.cap_units, self.cap_kv),
                  cfg.get("dacmax") == self.cap_units and cfg.get("cable") == 1,
                  "was dacmax=%s cable=%s" % (self.orig_cfg.get("dacmax"), self.orig_cfg.get("cable")))

        # zero offsets with the TOF3000 at 0 V, HV off
        acc = [0.0, 0.0]
        n = 15
        for _ in range(n):
            v = lk.volts()
            acc[0] += v[0]
            acc[1] += v[1]
            time.sleep(0.1)
        self.off_v = {"v_mon": acc[0] / n, "i_mon": acc[1] / n}
        ok = abs(self.off_v["v_mon"]) < 0.1 and abs(self.off_v["i_mon"]) < 0.5
        self.emit("0 v_mon / i_mon at HV off (used as zero offsets)", ok,
                  "v_mon %.4f V, i_mon %.4f V%s" % (self.off_v["v_mon"], self.off_v["i_mon"],
                                                     "" if ok else "  (cable not connected or TOF3000 not at 0 V?)"))
        if not ok:
            raise Abort("monitor inputs not plausible at HV off")

        r = self.q("d %d" % (self.cap_units + 1))
        self.emit("0 GUI cap refuses d %d (above %.2f kV), nothing sent" % (self.cap_units + 1, self.cap_kv),
                  r == "ERR PSU_WD" and self.q("D") == "0,0,0", r)
        if r == "OK":
            self.zero()
            raise Abort("GUI cap did not refuse")

    def test_1b(self):
        self.ctx.step("1. HV_enable shutdown (1b): e 0, d 0.5 kV, v_mon must stay at 0")
        self.must("e 0")
        self.must("d 1000")
        s = self.settle()
        vprog_ok = abs(s["vprog"] - 1000 / UNITS_PER_KV / VMON_KV_PER_V) < 0.05
        peak = s["vmon"]
        t_end = time.monotonic() + 8
        while time.monotonic() < t_end:
            peak = max(peak, self.sample()["vmon"])
            time.sleep(0.25)
        ok = peak < 0.02
        self.emit("1b HV_enable off: v_mon stays ~0 with d 1000 (0.51 kV programmed)", ok,
                  "v_prog %.3f V (expected 0.169)%s, peak v_mon %.4f V (= %.0f V)" % (
                      s["vprog"], "" if vprog_ok else " !", peak, peak * VMON_KV_PER_V * 1000))
        self.must("d 0")
        self.settle()
        if not ok:
            self.zero()
            raise Abort("1b FAILED: the TOF3000 is not shut off by HV_enable high. Stop: fix the hardware "
                        "before any further HV test (do not rely on e 0).")

    def test_firmware_cap(self):
        self.ctx.step("2. Firmware cap: d above dacmax must be refused by the firmware")
        cmd = "d %d" % (self.cap_units + 50)
        with self.link.lock:               # bypass the GUI cap for this one command (HV is off)
            p = self.link.permit
            self.link.permit = None
            try:
                r = self.link.query(cmd, quiet=False)
            finally:
                self.link.permit = p
        if r == "OK":
            self.q("d 0")
            self.zero()
        self.emit("2 firmware refuses %s (dacmax %d)" % (cmd, self.cap_units), r == "ERR RANGE", r)
        if r != "ERR RANGE":
            raise Abort("firmware cap not effective")

    def test_staircase(self):
        steps = [kv for kv in (0.2, 0.4, 0.6, 0.8) if kv <= self.test_max_kv + 1e-9]
        self.ctx.step("3. Staircase %s kV, %.0f s each" % ("/".join("%.1f" % k for k in steps), self.hold_s))
        self.must("e 1")
        for kv in steps:
            self.ctx.step("3. Staircase: %.1f kV" % kv)
            self.set_target(kv)
            s = self.settle()
            self.no_fault(s, "%.1f kV" % kv)
            ev0 = s["st"].get("ev", 0)
            kvs, ius = [], []
            t_end = time.monotonic() + self.hold_s
            while time.monotonic() < t_end:
                s, _ = self.wait(0.5)
                self.no_fault(s, "%.1f kV" % kv)
                kvs.append(s["kv"])
                ius.append(s["iua"])
            mean_kv = sum(kvs) / len(kvs)
            dev = (mean_kv - kv) / kv
            ok = abs(dev) <= 0.25
            self.emit("3 %.1f kV: v_mon agrees with the target" % kv, ok,
                      "v_mon %.3f kV (%+.1f %%), v_prog %.3f V, i_mon %.2f uA, ev +%d, %s" % (
                          mean_kv, dev * 100, s["vprog"], sum(ius) / len(ius), s["st"].get("ev", 0) - ev0,
                          self.supply()))
            if not ok:
                self.zero()
                raise Abort("v_mon %.3f kV deviates %.0f %% from %.1f kV" % (mean_kv, dev * 100, kv))
        self.ctx.step("3. Ramp down and decay")
        self.must("d 0")
        self.settle()
        t, s = self.decay_time()
        self.emit("3 ramp down: v_mon back below 0.02 V", t is not None,
                  "%.1f s after the DAC reached 0" % t if t is not None else "still %.3f V after 60 s" % s["vmon"])
        self.must("e 0")

    def test_xray(self):
        lk = self.link
        self.ctx.step("4a. X-ray: real GMC-800 background on %s" % self.xport)
        gmc = None
        try:
            gmc = self.ctx.make_gmc(self.xport)
        except Exception as e:  # noqa: BLE001
            self.emit("4a GMC-800 on %s opens" % self.xport, False, repr(e))
        if gmc is not None:
            g = self.ctx.xray_start(gmc)
            s, _ = self.wait(14.0)
            xr = s["st"].get("xr", 0)
            self.emit("4a GMC-800 readings reach the firmware (background below the 0.40 uSv/h test threshold)",
                      g.n_ok >= 3 and 0 <= xr < 40,
                      "xr=%d (x0.01 uSv/h), forwarded %d, bad %d, last: %s" % (xr, g.n_ok, g.n_bad, gmc.last_raw))
        else:
            self.sim.dose = X_LEVEL_SIM_LOW
            self.ctx.xray_start(self.sim)
            self.wait(3.0)

        self.ctx.step("4b. X-ray: alarm with simulated 1.00 uSv/h at 0.6 kV -> stepwise back-off")
        orig = {k: self.orig_cfg.get(k) for k in ("xthr", "xback", "xtimeout")}
        for k, v in (("xthr", 40), ("xback", 400), ("xtimeout", 5)):
            self.must("c %s %d" % (k, v))
        self.must("e 1")
        self.set_target(0.6)
        s = self.settle()
        self.no_fault(s, "x-ray 0.6 kV")
        x0 = s["st"].get("xtrips", 0)
        kv_before = s["kv"]
        self.sim.dose = X_LEVEL_SIM_HIGH
        self.ctx.xray_set_source(self.sim)
        s, ok = self.wait(14.0, lambda s: s["st"].get("tgt", 9999) <= kv_units(0.6) - 800)
        self.emit("4b alarm: firmware backs off 400 units per step (every 3 s)", ok and s["st"].get("xh") == 1,
                  "xtrips +%d, tgt %s, v_mon %.3f -> %.3f kV" % (s["st"].get("xtrips", 0) - x0, s["st"].get("tgt"),
                                                                kv_before, s["kv"]))
        r = self.q("d %d" % kv_units(0.8))
        self.emit("4b alarm: raising the target is refused (ERR XRAY)", r == "ERR XRAY", r)
        if r == "OK":
            self.q("d 0")

        self.ctx.step("4c. X-ray: back to 0.10 uSv/h -> hold released, voltage can be raised again")
        self.sim.dose = X_LEVEL_SIM_LOW
        s, ok = self.wait(5.0, lambda s: s["st"].get("xh") == 0)
        r = self.q("d %d" % kv_units(0.6))
        s = self.settle() if r == "OK" else s
        self.emit("4c hold released below 80 % of the threshold; d 0.6 kV accepted", ok and r == "OK",
                  "xh=%s, d -> %s, v_mon %.3f kV" % (s["st"].get("xh"), r, s["kv"]))

        self.ctx.step("4d. X-ray: monitor lost (forwarding stopped) at 0.6 kV -> 0x40 after 5 s")
        self.ctx.xray_stop()
        t0 = time.monotonic()
        s, ok = self.wait(9.0, lambda s: s["st"].get("flt", 0) & 0x40, check=True)
        t_flt = time.monotonic() - t0
        s, off = self.wait(10.0, lambda s: s["st"].get("hv") == 0 and self.q("D") == "0,0,0")
        wst = self.ctx.wd_status()
        self.emit("4d monitor lost: flt 0x40 after ~5 s, ramp down, HV off; 24 V untouched",
                  ok and off and wst.get("state") == "ARMED" and (wst.get("meas") or {}).get("on"),
                  "0x40 after %.1f s; %s; v_mon %.3f kV; watchdog %s" % (t_flt, _st(s["st"]), s["kv"], wst.get("state")))
        self.q("C")

        self.ctx.step("4e. X-ray: xback 0 -> alarm latches fault 0x20 and ramps to 0")
        self.must("c xback 0")
        self.sim.dose = X_LEVEL_SIM_LOW
        self.ctx.xray_start(self.sim)
        self.wait(2.5)
        self.must("e 1")
        self.set_target(0.4)
        s = self.settle()
        self.no_fault(s, "x-ray 0.4 kV")
        self.sim.dose = X_LEVEL_SIM_HIGH
        s, ok = self.wait(5.0, lambda s: s["st"].get("flt", 0) & 0x20)
        s, off = self.wait(10.0, lambda s: s["st"].get("hv") == 0 and self.q("D") == "0,0,0")
        self.emit("4e xback 0: fault 0x20 latched, DAC 0, HV off", ok and off, "%s; v_mon %.3f kV" % (_st(s["st"]), s["kv"]))
        self.ctx.xray_stop()
        self.q("C")
        for k, v in orig.items():
            if v is not None:
                self.must("c %s %s" % (k, v))
        self.emit("4 X-ray settings restored (RAM)", True, ", ".join("%s=%s" % kv for kv in orig.items()))

    def test_outer(self):
        self.ctx.step("5a. Outer interlock: watchdog process killed at 0.6 kV -> client sends z")
        self.must("e 1")
        self.set_target(0.6)
        s = self.settle()
        self.no_fault(s, "outer 0.6 kV")
        self.ctx.wd_kill()
        t0 = time.monotonic()
        s, ok = self.wait(8.0, lambda s: self.q("E") == "0" and self.q("D") == "0,0,0")
        t_z = time.monotonic() - t0
        t_dec, s2 = self.decay_time()
        self.emit("5a watchdog killed: z within ~2.5 s, HV off, v_mon decays (24 V stays on)", ok and t_dec is not None,
                  "z after %.1f s; v_mon < 0.02 V %s" % (t_z, "after %.1f s more" % t_dec if t_dec is not None else "NOT reached"))
        r = self.q("e 1")
        self.emit("5a no watchdog: e 1 refused", r == "ERR PSU_WD", r)
        if r == "OK":
            self.q("e 0")
        ok = self.ctx.wd_restart()
        self.emit("5a watchdog restarted and re-armed", ok, json_state(self.ctx))
        if not ok:
            raise Abort("watchdog could not be restarted")

        self.ctx.step("5b. Outer interlock: trip at 0.4 kV -> 24 V cut")
        self.must("e 1")
        self.set_target(0.4)
        s = self.settle()
        self.no_fault(s, "outer 0.4 kV")
        self.ctx.wd_trip("stage 7 test: trip at 0.4 kV")
        t0 = time.monotonic()
        off = False
        while time.monotonic() - t0 < 5:
            m = self.ctx.wd_status().get("meas") or {}
            if m.get("on") is False:
                off = True
                break
            time.sleep(0.1)
        t_off = time.monotonic() - t0
        time.sleep(2.0)
        st = self.link.status()
        self.emit("5b trip: CH1 off, firmware sees the 24 V loss and shuts HV off", off and st.get("hv") == 0,
                  "CH1 off after %.2f s; %s" % (t_off, _st(st)))
        self.log("waiting 10 s with 24 V off ...")
        time.sleep(10.0)
        wd = self.ctx.wd()
        r1 = wd.reset()
        r2 = wd.power_on()
        time.sleep(2.0)
        self.q("C")
        s = self.sample()
        self.emit("5b reset + 24 V ON: HV off, DAC 0, v_mon back at 0, no fault",
                  r1.get("ok") and r2.get("ok") and not r2.get("error") and s["st"].get("hv") == 0 and
                  s["st"].get("flt") == 0 and self.q("D") == "0,0,0" and s["vmon"] < 0.03,
                  "reset=%s power_on=%s; v_mon %.4f V; %s" % (r1.get("error") or "OK", r2.get("error") or "OK",
                                                              s["vmon"], _st(s["st"])))

    def finish(self, aborted):
        self.ctx.step("Finishing: HV off, DAC 0")
        try:
            self.ctx.xray_stop()
        except Exception:  # noqa: BLE001
            pass
        try:
            self.hv_off_and_zero()
        except Exception as e:  # noqa: BLE001
            self.log("!! finish: %s" % e)
            self.zero()
        for k in ("xthr", "xback", "xtimeout"):
            v = self.orig_cfg.get(k)
            if v is not None:
                try:
                    self.q("c %s %s" % (k, v))
                except Exception:  # noqa: BLE001
                    pass
        self.log("left in RAM (not saved): dacmax=%d (%.2f kV cap), cable=1; a reset restores the EEPROM values"
                 % (self.cap_units, self.cap_kv))
        self.log("highest v_mon seen during the run: %.3f kV" % self.max_kv_seen)

    def report(self, aborted, path_dir):
        os.makedirs(path_dir, exist_ok=True)
        path = os.path.join(path_dir, "stage7_report_%s.md" % datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
        n_ok = sum(1 for _, ok, _ in self.rows if ok is True)
        n_bad = sum(1 for _, ok, _ in self.rows if ok is False)
        lines = ["# Stage 7 tests (TOF3000 connected, <= %.1f kV)" % self.test_max_kv, "",
                 "Date: %s, duration %.0f s" % (datetime.datetime.now().strftime("%Y-%m-%d %H:%M"), time.time() - self.t0),
                 "Limits: test max %.2f kV, cap %.2f kV (GUI + firmware dacmax %d), abort at v_mon %.2f kV" % (
                     self.test_max_kv, self.cap_kv, self.cap_units, self.abort_kv),
                 "Highest v_mon seen: %.3f kV" % self.max_kv_seen,
                 "Result: %d passed, %d failed%s" % (n_ok, n_bad, ("; ABORTED: %s" % aborted) if aborted else ""), "",
                 "| Test | Result | Detail |", "|---|---|---|"]
        for name, ok, d in self.rows:
            lines.append("| %s | %s | %s |" % (name, "PASS" if ok is True else ("FAIL" if ok is False else "info"),
                                               str(d).replace("|", "/")))
        with open(path, "w", encoding="utf-8") as fh:
            fh.write("\n".join(lines) + "\n")
        return path


def _st(st):
    return "hv=%s tgt=%s cur=%s flt=0x%X xh=%s xtrips=%s" % (st.get("hv"), st.get("tgt"), st.get("cur"),
                                                           st.get("flt", 0), st.get("xh"), st.get("xtrips"))


def json_state(ctx):
    try:
        s = ctx.wd_status()
        return "state %s, CH1 %s" % (s.get("state"), "ON" if (s.get("meas") or {}).get("on") else "OFF")
    except Exception as e:  # noqa: BLE001
        return repr(e)


def run(ctx, sim_source, report_dir, **kw):
    """Run everything; returns (report_path, aborted_reason_or_None)."""
    t = Stage7(ctx, sim_source=sim_source, **kw)
    aborted = None
    try:
        t.prepare()
        t.test_1b()
        t.test_firmware_cap()
        t.test_staircase()
        t.test_xray()
        t.test_outer()
    except Abort as e:
        aborted = str(e)
        t.emit("ABORTED", False, aborted)
    except Exception as e:  # noqa: BLE001 - anything unexpected: make safe first
        aborted = "unexpected error: %r" % (e,)
        t.zero()
        t.emit("ABORTED", False, aborted)
    finally:
        t.finish(aborted)
    path = t.report(aborted, report_dir)
    ctx.step("Done: %s" % ("ABORTED - " + aborted if aborted else "all steps run"))
    t.log("stage 7 report: %s" % path)
    return path, aborted
