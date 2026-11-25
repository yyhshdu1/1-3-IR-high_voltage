"""
test_stage7_sim.py - run the GUI's stage 7 test sequence against a simulated board, TOF3000 and
24 V supply (no hardware).  The real App (window hidden), HVLink, WatchdogClient, PsuWatchdog and
stage7_tests code run; only the serial port, the SPD3303X and the dose meter are simulated.

    python test_stage7_sim.py            # normal TOF3000: expect no failures except none
    python test_stage7_sim.py --bad-1b   # TOF3000 ignores HV_enable: the run must abort at 1b
    python test_stage7_sim.py --hot      # v_mon reads 30 % high: the run must abort, never > 0.95 kV
"""
import sys
import threading
import time
import tkinter as tk

import hv_bringup_gui as gui
import psu_watchdog as pw
import stage7_tests as s7

BAD_1B = "--bad-1b" in sys.argv
HOT = "--hot" in sys.argv
U = s7.UNITS_PER_KV


class FakePSU(object):
    def __init__(self):
        self.on, self.v, self.i, self.alive = True, 24.0, 1.5, True
        self.idn = "FAKE SPD3303X"

    def status(self):
        return {"CH1_on": self.on, "CH1_cc": False, "mode": "independent"}

    def measure(self, ch):
        return {"v": 24.0 if self.on else 0.0, "i": 0.20 if self.on else 0.0, "p": 4.8 if self.on else 0.0}

    def voltage_setpoint(self, ch):
        return self.v

    def current_setpoint(self, ch):
        return self.i

    def output(self, ch, on):
        self.on = bool(on)

    def close(self):
        pass


class FakeBoard(object):
    """Subset of HV_control FW 2.8 + a TOF3000 model (output follows the DAC when enabled)."""

    def __init__(self, powered):
        self.powered = powered
        self.t_boot = time.monotonic()
        self.hv = False
        self.tgt = self.cur = 0.0
        self.faults = 0x11
        self.pending = False
        self.cfg = {"board": "B", "scale": 0.9058, "dacmax": 59055, "up": 1000, "down": 4000, "timeout": 10,
                    "vprogmax": 10.5, "vprogtol": 0.1, "ethr": 100, "sthr": 2500, "spark": 0, "sparkwin": 60,
                    "backoff": 1968, "xthr": 0, "xback": 3936, "xtimeout": 0, "pid": 0, "cable": 0, "pol": 1,
                    "name": "TOF+", "strict": 0, "ack": 1}
        self.x_level, self.x_hold, self.x_last, self.x_trips, self.x_step = 0, False, time.monotonic(), 0, None
        self.last_host = time.monotonic()
        self.tof_kv = 0.0
        self.adc_bad = 0
        self.lock = threading.Lock()
        threading.Thread(target=self._loop, daemon=True).start()

    def raw(self):
        return int(round(self.cur * self.cfg["scale"]))

    def safe_shutdown(self, flag):
        self.faults |= flag
        self.pending = False
        self.tgt = self.cur = 0.0
        self.hv = False

    def _loop(self):
        t = time.monotonic()
        while True:
            time.sleep(0.01)
            now = time.monotonic()
            dt, t = now - t, now
            with self.lock:
                if self.cur < self.tgt:
                    self.cur = min(self.tgt, self.cur + self.cfg["up"] * dt)
                elif self.cur > self.tgt:
                    self.cur = max(self.tgt, self.cur - self.cfg["down"] * dt)
                if self.pending and self.cur == 0:
                    self.hv = False
                    self.pending = False
                if self.cfg["timeout"] and self.hv and not self.faults & 0x04 and \
                        now - self.last_host > self.cfg["timeout"]:
                    self.faults |= 0x04
                    self.tgt = 0
                    self.pending = True
                if self.cfg["xthr"] and self.cfg["xtimeout"] and self.hv and self.raw() > 0 and \
                        not self.faults & 0x40 and now - self.x_last > self.cfg["xtimeout"]:
                    self.faults |= 0x40
                    self.tgt = 0
                    self.pending = True
                if not self.powered():
                    self.adc_bad += 1
                    if self.adc_bad >= 5 and not self.faults & 0x10:
                        self.safe_shutdown(0x11)
                else:
                    self.adc_bad = 0
                enabled = self.hv or BAD_1B
                want = (self.cur / U) if (enabled and self.powered()) else 0.0
                self.tof_kv += (want - self.tof_kv) * min(1.0, dt / 0.3)

    def volts(self):
        if not self.powered():
            return [12.2878] * 5
        vmon = self.tof_kv / 3.0 * (1.3 if HOT else 1.0) + 0.017
        return [vmon, 0.017 + self.tof_kv * 0.002, self.cur / U / 3.0, 0.004, -2.991]

    def x_alarm(self, now):
        self.x_hold = True
        if not self.hv or (self.tgt <= 0 and self.cur <= 0):
            return
        if self.x_step is not None and now - self.x_step < 3.0:
            return
        self.x_step = now
        self.x_trips += 1
        if self.cfg["xback"] > 0:
            base = min(self.cur, self.tgt)
            self.tgt = max(0.0, base - self.cfg["xback"])
        else:
            self.faults |= 0x20
            self.tgt = 0
            self.pending = True

    def handle(self, line):
        now = time.monotonic()
        if "*" in line:
            line = line.split("*")[0]
        parts = line.split()
        op, arg = parts[0], (parts[1] if len(parts) > 1 else None)
        with self.lock:
            self.last_host = now
            if op == "?":
                return "HV_control v1.3 ready. Board B. FW 2.8. Name TOF+. Pol +"
            if op == "s":
                return ("S up=%d rst=0x20 n_por=1 n_bor=0 n_ext=0 n_wdt=0 n_sw=0 hv=%d pol=1 rly=1 int=0 tgt=%d cur=%d "
                        "raw=%d flt=0x%X ev=0 sp=0 backoffs=0 xr=%d xh=%d xtrips=%d dacfix=0" % (
                            now - self.t_boot, self.hv, self.tgt, self.cur, self.raw(), self.faults, self.x_level,
                            self.x_hold, self.x_trips))
            if op == "c" and arg is None:
                return "CFG " + " ".join("%s=%s" % kv for kv in self.cfg.items())
            if op == "c":
                key, val = parts[1], parts[2] if len(parts) > 2 else None
                if key not in self.cfg or val is None:
                    return "ERR ARG"
                if key == "dacmax" and (self.hv or self.raw()):
                    return "ERR HV_ON"
                self.cfg[key] = type(self.cfg[key])(float(val)) if not isinstance(self.cfg[key], str) else val
                return "OK"
            if op == "C":
                self.faults, self.pending, self.x_last = 0, False, now
                return "OK"
            if op == "e":
                if arg == "1":
                    if self.faults:
                        return "ERR FAULT"
                    if self.raw():
                        return "ERR DAC_NZ"
                    self.hv, self.pending, self.x_last = True, False, now
                else:
                    self.hv, self.pending, self.tgt, self.cur = False, False, 0.0, 0.0
                return "OK"
            if op == "E":
                return "1" if self.hv else "0"
            if op == "d":
                try:
                    n = int(arg)
                except (TypeError, ValueError):
                    return "ERR ARG"
                if n < 0 or n > self.cfg["dacmax"]:
                    return "ERR RANGE"
                if n > 0 and self.faults:
                    return "ERR FAULT"
                if self.x_hold and self.cfg["xthr"] and n > self.tgt:
                    return "ERR XRAY"
                self.tgt = float(n)
                return "OK"
            if op == "D":
                return "%d,%d,%d" % (self.tgt, self.cur, self.raw())
            if op == "z":
                self.pending, self.tgt, self.cur, self.hv = False, 0.0, 0.0, False
                return "OK"
            if op == "x":
                self.x_level, self.x_last = int(arg), now
                if self.cfg["xthr"] == 0:
                    self.x_hold = False
                elif self.x_level >= self.cfg["xthr"]:
                    self.x_alarm(now)
                elif self.x_level * 5 < self.cfg["xthr"] * 4:
                    self.x_hold = False
                return "OK"
            if op == "k":
                return "OK"
            if op == "b":
                return ",".join("%.4f" % v for v in self.volts())
            if op == "a":
                return ",".join(str(min(65535, int((v + 12.288) / 24.576 * 65535))) for v in self.volts())
            if op == "P":
                return "1"
            if op == "m":
                return "M ev=0 sp=0 imax=0 vmin=0 ibase=0 evtotal=0 sptotal=0"
            return "ERR CMD"


class FakeSerial(object):
    is_open = True
    timeout = 1

    def __init__(self, board):
        self.board, self.reply = board, b""

    def reset_input_buffer(self):
        self.reply = b""

    def write(self, b):
        self.reply = (self.board.handle(b.decode().strip()) + "\n").encode()

    def readline(self):
        r, self.reply = self.reply, b""
        return r

    def close(self):
        pass


# ------------------------------------------------------------------ fake watchdog process
psu = FakePSU()
board = FakeBoard(lambda: psu.on)
wd_box = {}


def start_wd():
    stop, ready = threading.Event(), threading.Event()
    wd = pw.PsuWatchdog(lambda: psu, log=lambda m: print("[watchdog] " + m, flush=True))
    th = threading.Thread(target=wd.serve, args=(pw.DEFAULT_ADDR, stop, ready), daemon=True)
    th.start()
    ready.wait(3)
    wd_box.update(wd=wd, stop=stop, th=th)
    time.sleep(0.5)
    return True


def kill_wd(ctx_self):
    ctx_self.app.log("killing the (simulated) watchdog")
    wd_box["stop"].set()
    wd_box["th"].join(3)


if pw.is_running():
    sys.exit("a real watchdog is running on the default port - stop it first")

gui.messagebox.askokcancel = lambda *a, **k: True
gui.messagebox.showwarning = lambda *a, **k: None
gui.messagebox.showinfo = lambda *a, **k: None
gui.GuiCtx.wd_kill = kill_wd
gui.GuiCtx.make_gmc = lambda self, port: s7.SimDose(0.12)

root = tk.Tk()
root.withdraw()
app = gui.App(root, opts={"xport": "SIMGMC"})
app._wd_spawn = lambda: start_wd()
app.log = (lambda orig: (lambda m: (print(m, flush=True), orig(m))))(app.log)
start_wd()

# "connect": what on_connect's worker does, with the fake serial port
lk = app.link
lk.ser, lk.port = FakeSerial(board), "SIM"
lk.generation += 1
lk.query("c ack 1", quiet=True)
banner = lk.query("?", quiet=True)
app.poll_stop.clear()
app.set_connected(True)
app.poll_thread = threading.Thread(target=app._poll_loop, daemon=True)
app.poll_thread.start()
app._wd_attach(banner)

result = {}
max_kv = [0.0]


def watch_kv():
    while not result.get("done"):
        max_kv[0] = max(max_kv[0], board.tof_kv)
        time.sleep(0.02)


threading.Thread(target=watch_kv, daemon=True).start()


def start():
    if not (app.wd and app.wd.armed):
        root.after(300, start)
        return
    app.run_stage7_tests()
    root.after(1000, wait_done)


def wait_done():
    if app.job_thread is not None and app.job_thread.is_alive():
        root.after(1000, wait_done)
        return
    result["done"] = True
    rows = [app.tree7.item(i, "values") for i in app.tree7.get_children()]
    result["rows"] = rows
    app.poll_stop.set()
    if app.wd:
        app.wd.stop()
    root.destroy()


root.after(1000, start)
root.mainloop()
if "stop" in wd_box:
    wd_box["stop"].set()

rows = result.get("rows", [])
fails = [r for r in rows if r[1] == "FAIL"]
print("\n==== %d rows, %d FAIL; highest simulated TOF3000 output %.3f kV" % (len(rows), len(fails), max_kv[0]))
for r in rows:
    print("%-5s %s | %s" % (r[1], r[0], r[2]))
ok = max_kv[0] < 0.95
if BAD_1B:
    ok = ok and any(r[0] == "ABORTED" and "1b FAILED" in r[2] for r in rows) and max_kv[0] < 0.6
elif HOT:
    ok = ok and any(r[0] == "ABORTED" for r in rows)
else:
    ok = ok and not fails
print("SIM TEST %s" % ("PASSED" if ok else "FAILED"))
sys.exit(0 if ok else 1)
