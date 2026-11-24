"""
test_xray_bench.py - checks the FW 2.8 X-ray guard and the two current thresholds.

ONLY FOR THE BENCH BUILD (HV_control_v2_bench.ino) ON A BARE NANO EVERY, NO HV BOARD ATTACHED.
(The test injects current events with the bench-only 'X <ms>' command.)

    python test_xray_bench.py COM4

Prints PASS/FAIL per check.  It changes RAM configuration only (never 'c save'), so a reset
restores the saved values.
"""
import sys
import time

from hv_link import HVLink

results = []


def check(name, ok, detail=""):
    results.append(ok)
    print("[%s] %s %s" % ("PASS" if ok else "FAIL", name, detail))


def wait_cur(link, lo=None, hi=None, timeout=10.0):
    t_end = time.time() + timeout
    while time.time() < t_end:
        tgt, cur, _ = link.dstate()
        if (lo is None or cur >= lo) and (hi is None or cur <= hi):
            return cur
        time.sleep(0.1)
    return link.dstate()[1]


def main(port):
    link = HVLink(log=lambda m: None)
    link.open(port)
    banner = link.query("?")
    check("firmware 2.8", "FW 2.8" in banner, banner)
    link.query("C")
    link.query("z")

    # ---- FW 2.7: v1 raw-pin commands are refused (h0 meant ON in v1), HV stays off ------------------
    check("old h 0 refused", link.query("h 0") == "ERR USE_E")
    check("old h 1 refused", link.query("h 1") == "ERR USE_E")
    check("old H refused", link.query("H") == "ERR USE_E")
    check("HV still off after the refused h commands", link.query("E") == "0")

    # ---- FW 2.8: the clock bug.  FW <= 2.7 read millis() before the serial port, so a 1 ms tick made --------
    # ---- now - last_host_ms underflow and the heartbeat timeout tripped at random while commands arrived. ----
    # ---- Many commands, short timeout, must NEVER trip; then silence must trip it. -------------------------
    link.cmd_ok("c timeout 2")
    link.cmd_ok("e 1")
    t0 = time.time()
    n_cmd = 0
    while time.time() - t0 < 15:
        link.query("k", quiet=True)
        n_cmd += 1
        time.sleep(0.03)
    flt_busy = link.status()["flt"]
    check("c timeout 2: %d commands in 15 s never trip the heartbeat timeout" % n_cmd, flt_busy == 0, "flt=0x%X" % flt_busy)
    time.sleep(3.0)                                     # now stay silent: the timeout must fire
    flt_quiet = link.status()["flt"]
    check("c timeout 2: 3 s of silence does trip it (0x04)", bool(flt_quiet & 0x04), "flt=0x%X" % flt_quiet)
    link.cmd_ok("c timeout 0")                          # the rest of the test assumes no heartbeat timeout
    link.query("z")
    link.query("C")

    # same mechanism for the X-ray monitor-lost timeout (x_last_ms): steady reports must never trip 0x40
    link.cmd_ok("c xthr 40")
    link.cmd_ok("c xtimeout 2")
    link.cmd_ok("e 1")
    link.cmd_ok("d 3000")
    t0 = time.time()
    n_x = 0
    while time.time() - t0 < 15:
        link.query("x 0", quiet=True)
        n_x += 1
        time.sleep(0.03)
    flt_x = link.status()["flt"]
    check("c xtimeout 2: %d x reports in 15 s never trip the monitor-lost fault" % n_x, flt_x == 0, "flt=0x%X" % flt_x)
    link.cmd_ok("c xtimeout 0")
    link.cmd_ok("c xthr 0")
    link.query("z")
    link.query("C")

    # ---- per-supply polarity and name (RAM only here; "c save" would persist them) ------------
    # ---- FW 2.5: pid lock, polarity guard, cable fault ---------------------------------------------
    cfg0 = link.config()
    check("pid / cable keys visible, default locked", "pid" in cfg0 and "cable" in cfg0 and cfg0["pid"] == 0, "pid=%s cable=%s" % (cfg0.get("pid"), cfg0.get("cable")))
    link.cmd_ok("e 1")
    check("r 0 refused while pid = 0", link.query("r 0") == "ERR PIDOFF")
    link.cmd_ok("c pid 1")
    check("r 0 allowed after c pid 1", link.query("r 0") == "OK")
    link.cmd_ok("r 1")
    link.cmd_ok("c pid 0")
    link.cmd_ok("e 0")
    link.cmd_ok("U 3000")                              # bench: +1.1 V on CH0
    check("p refused while v_mon is not decayed", link.query("p 1") == "ERR VMON")
    check("c pol refused while v_mon is not decayed", link.query("c pol 1") == "ERR VMON")
    link.cmd_ok("U 0")
    check("p allowed again once v_mon is back at 0", link.query("p 0") == "OK")
    link.cmd_ok("c cable 1")
    link.cmd_ok("e 1")
    link.cmd_ok("d 10000")
    wait_cur(link, lo=9900)
    link.cmd_ok("W 3000")                              # bench: CH0 and CH1 float to full scale
    t0 = time.time()
    while time.time() - t0 < 3 and not (link.status()["flt"] & 0x80):
        link.query("k", quiet=True)
        time.sleep(0.1)
    st = link.status()
    check("floating monitor inputs with HV on -> fault 0x80 and ramp down", bool(st["flt"] & 0x80) and st["tgt"] == 0,
          "flt=0x%X tgt=%s after %.2f s" % (st["flt"], st["tgt"], time.time() - t0))
    time.sleep(3.5)
    link.cmd_ok("C")
    link.cmd_ok("c cable 0")
    link.cmd_ok("e 1")
    link.cmd_ok("d 5000")
    wait_cur(link, lo=4900)
    link.cmd_ok("W 1500")
    time.sleep(2.0)
    check("cable check off: no fault", link.status()["flt"] == 0, "flt=0x%X" % link.status()["flt"])
    link.query("z")
    link.query("C")

    pol0 = link.config().get("pol")
    link.cmd_ok("c pol 1")
    check("c pol 1 sets the pin and the banner", link.query("P") == "1" and "Pol +" in link.query("?"), link.query("?"))
    link.cmd_ok("c pol 0")
    check("c pol 0 sets the pin and the banner", link.query("P") == "0" and "Pol -" in link.query("?"), link.query("?"))
    check("c pol rejects 2", link.query("c pol 2") == "ERR ARG")
    link.cmd_ok("c name TOF+")
    check("name stored and shown", link.config().get("name") == "TOF+" and "Name TOF+" in link.query("?"), link.query("?"))
    check("name with bad character rejected", link.query("c name a/b") == "ERR ARG")
    check("name too long rejected", link.query("c name ABCDEFGHI") == "ERR ARG")
    link.cmd_ok("c name -")
    check("name cleared", link.config().get("name") == "-", str(link.config().get("name")))
    link.cmd_ok("e 1")
    link.cmd_ok("d 1000")
    check("c pol refused while HV is on", link.query("c pol 1") == "ERR BUSY")
    link.query("z")
    time.sleep(0.5)
    link.query("c pol %d" % int(pol0 if isinstance(pol0, int) else 0))
    link.query("C")
    for c in ("c up 20000", "c down 20000", "c spark 0", "c timeout 0", "c ack 1",
              "c xthr 40", "c xback 3936", "c xtimeout 0", "c sthr 1300", "c ethr 100"):
        link.cmd_ok(c)
    cfg = link.config()
    check("new config keys visible", all(k in cfg for k in ("sthr", "xthr", "xback", "xtimeout")),
          "sthr=%s xthr=%s xback=%s xtimeout=%s" % (cfg.get("sthr"), cfg.get("xthr"), cfg.get("xback"), cfg.get("xtimeout")))

    # ---- two current thresholds -------------------------------------------------------------
    link.cmd_ok("e 1")
    link.cmd_ok("d 10000")
    wait_cur(link, lo=9900)
    link.query("m")                                   # clear window counters
    st0 = link.status()
    link.cmd_ok("X 300")                              # +1500 counts: above both thresholds
    time.sleep(1.0)
    st1 = link.status()
    check("event above sthr counts in ev and sp", st1["ev"] == st0["ev"] + 1 and st1["sp"] == st0["sp"] + 1,
          "ev %s->%s sp %s->%s" % (st0["ev"], st1["ev"], st0["sp"], st1["sp"]))
    link.cmd_ok("c sthr 30000")                       # spark threshold out of reach
    link.cmd_ok("X 300")
    time.sleep(1.0)
    st2 = link.status()
    check("event below sthr counts only in ev", st2["ev"] == st1["ev"] + 1 and st2["sp"] == st1["sp"],
          "ev %s->%s sp %s->%s" % (st1["ev"], st2["ev"], st1["sp"], st2["sp"]))
    link.cmd_ok("c sthr 1300")

    # ---- spark back-off uses only the high threshold -----------------------------------------
    link.cmd_ok("c spark 2")
    link.cmd_ok("c backoff 1968")
    link.cmd_ok("d 20000")
    wait_cur(link, lo=19900)
    bo0 = link.status()["backoffs"]
    link.cmd_ok("X 200"); time.sleep(0.8)
    link.cmd_ok("X 200"); time.sleep(1.0)
    st = link.status()
    check("2 sparks in the window trigger one back-off", st["backoffs"] == bo0 + 1,
          "backoffs %s->%s tgt=%s" % (bo0, st["backoffs"], st["tgt"]))
    link.cmd_ok("c spark 0")

    # ---- X-ray alarm --------------------------------------------------------------------------
    link.cmd_ok("d 20000")
    wait_cur(link, lo=19900)
    link.cmd_ok("x 10")
    time.sleep(0.2)
    st = link.status()
    check("level below xthr: nothing happens", st["xh"] == 0 and st["xtrips"] == 0 and abs(st["tgt"] - 20000) < 1,
          "xr=%s xh=%s xtrips=%s tgt=%s" % (st["xr"], st["xh"], st["xtrips"], st["tgt"]))
    link.cmd_ok("x 80")
    time.sleep(0.2)
    st = link.status()
    check("alarm lowers the target below the output by xback", st["xh"] == 1 and st["xtrips"] == 1 and st["tgt"] <= 20000 - 3936 + 1,
          "xh=%s xtrips=%s tgt=%s" % (st["xh"], st["xtrips"], st["tgt"]))
    r = link.query("d 30000")
    check("increase refused during alarm", r == "ERR XRAY", repr(r))
    r = link.query("d 5000")
    check("decrease allowed during alarm", r == "OK", repr(r))
    link.cmd_ok("x 80")
    time.sleep(0.2)
    check("second alarm within 3 s does not step again", link.status()["xtrips"] == 1, "xtrips=%s" % link.status()["xtrips"])
    time.sleep(3.2)
    link.cmd_ok("x 80")
    time.sleep(0.2)
    check("alarm after 3 s steps again", link.status()["xtrips"] == 2, "xtrips=%s" % link.status()["xtrips"])
    link.cmd_ok("x 35")                               # between 80 % (32) and 100 % (40): hold stays
    check("hysteresis: still held at 35", link.status()["xh"] == 1)
    link.cmd_ok("x 20")
    check("released below 80 % of xthr", link.status()["xh"] == 0)
    r = link.query("d 8000")
    check("increase allowed again", r == "OK", repr(r))

    # ---- monitor lost ----------------------------------------------------------------------------
    link.cmd_ok("d 8000")
    wait_cur(link, lo=7900)
    link.cmd_ok("c xtimeout 3")
    link.cmd_ok("x 10")
    t0 = time.time()
    while time.time() - t0 < 6 and not (link.status()["flt"] & 0x40):
        link.query("k", quiet=True)                   # keep the host heartbeat alive, but send no 'x'
        time.sleep(0.3)
    st = link.status()
    check("monitor lost -> fault 0x40 and ramp down", bool(st["flt"] & 0x40) and st["tgt"] == 0,
          "flt=0x%X tgt=%s after %.1f s" % (st["flt"], st["tgt"], time.time() - t0))
    time.sleep(1.5)
    check("HV disabled after the ramp", link.status()["hv"] == 0)
    link.cmd_ok("c xtimeout 0")
    link.cmd_ok("C")

    # ---- xback 0: ramp down and latch --------------------------------------------------------------
    link.cmd_ok("c xback 0")
    link.cmd_ok("e 1")
    link.cmd_ok("d 10000")
    wait_cur(link, lo=9900)
    link.cmd_ok("x 80")
    time.sleep(1.5)
    st = link.status()
    check("xback 0 -> fault 0x20, target 0", bool(st["flt"] & 0x20) and st["tgt"] == 0,
          "flt=0x%X tgt=%s" % (st["flt"], st["tgt"]))

    link.query("z")
    link.query("C")
    link.close()
    print("\n%d/%d checks passed" % (sum(results), len(results)))
    return 0 if all(results) else 1


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(2)
    sys.exit(main(sys.argv[1]))
