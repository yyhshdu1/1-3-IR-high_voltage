"""
psu_watchdog.py - outermost interlock: switch the 24 V supply (SPD3303X CH1, which powers the
HV_control PCB and the TOF3000) OFF when the HV_control Arduino can no longer be controlled.

Why a separate process
    The Arduino serial port can only be open in one program, and opening it resets the Arduino.
    So the watchdog cannot check the Arduino itself.  The program that owns the port (the
    bring-up GUI, later the control program) runs a WatchdogClient that sends a heartbeat every
    0.5 s carrying "proof of control": how long ago the Arduino last answered, whether the port
    is open, whether HV is off.  This process owns the SPD3303X and cuts CH1 when that proof
    stops.  Because it is a separate process it also covers the control program crashing or
    hanging, which an in-program thread cannot.

What trips it (only while at least one client is ARMED)
    - no heartbeat from an armed client for --hb-timeout s (control program crashed / hung)
    - the client reports no reply from the Arduino for more than --ok-age-max s
    - the client reports its serial port closed (USB link lost) or an unexpected Arduino reset
    - a client sends "trip" (e.g. disarm requested while HV was not off)
    - lost contact with the SPD3303X (CH1 is switched off as soon as contact is back)
    - Ctrl-C on this program, or closing its console window, while armed
    A trip switches CH1 off, confirms it in the status word, and LATCHES: CH1 stays off until a
    client sends "reset" (needs every armed client fresh, Arduino answering, HV off) and then an
    explicit "power_on".
    CH1 found OFF while it should be on (front panel, supply fault) also latches TRIPPED.

What the clients do (WatchdogClient)
    - send "z" to their Arduino once when the watchdog trips, reports the SPD3303X lost, or
      stops answering for --reply-timeout s (so HV cannot come back by itself when 24 V returns)
    - optionally act as HVLink.permit: "e 1" and "d N>0" are refused unless the watchdog is
      ARMED, answering and has contact with the SPD3303X.

What it does NOT cover
    - the whole PC freezing or losing power (nothing on the PC runs): left to the firmware
      heartbeat ("c timeout") and the isolator default level on HV_enable.
    - this process being killed (Task Manager, crash): the clients see the watchdog stop
      answering and send "z", but CH1 stays as it was.
    - anything the Arduino reports as a firmware fault: the firmware handles its own interlocks.

Normal operation
    CH1 is meant to stay ON.  Starting the watchdog, arming, disarming and stopping it with no
    client armed never change CH1.  Only power_on / power_off / a trip do.

Run (normally started by the GUI):
    python psu_watchdog.py                      # CH1, expects 24 V +- 0.5 V setpoint
    python psu_watchdog.py --i-max 1.0          # also refuse power_on if the CH1 current limit > 1 A
    python psu_watchdog.py --status             # ask a running watchdog for its state
    python psu_watchdog.py --forget TOF+        # drop a stale client record (its program is gone)

Protocol: one JSON object per UDP datagram on 127.0.0.1:50733, every request gets a reply with
the full state.  Request fields: type (hb | arm | disarm | trip | reset | power_on | power_off |
status | forget), client (name), and the proof-of-control fields from WatchdogClient.fields().
"""
import argparse
import datetime
import json
import os
import socket
import sys
import threading
import time

DEFAULT_ADDR = ("127.0.0.1", 50733)
IDLE, ARMED, TRIPPED = "IDLE", "ARMED", "TRIPPED"
MAX_DATAGRAM = 8192
# Status keys that only change when the Arduino resets (reset cause + counters kept in .noinit RAM).
_RESET_KEYS = ("rst", "n_por", "n_bor", "n_ext", "n_wdt", "n_sw")

HERE = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(HERE, "logs")


def _now_str():
    return datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


# =============================================================================================
# Server
# =============================================================================================
class PsuWatchdog(object):
    """State machine + SPD3303X handling.  serve() adds the UDP socket around it.

    psu_factory: callable returning an opened SPD3303X-like object (status(), measure(ch),
                 voltage_setpoint(ch), current_setpoint(ch), output(ch, on), close()).
    """

    def __init__(self, psu_factory, ch="CH1", v_nom=24.0, v_tol=0.5, i_max=None,
                 hb_timeout=3.0, ok_age_max=5.0, poll_s=0.25,
                 reconnect_s=2.0, log=print, clock=time.monotonic):
        self.psu_factory = psu_factory
        self.ch = ch
        self.v_nom, self.v_tol, self.i_max = float(v_nom), float(v_tol), i_max
        self.hb_timeout, self.ok_age_max = float(hb_timeout), float(ok_age_max)
        self.poll_s, self.reconnect_s = poll_s, reconnect_s
        self.lock = threading.RLock()   # serve loop + console-close handler
        self.log, self.clock = log, clock

        self.state = IDLE
        self.reason = ""
        self.t_trip = None
        self.clients = {}            # name -> record (see _update_client)
        self.psu = None
        self.psu_ok = False
        self.psu_error = ""
        self.off_pending = False     # tripped but CH1 off not confirmed yet
        self.expected_on = None      # None until the first good read of the supply
        self.meas = {}               # last reading of the channel
        self.setpoints = (None, None)
        self._t_poll = -1e9
        self._t_reconnect = -1e9
        self._t_setpoints = -1e9

    # ------------------------------------------------------------------ supply
    def _connect(self):
        self._t_reconnect = self.clock()
        try:
            self.psu = self.psu_factory()
        except Exception as e:  # noqa: BLE001 - any VISA / driver error
            self.psu = None
            self._psu_failed("connect: %s" % e)
            return False
        self.psu_ok = True
        self.psu_error = ""
        self.log("SPD3303X connected: %s" % getattr(self.psu, "idn", "?"))
        return True

    def _psu_failed(self, err):
        was_ok = self.psu_ok
        self.psu_ok = False
        self.psu_error = str(err)
        if self.psu is not None:
            try:
                self.psu.close()
            except Exception:  # noqa: BLE001
                pass
            self.psu = None
        if was_ok:
            self.log("!! lost contact with the SPD3303X: %s" % err)
        if self.state == ARMED:
            self.trip("lost contact with the SPD3303X (%s)" % err)

    def _poll_psu(self):
        if self.psu is None:
            if self.clock() - self._t_reconnect >= self.reconnect_s:
                self._connect()
            if self.psu is None:
                return
        try:
            st = self.psu.status()
            m = self.psu.measure(self.ch)
            if self.clock() - self._t_setpoints > 2.0:
                self.setpoints = (self.psu.voltage_setpoint(self.ch), self.psu.current_setpoint(self.ch))
                self._t_setpoints = self.clock()
        except Exception as e:  # noqa: BLE001
            self._psu_failed(e)
            return
        on = bool(st.get("%s_on" % self.ch))
        self.meas = {"on": on, "cc": bool(st.get("%s_cc" % self.ch)), "mode": st.get("mode"),
                     "v": m.get("v"), "i": m.get("i"), "p": m.get("p")}
        if self.expected_on is None:
            self.expected_on = on
            self.log("%s output is %s at start; left as it is" % (self.ch, "ON" if on else "OFF"))
        elif self.expected_on and not on and self.state != TRIPPED:
            if self.state == ARMED:
                self.trip("%s output found OFF while it should be on (front panel or supply fault)" % self.ch)
            else:
                self.log("%s output went OFF (not armed)" % self.ch)
                self.expected_on = False
        elif not self.expected_on and on and self.state != TRIPPED:
            self.log("%s output was switched ON outside the watchdog" % self.ch)
            self.expected_on = True
        if self.off_pending:
            self._switch_off()

    def _switch_off(self):
        if self.psu is None:
            return False
        try:
            self.psu.output(self.ch, False)
        except Exception as e:  # noqa: BLE001
            self._psu_failed("switching %s off: %s" % (self.ch, e))
            return False
        self.off_pending = False
        self.expected_on = False
        self.meas["on"] = False
        self.log("%s output OFF (confirmed in the status word)" % self.ch)
        return True

    # ------------------------------------------------------------------ state
    def trip(self, reason):
        with self.lock:
            if self.state != TRIPPED:
                self.state = TRIPPED
                self.reason = reason
                self.t_trip = self.clock()
                self.log("!!! TRIP: %s -> switching %s off" % (reason, self.ch))
            else:
                self.log("(already tripped) %s" % reason)
            self.expected_on = False
            self.off_pending = True
            self._switch_off()

    def _armed_clients(self):
        return {n: c for n, c in self.clients.items() if c["armed"]}

    def _client_problem(self, name, c, now):
        """None if this armed client currently proves control of its Arduino, else why not."""
        age = now - c["t_hb"]
        if age > self.hb_timeout:
            return "%s: no heartbeat for %.1f s (control program stopped or hung)" % (name, age)
        if not c.get("link_open"):
            return "%s: serial port to the Arduino is closed (USB link lost?)" % name
        ok_age = c.get("ok_age")
        if ok_age is None or ok_age > self.ok_age_max:
            return "%s: no reply from the Arduino for %s s" % (
                name, "?" if ok_age is None else "%.1f" % ok_age)
        return None

    def _client_unsafe(self, name, c):
        if not c.get("safe"):
            return "%s: HV is not confirmed off (needs a fresh status with hv=0, DAC 0)" % name
        return None

    def check_clients(self):
        if self.state != ARMED:
            return
        now = self.clock()
        for name, c in self._armed_clients().items():
            problem = self._client_problem(name, c, now)
            if problem:
                self.trip(problem)
                return

    def tick(self):
        with self.lock:
            now = self.clock()
            if now - self._t_poll >= self.poll_s:
                self._t_poll = now
                self._poll_psu()
            self.check_clients()

    # --------------------------------------------------------------- requests
    def _update_client(self, name, msg):
        c = self.clients.setdefault(name, {"armed": False, "t_hb": -1e9, "t_first": self.clock()})
        c["t_hb"] = self.clock()
        for k in ("ok_age", "link_open", "safe", "up", "gen", "board", "port", "pid"):
            if k in msg:
                c[k] = msg[k]
        return c

    def handle(self, msg):
        """Process one request dict, return the reply dict."""
        with self.lock:
            return self._handle(msg)

    def _handle(self, msg):
        if not isinstance(msg, dict):
            return self.snapshot(ok=False, error="request must be a JSON object")
        typ = msg.get("type")
        name = str(msg.get("client", ""))[:32]
        if typ != "status" and typ != "forget" and not name:
            return self.snapshot(ok=False, error="missing client name")
        try:
            fn = getattr(self, "_req_" + str(typ))
        except AttributeError:
            return self.snapshot(ok=False, error="unknown request type %r" % typ)
        if typ not in ("status", "forget"):
            self._update_client(name, msg)
        ok, err = fn(name, msg)
        return self.snapshot(ok=ok, error=err)

    def _req_hb(self, name, msg):
        self.check_clients()
        return True, ""

    def _req_status(self, name, msg):
        return True, ""

    def _req_arm(self, name, msg):
        c = self.clients[name]
        problem = self._client_problem(name, c, self.clock())
        if problem:
            return False, "cannot arm: " + problem
        if not c["armed"]:
            c["armed"] = True
            self.log("client %s ARMED (port %s, board %s)" % (name, c.get("port"), c.get("board")))
        if self.state == IDLE:
            self.state = ARMED
            self.reason = ""
        if self.state == TRIPPED:
            return True, "armed, but the watchdog is TRIPPED: %s (reset needed)" % self.reason
        return True, ""

    def _req_disarm(self, name, msg):
        c = self.clients[name]
        if not c["armed"]:
            return True, ""
        unsafe = self._client_unsafe(name, c)
        if unsafe and self.state != TRIPPED:
            self.trip("disarm requested while HV not confirmed off (%s)" % unsafe)
        c["armed"] = False
        self.log("client %s disarmed" % name)
        if not self._armed_clients() and self.state == ARMED:
            self.state = IDLE
        return True, ""

    def _req_trip(self, name, msg):
        self.trip("%s: %s" % (name, str(msg.get("reason", "trip requested"))[:200]))
        return True, ""

    def _req_reset(self, name, msg):
        if self.state != TRIPPED:
            return True, "not tripped"
        if not self.psu_ok or self.off_pending:
            return False, "cannot reset: %s off not confirmed / SPD3303X not reachable (%s)" % (
                self.ch, self.psu_error)
        now = self.clock()
        armed = self._armed_clients()
        if name not in armed:
            return False, "cannot reset: client %s is not armed (arm first)" % name
        for n, c in armed.items():
            problem = self._client_problem(n, c, now) or self._client_unsafe(n, c)
            if problem:
                return False, "cannot reset: " + problem
        self.log("reset by %s (was: %s); %s stays OFF until power_on" % (name, self.reason, self.ch))
        self.state = ARMED
        self.reason = ""
        self.t_trip = None
        return True, ""

    def _req_power_on(self, name, msg):
        if self.state != ARMED:
            return False, "cannot power on: watchdog is %s%s" % (
                self.state, (" (%s)" % self.reason) if self.reason else " (no armed client)")
        if not self.psu_ok:
            return False, "cannot power on: SPD3303X not reachable (%s)" % self.psu_error
        now = self.clock()
        armed = self._armed_clients()
        if name not in armed:
            return False, "cannot power on: client %s is not armed" % name
        for n, c in armed.items():
            problem = self._client_problem(n, c, now) or self._client_unsafe(n, c)
            if problem:
                return False, "cannot power on: " + problem
        try:
            v_set = self.psu.voltage_setpoint(self.ch)
            i_set = self.psu.current_setpoint(self.ch)
        except Exception as e:  # noqa: BLE001
            self._psu_failed(e)
            return False, "cannot power on: %s" % e
        self.setpoints = (v_set, i_set)
        if abs(v_set - self.v_nom) > self.v_tol:
            return False, "cannot power on: %s voltage setpoint %.3f V is not %.2f +- %.2f V" % (
                self.ch, v_set, self.v_nom, self.v_tol)
        if self.i_max is not None and i_set > self.i_max + 1e-6:
            return False, "cannot power on: %s current limit %.3f A is above --i-max %.3f A" % (
                self.ch, i_set, self.i_max)
        try:
            self.psu.output(self.ch, True)
        except Exception as e:  # noqa: BLE001
            self._psu_failed(e)
            return False, "power on failed: %s" % e
        self.expected_on = True
        self.meas["on"] = True
        self.log("%s output ON by %s (%.3f V, limit %.3f A)" % (self.ch, name, v_set, i_set))
        return True, ""

    def _req_power_off(self, name, msg):
        if not self.psu_ok:
            self.off_pending = True
            return False, "SPD3303X not reachable; will switch off as soon as it is back"
        self.expected_on = False
        if not self._switch_off():
            return False, "switch off failed: %s" % self.psu_error
        self.log("%s output OFF by %s (manual)" % (self.ch, name))
        return True, ""

    def _req_forget(self, name, msg):
        target = str(msg.get("target", ""))
        if target in self.clients:
            del self.clients[target]
            self.log("client record %s forgotten" % target)
            if not self._armed_clients() and self.state == ARMED:
                self.state = IDLE
            return True, ""
        return False, "no client %r" % target

    def snapshot(self, ok=True, error=""):
        now = self.clock()
        clients = {}
        for n, c in self.clients.items():
            clients[n] = {"armed": c["armed"], "hb_age": round(now - c["t_hb"], 2),
                          "ok_age": c.get("ok_age"), "link_open": c.get("link_open"),
                          "safe": c.get("safe"), "port": c.get("port"), "board": c.get("board")}
        return {"ok": ok, "error": error, "state": self.state, "reason": self.reason,
                "ch": self.ch, "psu_ok": self.psu_ok, "psu_error": self.psu_error,
                "off_pending": self.off_pending, "expected_on": self.expected_on,
                "meas": self.meas, "v_set": self.setpoints[0], "i_set": self.setpoints[1],
                "v_nom": self.v_nom, "clients": clients, "pid": os.getpid(), "t": time.time()}

    # ------------------------------------------------------------------ serve
    def serve(self, addr=DEFAULT_ADDR, stop_evt=None, ready_evt=None):
        sock = bind_socket(addr)
        sock.settimeout(0.05)
        self.addr = sock.getsockname()
        self.log("listening on %s:%d" % self.addr)
        if ready_evt is not None:
            ready_evt.set()
        self._connect()
        try:
            while stop_evt is None or not stop_evt.is_set():
                try:
                    data, peer = sock.recvfrom(MAX_DATAGRAM)
                except socket.timeout:
                    data = None
                except ConnectionResetError:   # Windows: ICMP port unreachable from an old client
                    data = None
                if data:
                    try:
                        msg = json.loads(data.decode("utf-8"))
                    except ValueError:
                        reply = self.snapshot(ok=False, error="bad JSON")
                    else:
                        reply = self.handle(msg)
                    try:
                        sock.sendto(json.dumps(reply).encode("utf-8"), peer)
                    except OSError:
                        pass
                self.tick()
        except KeyboardInterrupt:
            if self.state == ARMED:
                self.trip("watchdog stopped (Ctrl-C) while armed")
            raise
        finally:
            sock.close()
            if self.psu is not None:
                try:
                    self.psu.close()
                except Exception:  # noqa: BLE001
                    pass


def bind_socket(addr):
    """UDP socket bound exclusively, so a second watchdog on the same port fails loudly."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):          # Windows
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
    try:
        sock.bind(addr)
    except OSError:
        sock.close()
        raise
    return sock


# =============================================================================================
# Client (runs inside the program that owns the HVLink)
# =============================================================================================
class WatchdogError(Exception):
    pass


def request(msg, addr=DEFAULT_ADDR, timeout=1.0):
    """Send one request and wait for the reply (raises WatchdogError if none)."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(timeout)
    try:
        sock.sendto(json.dumps(msg).encode("utf-8"), addr)
        data, _ = sock.recvfrom(MAX_DATAGRAM)
        return json.loads(data.decode("utf-8"))
    except (OSError, ValueError) as e:
        raise WatchdogError("watchdog at %s:%d not answering (%s)" % (addr[0], addr[1], e))
    finally:
        sock.close()


def is_running(addr=DEFAULT_ADDR):
    try:
        request({"type": "status"}, addr, timeout=0.5)
        return True
    except WatchdogError:
        return False


class WatchdogClient(threading.Thread):
    """Heartbeat sender for one HVLink.

    link     : HVLink (uses .lock, .is_open, .last_ok, .last_status, .generation, .status(),
               .emergency_stop(), .port)
    name     : unique client name, e.g. the board name from "c name" (TOF+ / TOF-) or the port
    on_update: optional callable(reply_or_None, event) called after every heartbeat (any thread)
    """

    def __init__(self, link, name, addr=DEFAULT_ADDR, period=0.5, reply_timeout=2.0,
                 status_max_age=2.0, log=print, on_update=None, board=None):
        threading.Thread.__init__(self, daemon=True)
        self.link, self.name, self.addr = link, str(name)[:32], addr
        self.period, self.reply_timeout, self.status_max_age = period, reply_timeout, status_max_age
        self.log, self.on_update, self.board = log, on_update, board
        self.stop_evt = threading.Event()
        self.armed = False
        self.reply = None
        self.t_reply = time.monotonic()
        self.event = ""              # "" = all good; otherwise why HV must not be on
        self._z_event = ""
        self._trip_sent = ""
        self._prev = None            # (generation, up) of the previous status seen
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.02)

    # ---------------------------------------------------------- proof of control
    def fields(self):
        now = time.monotonic()
        lk = self.link
        ok_age = (now - lk.last_ok) if lk.last_ok else None
        safe = False
        up = None
        ls = lk.last_status
        if ls is not None:
            t, gen, st = ls
            up = st.get("up")
            safe = (gen == lk.generation and now - t <= self.status_max_age and
                    not st.get("hv") and not st.get("tgt") and not st.get("cur") and not st.get("raw"))
        return {"client": self.name, "ok_age": None if ok_age is None else round(ok_age, 2),
                "link_open": bool(lk.is_open), "safe": bool(safe), "up": up,
                "gen": lk.generation, "port": getattr(lk, "port", None), "board": self.board,
                "pid": os.getpid()}

    def _refresh_status(self, wait=False):
        """Poll 's' ourselves if nobody else has recently.  The heartbeat loop never waits for the
        link lock; requests (arm / reset / power_on) wait up to 3 s so they carry a fresh status."""
        lk = self.link
        ls = lk.last_status
        if ls is not None and time.monotonic() - ls[0] < self.period:
            return
        if not (lk.lock.acquire(timeout=3.0) if wait else lk.lock.acquire(blocking=False)):
            return
        try:
            if lk.is_open:
                try:
                    lk.status(quiet=True)
                except Exception:  # noqa: BLE001 - HVError: ok_age grows, the watchdog decides
                    pass
        finally:
            lk.lock.release()

    def _check_link(self):
        """Report a closed port or a spontaneous reset.  Skipped while the link lock is held
        (HVLink.open() closes and reopens the port inside the lock)."""
        lk = self.link
        if not lk.lock.acquire(blocking=False):
            return None
        try:
            if not lk.is_open:
                return "serial port to the Arduino closed (USB link lost?)"
            ls = lk.last_status
            if ls is not None:
                gen, st = ls[1], ls[2]
                sig = tuple(st.get(k) for k in _RESET_KEYS)
                up = st.get("up")
                prev = self._prev
                self._prev = (gen, up, sig)
                # Same connection (no reopen by us) but 'up' went back, or the reset cause / the
                # reset counters changed: the Arduino reset by itself (spark, brown-out, watchdog).
                if prev is not None and prev[0] == gen and (
                        (up is not None and prev[1] is not None and up < prev[1]) or sig != prev[2]):
                    return "Arduino reset by itself (up %s -> %s s, rst=0x%X, n_por=%s n_bor=%s n_ext=%s n_wdt=%s n_sw=%s)" % (
                        prev[1], up, st.get("rst", 0), st.get("n_por"), st.get("n_bor"), st.get("n_ext"),
                        st.get("n_wdt"), st.get("n_sw"))
            return None
        finally:
            lk.lock.release()

    # ------------------------------------------------------------- requests
    def _request(self, typ, **extra):
        msg = self.fields()
        msg.update(extra)
        msg["type"] = typ
        reply = request(msg, self.addr)
        self.reply, self.t_reply = reply, time.monotonic()
        return reply

    def arm(self):
        self._refresh_status(wait=True)
        reply = self._request("arm")
        if reply.get("ok"):
            self.armed = True
            self._z_event = ""
            self._trip_sent = ""
        return reply

    def disarm(self):
        """Clean release.  Polls a fresh status first; the watchdog trips if HV is not off."""
        try:
            with self.link.lock:
                if self.link.is_open:
                    self.link.status(quiet=True)
        except Exception:  # noqa: BLE001
            pass
        self.armed = False
        return self._request("disarm")

    def reset(self):
        self._refresh_status(wait=True)
        return self._request("reset")

    def power_on(self):
        self._refresh_status(wait=True)
        return self._request("power_on")

    def power_off(self):
        return self._request("power_off")

    def stop(self):
        self.stop_evt.set()

    # ---------------------------------------------------------------- permit
    def permit(self, cmd):
        """For HVLink.permit: None if HV may be raised, else the reason (uses cached state only)."""
        if not self.armed:
            return "supply watchdog not armed"
        if self.event:
            return self.event
        r = self.reply
        if r is None or time.monotonic() - self.t_reply > self.reply_timeout:
            return "supply watchdog not answering"
        if r.get("state") != ARMED:
            return "supply watchdog is %s %s" % (r.get("state"), r.get("reason", ""))
        if not r.get("psu_ok"):
            return "supply watchdog has no contact with the SPD3303X"
        return None

    # ------------------------------------------------------------------ loop
    def _drain(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(MAX_DATAGRAM)
            except (socket.timeout, BlockingIOError):
                return
            except OSError:          # Windows: ConnectionResetError when nobody listens
                return
            try:
                self.reply, self.t_reply = json.loads(data.decode("utf-8")), time.monotonic()
            except ValueError:
                pass

    def _evaluate(self):
        if not self.armed:
            return ""
        r = self.reply
        if time.monotonic() - self.t_reply > self.reply_timeout:
            return "supply watchdog not answering (no reply for > %.1f s)" % self.reply_timeout
        if r is None:
            return ""
        if r.get("state") == TRIPPED:
            return "supply watchdog TRIPPED: %s" % r.get("reason", "")
        if not r.get("psu_ok"):
            return "supply watchdog lost the SPD3303X: %s" % r.get("psu_error", "")
        return ""

    def _zero_arduino(self, why):
        self.log("!! %s -> sending z to the Arduino (%s)" % (why, self.name))
        try:
            reply = self.link.emergency_stop()
            self.log("z -> %s" % reply)
        except Exception as e:  # noqa: BLE001
            self.log("z failed: %s" % e)

    def run(self):
        while not self.stop_evt.is_set():
            t0 = time.monotonic()
            self._refresh_status()
            problem = self._check_link() if self.armed else None
            if problem and problem != self._trip_sent:
                self._trip_sent = problem
                self.log("!! %s: %s -> asking the watchdog to trip" % (self.name, problem))
                msg = self.fields()
                msg.update({"type": "trip", "reason": problem})
            else:
                msg = self.fields()
                msg["type"] = "hb"
            try:
                self.sock.sendto(json.dumps(msg).encode("utf-8"), self.addr)
            except OSError:
                pass
            self.stop_evt.wait(0.05)
            self._drain()
            event = self._evaluate()
            if event and event != self._z_event:
                self._z_event = event
                self._zero_arduino(event)
            if not event:
                self._z_event = ""
            self.event = event
            if self.on_update is not None:
                try:
                    self.on_update(self.reply, event)
                except Exception:  # noqa: BLE001
                    pass
            self.stop_evt.wait(max(0.0, self.period - (time.monotonic() - t0)))
        self.sock.close()


# =============================================================================================
# CLI
# =============================================================================================
def install_close_handler(wd):
    """Windows: closing the console window, logoff or shutdown trips the watchdog while armed
    (Windows allows a few seconds before it kills the process).  Returns the callback, which
    must be kept referenced."""
    if os.name != "nt":
        return None
    import ctypes
    from ctypes import wintypes
    handler_type = ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.DWORD)

    def handler(event):
        if event in (2, 5, 6):            # CTRL_CLOSE_EVENT, CTRL_LOGOFF_EVENT, CTRL_SHUTDOWN_EVENT
            with wd.lock:
                if wd.state == ARMED:
                    wd.trip("watchdog window closed / logoff / shutdown while armed")
                wd.log("console closing (event %d), state %s" % (event, wd.state))
        return False                      # pass on: Ctrl-C still raises KeyboardInterrupt
    cb = handler_type(handler)
    ctypes.windll.kernel32.SetConsoleCtrlHandler(cb, True)
    return cb


def _make_logger(path):
    fh = open(path, "a", encoding="utf-8")
    lock = threading.Lock()

    def log(msg):
        line = "%s  %s" % (_now_str(), msg)
        with lock:
            print(line, flush=True)
            fh.write(line + "\n")
            fh.flush()
    return log


def main(argv=None):
    ap = argparse.ArgumentParser(description="SPD3303X 24 V outer interlock for HV_control")
    ap.add_argument("-r", "--resource", help="VISA resource of the SPD3303X (default: the single Siglent USB unit)")
    ap.add_argument("--ch", default="CH1", choices=("CH1", "CH2"), help="output that feeds the PCB/TOF3000")
    ap.add_argument("--v-nom", type=float, default=24.0, help="expected voltage setpoint, V (default 24)")
    ap.add_argument("--v-tol", type=float, default=0.5, help="allowed setpoint deviation for power_on, V")
    ap.add_argument("--i-max", type=float, default=None, help="refuse power_on if the current limit is above this, A")
    ap.add_argument("--hb-timeout", type=float, default=3.0, help="trip after this long without a heartbeat, s")
    ap.add_argument("--ok-age-max", type=float, default=5.0, help="trip when the Arduino has not answered for this long, s")
    ap.add_argument("--port", type=int, default=DEFAULT_ADDR[1], help="UDP port on 127.0.0.1")
    ap.add_argument("--status", action="store_true", help="print the state of a running watchdog and exit")
    ap.add_argument("--forget", metavar="CLIENT", help="drop a stale client record in a running watchdog")
    args = ap.parse_args(argv)
    addr = (DEFAULT_ADDR[0], args.port)

    if args.status or args.forget:
        msg = {"type": "forget", "target": args.forget} if args.forget else {"type": "status"}
        try:
            print(json.dumps(request(msg, addr), indent=1))
        except WatchdogError as e:
            print(e)
            return 1
        return 0

    from spd3303x import SPD3303X     # pyvisa only needed here

    os.makedirs(LOG_DIR, exist_ok=True)
    log_path = os.path.join(LOG_DIR, "psu_watchdog_%s.log" % datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
    log = _make_logger(log_path)
    log("psu_watchdog start: ch=%s v_nom=%.2f+-%.2f i_max=%s hb_timeout=%.1f ok_age_max=%.1f log=%s" % (
        args.ch, args.v_nom, args.v_tol, args.i_max, args.hb_timeout, args.ok_age_max, log_path))

    wd = PsuWatchdog(lambda: SPD3303X(args.resource).open(), ch=args.ch, v_nom=args.v_nom,
                     v_tol=args.v_tol, i_max=args.i_max, hb_timeout=args.hb_timeout,
                     ok_age_max=args.ok_age_max, log=log)
    close_cb = install_close_handler(wd)  # noqa: F841 - must stay referenced
    try:
        wd.serve(addr)
    except OSError as e:
        log("cannot listen on %s:%d (%s) - is another watchdog already running?" % (addr[0], addr[1], e))
        return 1
    except KeyboardInterrupt:
        log("stopped (state %s, %s output %s)" % (
            wd.state, wd.ch, "ON" if wd.meas.get("on") else "OFF"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
