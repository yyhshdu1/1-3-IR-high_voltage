"""Driver for the Siglent SPD3303X / SPD3303X-E bench power supply (SCPI over VISA).

The SPD3303X is not a COM port. Over USB it is a USB-TMC instrument (Device Manager:
"USB Test and Measurement Device (IVI)", VID 0xF4EC, PID 0x1430) and is reached
through VISA. pyvisa uses the NI-VISA library installed on the lab PC.

Verified on a real unit (2026-11-24, outputs unloaded): SPD3303X, serial
SPD3XJGX903210, firmware 1.01.01.03.11R1, hardware V6.2, USB + NI-VISA + pyvisa 1.16.2.
- Queries: *IDN?, SYSTem:VERSion?, SYSTem:STATus?, SYSTem:ERRor?, CHn:VOLTage?,
  CHn:CURRent?, MEASure:VOLTage?/CURRent?/POWEr? CHn. 20 back-to-back queries without
  a pause all answered (~2 ms each).
- CHn:VOLTage <v> and CHn:CURRent <i> (CH1 5 V / 0.1 A, CH2 3.3 V / 0.1 A): read back
  correctly; set + readback ~40 ms. 10 back-to-back set_voltage calls all verified.
- OUTPut CHn,ON|OFF: status word follows (0x4 all off, 0x14 CH1 on, 0x24 CH2 on);
  measured voltage reaches the setpoint within 0.2 s and is 0 V after off; switching
  one channel does not touch the other. OUTPut CH3,ON|OFF accepted (state not readable).
- Status bits: 0x4 = independent mode (bits 2-3 = 0b01), bit 4 = CH1 on, bit 5 = CH2 on.
  The CC bits (0, 1) were not exercised (no load, so always CV).
- SYSTem:ERRor? does NOT report an undefined command on this firmware ('FOO:BAR 1'
  leaves the queue at '0  No Error'). So the error-queue check below only catches what
  the unit chooses to report; the setpoint/output readback is the real verification.

Safety rules built into the driver:
- Opening and closing the connection change nothing on the supply (only *IDN? is sent).
- Every setpoint is checked against the hardware range (CH1/CH2: 0-32 V, 0-3.2 A) and
  against optional soft limits (v_max / i_max) before it is sent, then read back.
- After every command that changes the supply, the SCPI error queue is read; a
  non-zero error raises SPD3303XError (see the note above: the unit does not report
  everything).
- output(ch, True) first reads the setpoints back from the supply and refuses to turn
  the output on if they are outside the limits (someone may have changed them on the
  front panel).
- Series/parallel tracking mode is not implemented; set_* and output() refuse to run
  unless the supply is in independent mode.
- CH3 (2.5/3.3/5 V, selected by the front-panel switch) can only be switched on/off;
  its voltage cannot be set or measured over SCPI, and its on/off state is not in the
  status word.

Usage (library):
    from spd3303x import SPD3303X
    with SPD3303X(v_max=24.5, i_max=1.0) as psu:   # auto-finds the USB unit
        print(psu.idn)
        psu.set_voltage("CH1", 24.0)
        psu.set_current("CH1", 0.5)                 # current limit
        psu.output("CH1", True)
        print(psu.measure("CH1"))                   # {'v': ..., 'i': ..., 'p': ...}
        psu.output("CH1", False)

Usage (command line; read-only unless set/on/off is given):
    python spd3303x.py                          # IDN, mode, setpoints, outputs, readings
    python spd3303x.py --list                   # list VISA resources and exit
    python spd3303x.py monitor -i 1             # print CH1/CH2 V, I, P every 1 s
    python spd3303x.py monitor -i 1 --csv psu.csv
    python spd3303x.py set CH1 24 0.5           # setpoints only; output state unchanged
    python spd3303x.py on CH1
    python spd3303x.py off CH1                  # or: off all
    python spd3303x.py --v-max 24.5 --i-max 1 set CH1 24 0.5   # with soft limits
"""

import argparse
import csv
import sys
import threading
import time
from datetime import datetime

import pyvisa

SIGLENT_VID = 0xF4EC
CHANNELS = ("CH1", "CH2")          # programmable channels
OUTPUTS = ("CH1", "CH2", "CH3")    # channels that can be switched on/off
V_RANGE = (0.0, 32.0)              # CH1/CH2 hardware range, volts
I_RANGE = (0.0, 3.2)               # CH1/CH2 hardware range, amps
RESOLUTION = 0.001                 # setpoint resolution (1 mV, 1 mA)

# SYSTem:STATus? bits (Siglent programming guide; bits 0-5 used here).
_BIT_CC = {"CH1": 0, "CH2": 1}     # 0 = CV, 1 = CC
_BIT_ON = {"CH1": 4, "CH2": 5}     # 0 = off, 1 = on
_MODE_SHIFT = 2                    # bits 2-3: 0b01 = independent (verified)
_MODE_INDEPENDENT = 0b01


class SPD3303XError(RuntimeError):
    pass


def find_resources(rm=None):
    """VISA resource strings of all Siglent USB instruments (vendor ID 0xF4EC)."""
    rm = rm or pyvisa.ResourceManager()
    return [r for r in rm.list_resources() if f"0X{SIGLENT_VID:04X}" in r.upper()]


def _check_channel(ch, allowed=CHANNELS):
    ch = str(ch).upper()
    if ch not in allowed:
        raise SPD3303XError(f"channel must be one of {allowed}, got {ch!r}")
    return ch


def _per_channel(value):
    """None, a float for both channels, or a dict {'CH1': x, 'CH2': y}."""
    if value is None:
        return {}
    if isinstance(value, dict):
        out = {}
        for ch, x in value.items():
            out[_check_channel(ch)] = float(x)
        return out
    return {ch: float(value) for ch in CHANNELS}


class SPD3303X:
    def __init__(self, resource=None, timeout_ms=2000, v_max=None, i_max=None,
                 readback_timeout_s=1.0, visa_library=""):
        """
        resource: VISA resource string, e.g. 'USB0::0xF4EC::0x1430::SPD3XJGX903210::INSTR'
            or 'TCPIP0::192.168.1.50::5025::SOCKET' for LAN. None = the single Siglent
            USB instrument found (error if none or more than one).
        v_max, i_max: optional soft limits, a float for both channels or a dict per
            channel, e.g. v_max={'CH1': 24.5}. Applied on top of the hardware range.
        readback_timeout_s: how long to wait for a written setpoint/output state to
            show up in the readback before raising.
        """
        self.resource = resource
        self.timeout_ms = timeout_ms
        self.v_max = _per_channel(v_max)
        self.i_max = _per_channel(i_max)
        self.readback_timeout_s = readback_timeout_s
        self.visa_library = visa_library
        self._rm = None
        self._inst = None
        self._idn = None
        self._lock = threading.RLock()

    # ------------------------------------------------------------ connection
    def open(self):
        if self._inst is not None:
            return self
        self._rm = pyvisa.ResourceManager(self.visa_library)
        res = self.resource
        if res is None:
            found = [r for r in find_resources(self._rm) if r.upper().startswith("USB")]
            if len(found) != 1:
                raise SPD3303XError(f"expected exactly one Siglent USB instrument, found {found}")
            res = found[0]
        inst = self._rm.open_resource(res, timeout=self.timeout_ms,
                                      read_termination="\n", write_termination="\n")
        self._inst = inst
        self.resource = res
        try:
            idn = self.query("*IDN?")
        except Exception:
            self.close()
            raise
        if "SPD3303X" not in idn.upper():
            self.close()
            raise SPD3303XError(f"{res} is not an SPD3303X: {idn!r}")
        self._idn = idn
        return self

    def close(self):
        """Close the VISA session. Outputs and setpoints are left as they are."""
        if self._inst is not None:
            try:
                self._inst.close()
            finally:
                self._inst = None
        if self._rm is not None:
            try:
                self._rm.close()
            finally:
                self._rm = None

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()

    # --------------------------------------------------------------- low level
    def query(self, cmd: str) -> str:
        with self._lock:
            if self._inst is None:
                raise SPD3303XError("not connected (call open() or use 'with')")
            return self._inst.query(cmd).strip()

    def _query_float(self, cmd: str) -> float:
        r = self.query(cmd)
        try:
            return float(r)
        except ValueError:
            raise SPD3303XError(f"{cmd}: expected a number, got {r!r}") from None

    def error(self):
        """Pop one entry from the SCPI error queue: (code, message); code 0 = no error."""
        r = self.query("SYSTem:ERRor?")
        head, _, msg = r.partition(" ")
        try:
            code = int(head.rstrip(","))
        except ValueError:
            raise SPD3303XError(f"SYSTem:ERRor?: unexpected reply {r!r}") from None
        return code, msg.strip()

    def write(self, cmd: str):
        """Send a command that changes the supply, then check the error queue."""
        with self._lock:
            if self._inst is None:
                raise SPD3303XError("not connected (call open() or use 'with')")
            self._inst.write(cmd)
            code, msg = self.error()
        if code != 0:
            raise SPD3303XError(f"{cmd!r} -> error {code}: {msg}")

    # ------------------------------------------------------------ read-only API
    @property
    def idn(self) -> str:
        """'Siglent Technologies,SPD3303X,<serial>,<firmware>,<hardware>'."""
        if self._idn is None:
            self._idn = self.query("*IDN?")
        return self._idn

    @property
    def serial_number(self) -> str:
        parts = self.idn.split(",")
        return parts[2].strip() if len(parts) > 2 else ""

    def status(self) -> dict:
        """Decoded SYSTem:STATus?: raw value, mode, and per-channel on / CC flags."""
        r = self.query("SYSTem:STATus?")
        try:
            raw = int(r, 16)
        except ValueError:
            raise SPD3303XError(f"SYSTem:STATus?: unexpected reply {r!r}") from None
        mode_bits = (raw >> _MODE_SHIFT) & 0b11
        out = {
            "raw": raw,
            "mode": "independent" if mode_bits == _MODE_INDEPENDENT
                    else f"tracking (bits 0b{mode_bits:02b})",
        }
        for ch in CHANNELS:
            out[f"{ch}_on"] = bool(raw >> _BIT_ON[ch] & 1)
            out[f"{ch}_cc"] = bool(raw >> _BIT_CC[ch] & 1)
        return out

    def is_on(self, ch) -> bool:
        ch = _check_channel(ch)
        return self.status()[f"{ch}_on"]

    def voltage_setpoint(self, ch) -> float:
        return self._query_float(f"{_check_channel(ch)}:VOLTage?")

    def current_setpoint(self, ch) -> float:
        """Current limit setpoint, amps."""
        return self._query_float(f"{_check_channel(ch)}:CURRent?")

    def measure(self, ch) -> dict:
        """Measured output: {'v': volts, 'i': amps, 'p': watts}."""
        ch = _check_channel(ch)
        return {"v": self._query_float(f"MEASure:VOLTage? {ch}"),
                "i": self._query_float(f"MEASure:CURRent? {ch}"),
                "p": self._query_float(f"MEASure:POWEr? {ch}")}

    def read(self, channels=CHANNELS) -> dict:
        """One timestamped reading of all channels, for logging."""
        st = self.status()
        row = {"time": datetime.now().isoformat(timespec="milliseconds")}
        for ch in channels:
            ch = _check_channel(ch)
            m = self.measure(ch)
            row.update({f"{ch}_on": int(st[f"{ch}_on"]), f"{ch}_cc": int(st[f"{ch}_cc"]),
                        f"{ch}_v": m["v"], f"{ch}_i": m["i"], f"{ch}_p": m["p"]})
        return row

    # ---------------------------------------------------------------- limits
    def _limit(self, ch, kind):
        hw = V_RANGE[1] if kind == "v" else I_RANGE[1]
        soft = (self.v_max if kind == "v" else self.i_max).get(ch)
        return hw if soft is None else min(hw, soft)

    def _check_value(self, ch, kind, value):
        value = float(value)
        lo = 0.0
        hi = self._limit(ch, kind)
        unit = "V" if kind == "v" else "A"
        if not (lo <= value <= hi + RESOLUTION / 2):
            raise SPD3303XError(f"{ch}: {value:g} {unit} is outside {lo:g}-{hi:g} {unit}")
        return round(value, 3)

    def _require_independent(self):
        mode = self.status()["mode"]
        if mode != "independent":
            raise SPD3303XError(f"supply is in {mode} mode; only independent mode is supported")

    def _wait_for(self, read, want, what):
        t_end = time.monotonic() + self.readback_timeout_s
        while True:
            got = read()
            ok = (abs(got - want) <= RESOLUTION * 1.1) if isinstance(want, float) else got == want
            if ok:
                return got
            if time.monotonic() > t_end:
                raise SPD3303XError(f"{what}: wrote {want!r}, read back {got!r}")
            time.sleep(0.05)

    # ------------------------------------------------------- commands that act
    def set_voltage(self, ch, volts):
        """Set the voltage setpoint (output on/off state is not changed)."""
        ch = _check_channel(ch)
        v = self._check_value(ch, "v", volts)
        with self._lock:
            self._require_independent()
            self.write(f"{ch}:VOLTage {v:.3f}")
            return self._wait_for(lambda: self.voltage_setpoint(ch), v, f"{ch} voltage")

    def set_current(self, ch, amps):
        """Set the current limit (output on/off state is not changed)."""
        ch = _check_channel(ch)
        i = self._check_value(ch, "i", amps)
        with self._lock:
            self._require_independent()
            self.write(f"{ch}:CURRent {i:.3f}")
            return self._wait_for(lambda: self.current_setpoint(ch), i, f"{ch} current")

    def output(self, ch, on: bool):
        """Switch an output on or off. Turning CH1/CH2 on first re-checks its setpoints
        (as stored in the supply) against the limits."""
        ch = _check_channel(ch, OUTPUTS)
        with self._lock:
            if on and ch in CHANNELS:
                self._require_independent()
                self._check_value(ch, "v", self.voltage_setpoint(ch))
                self._check_value(ch, "i", self.current_setpoint(ch))
            self.write(f"OUTPut {ch},{'ON' if on else 'OFF'}")
            if ch in CHANNELS:  # CH3 is not reported in the status word
                self._wait_for(lambda: self.is_on(ch), bool(on), f"{ch} output")

    def all_off(self):
        """Turn CH1, CH2 and CH3 off. Tries every channel even if one fails."""
        errors = []
        for ch in OUTPUTS:
            try:
                self.output(ch, False)
            except Exception as e:  # noqa: BLE001 - keep going, report at the end
                errors.append(f"{ch}: {e}")
        if errors:
            raise SPD3303XError("all_off: " + "; ".join(errors))


# ------------------------------------------------------------------------ CLI
def _print_summary(psu):
    st = psu.status()
    print(f"# {psu.idn}")
    print(f"# resource {psu.resource}   mode {st['mode']}   status 0x{st['raw']:X}")
    for ch in CHANNELS:
        m = psu.measure(ch)
        print(f"{ch}: set {psu.voltage_setpoint(ch):.3f} V / {psu.current_setpoint(ch):.3f} A   "
              f"output {'ON ' if st[f'{ch}_on'] else 'OFF'}  {'CC' if st[f'{ch}_cc'] else 'CV'}   "
              f"meas {m['v']:.3f} V  {m['i']:.3f} A  {m['p']:.3f} W")
    print("CH3: output state is not reported over SCPI")


def _monitor(psu, interval, csv_path, channels):
    fields = ["time"] + [f"{ch}_{k}" for ch in channels for k in ("on", "cc", "v", "i", "p")]
    fh = writer = None
    if csv_path:
        fh = open(csv_path, "a", newline="")
        writer = csv.DictWriter(fh, fieldnames=fields)
        if fh.tell() == 0:
            writer.writeheader()
    try:
        while True:
            t0 = time.monotonic()
            r = psu.read(channels)
            print(r["time"] + "  " + "   ".join(
                f"{ch} {'ON ' if r[f'{ch}_on'] else 'OFF'} {r[f'{ch}_v']:7.3f} V "
                f"{r[f'{ch}_i']:6.3f} A {r[f'{ch}_p']:7.3f} W{' CC' if r[f'{ch}_cc'] else ''}"
                for ch in channels))
            if writer:
                writer.writerow(r)
                fh.flush()
            time.sleep(max(0.0, interval - (time.monotonic() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        if fh:
            fh.close()


def main(argv=None):
    ap = argparse.ArgumentParser(description="Siglent SPD3303X power supply (pyvisa)")
    ap.add_argument("-r", "--resource", help="VISA resource (default: the single Siglent USB unit)")
    ap.add_argument("--list", action="store_true", help="list VISA resources and exit")
    ap.add_argument("--v-max", type=float, help="soft voltage limit for CH1/CH2, volts")
    ap.add_argument("--i-max", type=float, help="soft current limit for CH1/CH2, amps")
    sub = ap.add_subparsers(dest="cmd")
    p = sub.add_parser("monitor", help="log V/I/P periodically (read-only)")
    p.add_argument("-i", "--interval", type=float, default=1.0)
    p.add_argument("--csv", help="append readings to this CSV file")
    p.add_argument("--ch", nargs="+", default=list(CHANNELS), choices=CHANNELS)
    p = sub.add_parser("set", help="set voltage and current limit (output unchanged)")
    p.add_argument("ch", choices=CHANNELS)
    p.add_argument("volts", type=float)
    p.add_argument("amps", type=float)
    p = sub.add_parser("on", help="turn an output on")
    p.add_argument("ch", choices=OUTPUTS)
    p = sub.add_parser("off", help="turn an output off ('all' = CH1, CH2 and CH3)")
    p.add_argument("ch", choices=OUTPUTS + ("all",))
    args = ap.parse_args(argv)

    if args.list:
        rm = pyvisa.ResourceManager()
        for r in rm.list_resources():
            print(r, "  <- Siglent" if f"0X{SIGLENT_VID:04X}" in r.upper() else "")
        return 0

    with SPD3303X(args.resource, v_max=args.v_max, i_max=args.i_max) as psu:
        if args.cmd == "monitor":
            _monitor(psu, args.interval, args.csv, args.ch)
            return 0
        if args.cmd == "set":
            v = psu.set_voltage(args.ch, args.volts)
            i = psu.set_current(args.ch, args.amps)
            print(f"{args.ch}: set {v:.3f} V / {i:.3f} A (output unchanged)")
        elif args.cmd == "on":
            psu.output(args.ch, True)
            print(f"{args.ch}: output ON")
        elif args.cmd == "off":
            if args.ch == "all":
                psu.all_off()
            else:
                psu.output(args.ch, False)
            print(f"{args.ch}: output OFF")
        _print_summary(psu)
    return 0


if __name__ == "__main__":
    sys.exit(main())
