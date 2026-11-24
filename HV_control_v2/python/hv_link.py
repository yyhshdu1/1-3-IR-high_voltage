"""
hv_link.py - serial protocol layer for HV_control firmware v2 (FW 2.x).

Thread-safe, line based, one reply line per command (requires "c ack 1", which
connect() switches on).  Optional XOR checksum "<text>*HH" is appended to every
command by default.

NOTE: opening the serial port resets the Arduino (DTR).  After a reset the firmware
boots safe: HV off, DAC 0, relay released.

Designed to be reused later by the full control program.
"""
import json
import os
import re
import threading
import time

import serial
from serial.tools import list_ports

# megaAVR RSTCTRL.RSTFR bits (as printed by "s" -> rst=0x..)
RST_BITS = {
    0x01: "POR(上电)",
    0x02: "BOR(掉电)",
    0x04: "EXT(外部复位)",
    0x08: "WDT(看门狗)",
    0x10: "SW(软件复位)",
    0x20: "UPDI(烧录/打开串口DTR复位)",
}

# firmware fault flags ("flt=0x..")
FLT_BITS = {
    0x01: "VPROG过压",
    0x02: "DAC回读不符",
    0x04: "主机超时",
    0x08: "放电过多",
    0x10: "ADC卡死",
    0x20: "X射线告警(已降压)",
    0x40: "X射线监测丢失",
    0x80: "J1监测线缆未接(已降压)",
}

CH_NAMES = ["v_mon", "i_mon", "v_prog", "v_div", "v_err"]


def decode_bits(value, table):
    names = [name for bit, name in table.items() if value & bit]
    return ", ".join(names) if names else "无"


def checksum(text):
    x = 0
    for c in text.encode("ascii"):
        x ^= c
    return x


def with_checksum(text):
    return "%s*%02X" % (text, checksum(text))


class HVError(Exception):
    """I/O problem or unexpected reply."""


class HVTimeout(HVError):
    pass


class HVCommandError(HVError):
    """Firmware answered ERR ..."""

    def __init__(self, cmd, reply):
        HVError.__init__(self, "%s -> %s" % (cmd, reply))
        self.cmd = cmd
        self.reply = reply


_kv_re = re.compile(r"(\w+)=(\S+)")


def _conv(v):
    try:
        return int(v, 0)
    except ValueError:
        pass
    try:
        return float(v)
    except ValueError:
        return v


def parse_kv(line):
    return dict((k, _conv(v)) for k, v in _kv_re.findall(line))


class HVLink(object):
    def __init__(self, log=None):
        self.ser = None
        self.port = None
        self.lock = threading.RLock()
        self.log = log or (lambda s: None)
        self.use_checksum = True
        self.log_io = True          # log TX/RX of non-quiet commands
        self.generation = 0         # increments on every (re)open = Arduino reset

    # ------------------------------------------------------------------ ports
    @staticmethod
    def list_ports():
        return [(p.device, p.description) for p in list_ports.comports()]

    @property
    def is_open(self):
        return self.ser is not None and self.ser.is_open

    # ------------------------------------------------------------ open / close
    def open(self, port, boot_wait=2.0):
        """Open the port (this resets the Arduino), verify the firmware, enable ack."""
        with self.lock:
            self.close()
            self.ser = serial.Serial(port, 9600, timeout=0.3, write_timeout=1.0)
            self.port = port
            self.generation += 1
            time.sleep(boot_wait)
            self.ser.reset_input_buffer()
            # ack on: the firmware answers OK / ERR to every write command
            try:
                self.query("c ack 1", quiet=True)
                banner = self.query("?", quiet=True)
            except HVError:
                self.close()
                raise
            if not banner.startswith("HV_control"):
                self.close()
                raise HVError("unexpected banner: %r" % banner)
            self.log("已连接 %s: %s" % (port, banner))
            return banner

    def reopen(self, boot_wait=2.0):
        """Close and open again = hardware reset of the Arduino via DTR."""
        port = self.port
        if port is None:
            raise HVError("no port")
        return self.open(port, boot_wait)

    def close(self):
        with self.lock:
            if self.ser is not None:
                try:
                    self.ser.close()
                except Exception:
                    pass
            self.ser = None

    # --------------------------------------------------------------- commands
    def query(self, cmd, timeout=1.5, quiet=False, raw=False):
        """Send one command line, return the single reply line (stripped).
        An "ERR ..." reply is returned as a string; use cmd_ok() to raise on it."""
        with self.lock:
            if not self.is_open:
                raise HVError("serial port not open")
            line = cmd if (raw or not self.use_checksum) else with_checksum(cmd)
            try:
                self.ser.reset_input_buffer()
                self.ser.timeout = timeout
                self.ser.write((line + "\n").encode("ascii"))
                data = self.ser.readline()
            except (serial.SerialException, OSError) as e:
                self.close()
                raise HVError("serial error: %s" % e)
            reply = data.decode("ascii", "replace").strip()
            if self.log_io and not quiet:
                self.log("> %s   < %s" % (line, reply if reply else "(无回复)"))
            if not data.endswith(b"\n"):
                raise HVTimeout("no reply to %r (got %r)" % (cmd, reply))
            return reply

    def cmd_ok(self, cmd, quiet=False):
        """Write command that must be answered with OK."""
        reply = self.query(cmd, quiet=quiet)
        if reply != "OK":
            raise HVCommandError(cmd, reply)
        return reply

    # ---------------------------------------------------------------- helpers
    def status(self, quiet=True):
        line = self.query("s", quiet=quiet)
        if not line.startswith("S "):
            raise HVError("bad status reply: %r" % line)
        return parse_kv(line)

    def config(self, quiet=True):
        line = self.query("c", quiet=quiet)
        if not line.startswith("CFG "):
            raise HVError("bad config reply: %r" % line)
        return parse_kv(line)

    def volts(self, quiet=True):
        line = self.query("b", quiet=quiet)
        try:
            vals = [float(x) for x in line.split(",")]
        except ValueError:
            raise HVError("bad volts reply: %r" % line)
        if len(vals) != 5:
            raise HVError("bad volts reply: %r" % line)
        return vals

    # ------------------------------------------------ zero offsets (v_mon / i_mon)
    # With the TOF3000 monitor inputs shorted to ground the board still reads about +17 mV on CH0 / CH1
    # (ADC / buffer offset).  Measure it once per board, store it by board name, and subtract it in Python.
    # Firmware 'a' / 'b' stay raw on purpose (they are what the calibration stages compare).
    def measure_offsets(self, n=40, delay=0.05):
        """Mean of CH0 / CH1 in volts.  ONLY with the J1 monitor inputs grounded (or the TOF3000 at 0 V, HV off)."""
        s0 = s1 = 0.0
        for _ in range(n):
            v = self.volts()
            s0 += v[0]
            s1 += v[1]
            time.sleep(delay)
        return {"v_mon": s0 / n, "i_mon": s1 / n}

    def volts_corrected(self, offsets, quiet=True):
        v = self.volts(quiet=quiet)
        v[0] -= offsets.get("v_mon", 0.0)
        v[1] -= offsets.get("i_mon", 0.0)
        return v

    def counts(self, quiet=True):
        line = self.query("a", quiet=quiet)
        try:
            return [int(x) for x in line.split(",")]
        except ValueError:
            raise HVError("bad counts reply: %r" % line)

    def dstate(self, quiet=True):
        """(target, current, raw DAC code) in host units / counts."""
        line = self.query("D", quiet=quiet)
        try:
            t, c, r = [int(float(x)) for x in line.split(",")]
        except ValueError:
            raise HVError("bad D reply: %r" % line)
        return t, c, r

    def window(self, quiet=True):
        """'m' command: statistics since the last call (resets the firmware window)."""
        line = self.query("m", quiet=quiet)
        if not line.startswith("M "):
            raise HVError("bad m reply: %r" % line)
        return parse_kv(line)

    def emergency_stop(self):
        """'z': DAC 0, relay released, HV off.  Retries once."""
        for _ in range(2):
            try:
                return self.query("z", timeout=1.0)
            except HVError:
                time.sleep(0.2)
        raise HVError("emergency stop got no reply")


# ---------------------------------------------------------------------------------------------
# per-board zero offsets (stored next to this file, keyed by the board's 'name' from "c name")
# ---------------------------------------------------------------------------------------------
OFFSET_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "offsets")


def offset_path(name):
    return os.path.join(OFFSET_DIR, "offsets_%s.json" % name.replace("+", "plus").replace("-", "minus"))


def load_offsets(name):
    try:
        with open(offset_path(name), encoding="utf8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return {"v_mon": 0.0, "i_mon": 0.0}


def save_offsets(name, offs):
    os.makedirs(OFFSET_DIR, exist_ok=True)
    with open(offset_path(name), "w", encoding="utf8") as f:
        json.dump(offs, f, indent=1)


if __name__ == "__main__":
    # python hv_link.py zero COM4      (J1 monitor inputs grounded, HV off)
    import sys
    if len(sys.argv) == 3 and sys.argv[1] == "zero":
        lk = HVLink(log=lambda m: None)
        lk.open(sys.argv[2])
        banner = lk.query("?")
        name = banner.split("Name ")[1].split(".")[0].strip() if "Name " in banner else "-"
        if name == "-":
            print("board has no name: set it first (c name TOF+ / TOF-, then c save)")
        else:
            st = lk.status()
            if st["hv"] or st["cur"]:
                print("HV is on or DAC is not zero: refusing")
            else:
                offs = lk.measure_offsets()
                print("board %s: v_mon offset %.4f V, i_mon offset %.4f V" % (name, offs["v_mon"], offs["i_mon"]))
                if max(abs(offs["v_mon"]), abs(offs["i_mon"])) > 0.1:
                    print("offset larger than 0.1 V: the monitor inputs are probably not grounded; NOT saved")
                else:
                    save_offsets(name, offs)
                    print("saved", offset_path(name))
        lk.close()
    else:
        print("usage: python hv_link.py zero COMn")
