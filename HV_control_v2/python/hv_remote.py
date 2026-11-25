"""
hv_remote.py - send commands to a running hv_bringup_gui.py through its local remote control
(127.0.0.1:50734; enable with "hv_bringup_gui.py --remote" or the checkbox in the 24 V strip).

The GUI forwards whitelisted commands over its own HVLink, so its session HV cap, the supply
watchdog permit and its logging all apply.  Every remote command shows up in the GUI log.

    python hv_remote.py --status          # board status, monitor volts, D, watchdog state
    python hv_remote.py "d 394"           # one firmware command, prints the reply
    python hv_remote.py "e 1" "d 394"     # several, in order (stops at the first error)
"""
import argparse
import json
import socket
import sys

ADDR = ("127.0.0.1", 50734)


def send(cmd, timeout=5.0):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(timeout)
    try:
        s.sendto(json.dumps({"cmd": cmd}).encode("utf-8"), ADDR)
        data, _ = s.recvfrom(65536)
        return json.loads(data.decode("utf-8"))
    except OSError as e:
        return {"ok": False, "error": "GUI remote control not answering (%s)" % e}
    finally:
        s.close()


def main(argv=None):
    ap = argparse.ArgumentParser(description="Remote commands for hv_bringup_gui.py")
    ap.add_argument("--status", action="store_true")
    ap.add_argument("cmds", nargs="*")
    args = ap.parse_args(argv)
    if args.status:
        r = send("#status")
        print(json.dumps(r, indent=1))
        return 0 if r.get("ok") else 1
    for c in args.cmds:
        r = send(c)
        print("%s -> %s" % (c, r.get("reply") if r.get("ok") else "ERROR: %s" % r.get("error")))
        if not r.get("ok") or str(r.get("reply", "")).startswith("ERR"):
            return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
