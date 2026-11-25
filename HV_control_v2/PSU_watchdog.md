# 24 V supply outer interlock (SPD3303X + psu_watchdog.py)

Written 2026-11-24. Applies to FW 2.8 and the Python files in `python/`.

## 1. What it is

The Siglent SPD3303X CH1 supplies 24 V to the HV_control PCB and the TOF3000. In normal operation CH1 stays **on**. `psu_watchdog.py` is the outermost interlock: when the PC loses control of the HV_control Arduino, it switches CH1 **off**. That removes power from the TOF3000 whatever the Arduino or the HV_enable line is doing.

It sits on top of the existing layers (see `配置与测试计划.md` section 5) and replaces none of them:

| Layer | Acts when | Action |
|---|---|---|
| Firmware interlocks (spark, X-ray, v_prog, ADC, cable) | Nano powered, fault seen | DAC to 0 / back-off, HV_enable off |
| Firmware heartbeat `c timeout` | Nano powered, PC silent | ramp down at `down` rate, latch 0x04 |
| Isolator default level on HV_enable | Nano / USB unpowered | HV_enable high = off (TOF3000 behaviour at ~5 V **not yet verified**, stage 7 1b) |
| **psu_watchdog.py (this)** | PC running, control of the Arduino lost | **CH1 24 V off**, latched |

## 2. Pieces

| File | Role |
|---|---|
| `python/spd3303x.py` | pyvisa driver for the SPD3303X (USB-TMC, not a COM port). Also a CLI. |
| `python/psu_watchdog.py` | The watchdog process (`PsuWatchdog`), plus `WatchdogClient` used by the program that owns the Arduino port. |
| `python/hv_link.py` | Three additions: `last_ok` (time of the last reply), `last_status` (last `s` reply), and an optional `permit` hook that refuses `e 1` / `d N>0` with `ERR PSU_WD`. |
| `python/hv_bringup_gui.py` | 24 V strip under the status strip: watchdog state, CH1 V/I, buttons, and the "HV only while armed" checkbox. |
| `python/test_psu_watchdog.py` | Offline test: simulated supply and Arduino, real UDP. 53 checks. |

### Why a separate process

The Arduino port can be open in only one program, and opening it resets the Arduino (DTR). So the watchdog cannot check the Arduino directly. The program that owns the port (the GUI now, the control program later) runs a `WatchdogClient` thread. Every 0.5 s it sends a heartbeat carrying *proof of control*:
- how long ago the Arduino last answered (`ok_age`)
- whether the port is open
- whether HV is confirmed off (fresh `s` with hv=0, tgt=cur=raw=0)

The watchdog owns the SPD3303X and cuts CH1 when that proof stops. Because it runs in its own process, it also catches the control program crashing or hanging.

Messages are JSON over UDP on `127.0.0.1:50733`, and every request gets the full state back. The port is bound exclusively, so a second watchdog refuses to start.

## 3. States and trip conditions

States: `IDLE` (no client armed), `ARMED`, `TRIPPED` (latched).

While at least one client is armed, any of the following trips the watchdog:

| Condition | Default threshold |
|---|---|
| No heartbeat from an armed client (control program crashed, hung or killed) | 3 s (`--hb-timeout`) |
| The client reports the Arduino has not answered | 5 s (`--ok-age-max`) |
| The client reports its serial port closed (USB link lost) | immediate |
| The client sees an unexpected Arduino reset: `up` went back, or `rst` / reset counters changed, within one connection. A reopen by the program itself (new generation) is not counted. | immediate |
| Disarm requested while HV is not confirmed off | immediate |
| Lost contact with the SPD3303X | immediate. CH1 is switched off as soon as contact returns. |
| CH1 found OFF while it should be on (front panel, supply fault) | immediate (already off) |
| Ctrl-C in the watchdog window, or the window closed / logoff / shutdown | immediate |

A trip switches CH1 off and confirms it in the status word. CH1 then stays off until **Reset interlock** and then **24 V ON**:
- **Reset** needs every armed client fresh, its Arduino answering, and HV off.
- **24 V ON** additionally checks that the CH1 setpoint is 24 V ± 0.5 V (`--v-nom`, `--v-tol`) and, if `--i-max` is given, that the current limit is not above it.

The client side:
- **Sends `z` to its Arduino once** when the watchdog trips, reports the SPD3303X lost, or stops answering (2 s). HV then cannot come back by itself when 24 V returns.
- **Refuses `e 1` and `d N>0` with `ERR PSU_WD`** unless the watchdog is ARMED, answering and connected to the SPD3303X. This applies when the GUI checkbox "HV (e 1 / d>0) only while armed" is ticked, which is the default. Untick it only for bench work without the TOF3000.

Never changing CH1: starting the watchdog, arming, a clean disarm (HV off), stopping the watchdog while IDLE.

## 4. How to run

The GUI starts the watchdog by itself on **Connect** if it is not running (own console window), then arms. Manually:

```
python psu_watchdog.py                 # CH1, 24 V +- 0.5 V
python psu_watchdog.py --i-max 1.0     # also check the current limit before 24 V ON
python psu_watchdog.py --status        # state of a running watchdog
python psu_watchdog.py --forget NAME   # drop a stale client record
```

GUI sequence: **Connect** → the watchdog is ARMED → **24 V ON** (asks for confirmation) → normal work. **Disconnect** or closing the GUI disarms cleanly: 24 V stays on and the watchdog keeps running. If HV is not off at that moment, the watchdog trips instead.

Python: `spd3303x.py` and `psu_watchdog.py` need `pyvisa` and the NI-VISA runtime. On the lab PC (2026-11-24) only `C:\Users\CeNTREX\AppData\Local\Programs\Python\Python311\python.exe` has both pyvisa and pyserial. Anaconda base has neither, and `运行GUI.bat` picks Anaconda first. The GUI starts the watchdog with its own interpreter.

Do not use `spd3303x.py set/on/off` while the watchdog is running. If CH1 is switched off behind its back, it trips.

## 5. What it does not cover

- **The PC freezes or loses power.** Nothing on the PC runs. The firmware heartbeat and the HV_enable default level remain.
- **The watchdog process is killed** (Task Manager, crash). The clients see it stop answering and send `z`, but CH1 stays as it was.
- **Firmware faults.** The firmware handles its own interlocks.
- **Cutting 24 V at HV is a hard stop, not a ramp.** How the TOF3000 discharges when it loses 24 V (internal bleeder or not) is not known yet: check the manual (stage 7 list).
- **Power-up order.** 24 V-first vs USB-first behaviour of DAC / v_prog (plan B.4) is still unmeasured. `24 V ON` therefore requires the Arduino connected and HV off.

## 6. Test status (2026-11-24)

| Test | Result |
|---|---|
| `spd3303x.py` offline (simulated supply) | 32/32 |
| `spd3303x.py` on the real SPD3303X, outputs unloaded, ≤ 5 V | 24/25. The only failure: the unit does not report an undefined command in `SYSTem:ERRor?`, so readback is the real check. |
| `test_psu_watchdog.py` (simulated supply + Arduino) | 54/54 |
| Watchdog process with the real SPD3303X, simulated Arduino, CH1 kept OFF | 13/13. Client stopped → trip after 2.6 s, CH1 OFF confirmed 45 ms later. Watchdog killed → client sent `z`. Second instance refused. |
| GUI starts with no watchdog running; permit refuses `e 1` | OK (smoke test, window hidden) |
| **Real system, no TOF3000** (below) | **31/31, run twice** |
| Closing the watchdog console window while armed (CTRL_CLOSE handler) | **not done** |
| Unplugging the Arduino USB / the SPD3303X USB, Nano reset button, CH1 off on the front panel | **not done** (need hands) |
| TOF3000 behaviour when 24 V is cut at HV | **not done** |

### Real-system test, 2026-11-24

Setup:
- SPD3303X CH1 (24.000 V, limit 1.500 A) to the HV_control PCB 24 V input.
- Board TOF+ (Board B, FW 2.8, Pol +) on COM4.
- **J1 open, no TOF3000.** `e 1` + `d 3000` therefore only puts v_prog ≈ 0.53 V on the open J1-2.

Script: scratchpad `hw_test_system.py`, real `psu_watchdog.py` process. The sequence was run twice and passed 31/31 both times.

Measured:
- **PCB load at 24 V**: 0.075 A, 1.8 W steady. No CC (current limit) seen. The supply is polled at 4 Hz, so the inrush peak is not resolved.
- **24 V on, USB already connected**:
  - DAC stays 0,0,0 and hv=0.
  - The firmware latches `flt=0x13` (v_prog over-voltage, DAC read-back mismatch, ADC stuck) from the unpowered period. `C` clears it to 0.
  - Stage 3 boot check: all 9 checks pass.
- **ADC with J1 open**: v_mon and i_mon read full scale (65535, 12.288 V), because the inputs float. v_prog = 0.019 V, v_div = 0.004 V, v_err = -2.991 V.
- **24 V off with USB connected**: the ADC-stuck fault appears within 0.6 s, HV stays off and DAC stays 0. With faults latched, `e 1` is refused by the firmware (`ERR FAULT`).

Loss-of-control cases, each with HV enabled and DAC at 3000 unless noted:

| Case | Result |
|---|---|
| Control program "crashes": client and all link traffic stop | TRIPPED after 3.0 s, CH1 off. The firmware lost 24 V and shut HV off by itself (`flt=0x11`) before any `z`. Restart → reset → 24 V ON → hv=0, DAC 0, `C` clears. |
| Disarm while HV on | TRIPPED, CH1 off, `z` sent |
| Watchdog process killed | Client sent `z` once after 2.5 s: HV off, DAC 0. `e 1` refused (`ERR PSU_WD`). CH1 stays ON, the documented gap. |
| Real separate control process killed (HV off) | TRIPPED after 3.1 s, CH1 off. After reopening: hv=0, DAC 0. |
| Firmware heartbeat on the real board (`timeout=10`, no client, watchdog IDLE) | After 13 s of silence: `flt=0x04`, HV off, DAC 0. 24 V untouched. First test of the heartbeat on the real board; earlier only on the bench. |
| Permit before arming | `e 1` → `ERR PSU_WD`, E stays 0 |

Fixed after the first run:
- **`z` was sent on every heartbeat while the watchdog was gone.** The event text contained the elapsed time, so it never matched the previous one. In the GUI this would have opened a warning popup every 0.5 s. The text is now fixed and `z` is sent once.
- **arm / reset / power_on could carry `safe=false`.** If the heartbeat thread held the link lock at that moment, the request went out without a fresh status. These requests now wait up to 3 s for a fresh status.

### Stage 7 run with the TOF3000, 2026-11-24 16:55 (GUI tab "Stage 7 tests", `stage7_tests.py`)

Setup:
- TOF3000 connected. Its polarity follows the board: Pol +, so positive.
- Output: a corona nut in the insulated box.
- DB15 adapter board: not recorded.
- Limits:
  - test maximum 0.8 kV
  - hard cap 0.9 kV in two places: the GUI permit and firmware `dacmax 1771` (RAM only)
  - abort if v_mon goes above 0.95 kV
- Highest v_mon seen: **0.782 kV**.
- Result: 22 passed, 1 failed (the GMC-800 port). Report: `python/logs/stage7_report_20261124_165712.md`.
- Before the real run, the same sequence ran in simulation (`test_stage7_sim.py`): normal case passed. A TOF3000 that ignores HV_enable aborted at 1b. A v_mon reading 30 % high aborted at the first step (0.2 kV).

| Test | Result |
|---|---|
| 1b: `e 0`, `d 1000` (0.51 kV programmed, v_prog 0.185 V) | **PASS**: peak v_mon 0.5 mV (≈ 1 V). HV_enable high shuts the TOF3000 off. |
| GUI cap: `d 1772` | refused, nothing sent |
| Firmware cap: `d 1821` | `ERR RANGE` |
| Staircase 0.2 / 0.4 / 0.6 / 0.8 kV | v_mon 0.177 / 0.378 / 0.578 / 0.780 kV (−11.7 / −5.6 / −3.7 / −2.5 %), i_mon ≈ 0, no events. The roughly constant −20 V offset fits the v_prog read-back offset of ~+15 mV. |
| 24 V supply current | 0.32 A (PCB + TOF3000, HV off), 0.37 A at 0.8 kV |
| Ramp down | v_mon < 0.02 V 1.5 s after DAC 0 |
| X-ray, real GMC-800 on COM5 | first run **FAIL**: port does not open (Windows error 31, device not functioning, at every baud rate). Worked after a second USB replug. Rerun 17:15 (`stage7_report_20261124_171700.md`, **23/23 passed**, max v_mon 0.782 kV): 14 readings forwarded, firmware `xr=12` (0.078 µSv/h background). All other rows matched the first run. |
| X-ray alarm (simulated 1.00 µSv/h, `xthr 40`, `xback 400`) | back-off 400 units every 3 s (0.575 → 0.204 kV), raising refused with `ERR XRAY`; at 0.10 µSv/h the hold is released |
| X-ray monitor lost (`xtimeout 5`) | `flt 0x40` after 4.6 s, ramp down, HV off. 24 V untouched. |
| X-ray `xback 0` | `flt 0x20` latched, HV off |
| Watchdog killed at 0.6 kV | client sent `z` after 2.3 s; v_mon < 0.02 V 1.5 s later; `e 1` refused; watchdog restarted and re-armed |
| Trip at 0.4 kV (24 V cut) | CH1 off at once. The firmware saw the 24 V loss (`flt 0x11`) and turned HV off. After reset + 24 V ON: v_mon 0.2 mV, no fault. |

Independent check of the output, 2026-11-24 17:27:
- Setup: `d 394` (0.200 kV target), driven through the GUI remote control (`hv_remote.py`). Corona nut behind a 1200 MΩ series resistor; 10 MΩ multimeter across the nut side, so the divider ratio is 10 / 1210 ≈ 1/121.
- Multimeter reading: **1.7 V**, so the output is ≈ 206 V. Resistor and meter-input tolerances are not known, so the true value lies somewhere around 190-210 V.
- v_mon reading: 0.176 kV, i.e. v_mon reads ~20-30 V low. This matches the −20 V offset seen in the staircase.
- Conclusion: the DAC programming (v_prog) is close to the true output. v_mon underreads by a roughly constant ~25 V at low kV.
- Effect on the limits: the 0.95 kV v_mon abort level therefore corresponds to ~0.98 kV true. The DAC caps (0.9 kV in the GUI and in firmware `dacmax`) remain the primary limit.
- Load seen on i_mon: +0.1 µA, consistent with 206 V across 1210 MΩ (0.17 µA).

Before relying on it at higher HV:
1. Check each remaining loss of control once, with HV off, DAC 0, J1 as in stage 7:
   - unplug the Arduino USB
   - press the Nano reset button
   - close the watchdog window
   - switch CH1 off on the front panel
   - unplug the SPD3303X USB

   Each must turn CH1 off (or find it off) and latch TRIPPED.
2. With the TOF3000 connected (stage 7), repeat once at low HV (e.g. `d 1000` ≈ 0.5 kV) and watch v_mon fall.

Open settings:
- **CH1 current limit**: now 1.5 A. The PCB alone draws 0.075 A; the TOF3000 24 V current is still to be taken from its manual.
- **`--i-max`**
- **Timeouts**: whether 3 s / 5 s are right.
