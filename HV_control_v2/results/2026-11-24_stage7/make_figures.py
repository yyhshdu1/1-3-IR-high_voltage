"""
make_figures.py - figures for Stage7_summary.md (2026-11-24 bring-up: SPD3303X outer interlock,
stage 7 with the TOF3000 below 1 kV).  The numbers are copied from the logs and reports named
next to each data block.

    python make_figures.py      # writes fig1..fig4 *.png next to this file
"""
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyBboxPatch  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
NL = "\n"

# reference palette (dataviz skill, light mode): categorical slots 1-3 + status critical
S1, S2, S3 = "#2a78d6", "#eb6834", "#1baf7a"
CRIT = "#d03b3b"
INK, INK2, MUTED, GRID, SURF = "#0b0b0b", "#52514e", "#8a8984", "#e4e3df", "#fcfcfb"

plt.rcParams.update({
    "font.family": "DejaVu Sans", "font.size": 10, "axes.edgecolor": MUTED, "axes.labelcolor": INK2,
    "xtick.color": INK2, "ytick.color": INK2, "axes.titlesize": 12, "axes.titleweight": "bold",
    "axes.titlecolor": INK, "axes.titlelocation": "left", "figure.facecolor": SURF, "axes.facecolor": SURF,
    "axes.spines.top": False, "axes.spines.right": False, "axes.grid": True, "grid.color": GRID,
    "grid.linewidth": 0.8, "legend.frameon": False, "savefig.dpi": 160, "savefig.facecolor": SURF,
})


def save(fig, name):
    path = os.path.join(HERE, name)
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)
    print("wrote", path)


# ---------------------------------------------------------------- fig 1: output vs target
# python/logs/stage7_report_20261124_165712.md and _171700.md (same v_mon within 1 V in both runs);
# the 1b row of the same reports; multimeter check in PSU_watchdog.md (1.7 V on 10 MOhm behind 1200 MOhm)
target = [0.2, 0.4, 0.6, 0.8]
vmon_run1 = [0.177, 0.378, 0.578, 0.780]
vmon_run2 = [0.177, 0.378, 0.578, 0.779]
mm_kv, mm_lo, mm_hi = 1.7 * 1210 / 10 / 1000, 0.19, 0.21

fig, ax = plt.subplots(figsize=(6.4, 4.6))
ax.plot([0, 0.9], [0, 0.9], color=MUTED, lw=1, ls=(0, (4, 3)), zorder=1)
ax.text(0.44, 0.475, "output = target", color=MUTED, ha="left", va="bottom", fontsize=9, rotation=33)
vm = [(a + b) / 2 for a, b in zip(vmon_run1, vmon_run2)]
ax.plot(target, vm, color=S1, lw=2, zorder=3)
ax.scatter(target, vm, s=64, color=S1, edgecolor=SURF, linewidth=2, zorder=4, label="TOF3000 v_mon (2 runs)")
ax.errorbar([0.2], [mm_kv], yerr=[[mm_kv - mm_lo], [mm_hi - mm_kv]], fmt="D", ms=8, color=S2, mec=SURF,
            mew=2, ecolor=S2, elinewidth=2, capsize=4, zorder=5, label="multimeter via 1210 MOhm divider")
ax.scatter([0.51], [0.0005], s=80, marker="s", color=S3, edgecolor=SURF, linewidth=2, zorder=5,
           label="1b: 0.51 kV programmed, HV_enable OFF")
ax.annotate("0.780 kV", (0.8, vm[-1]), xytext=(-8, 10), textcoords="offset points", ha="right", color=INK2,
            fontsize=9)
ax.annotate("~206 V", (0.2, mm_kv), xytext=(-12, 4), textcoords="offset points", ha="right", color=INK2, fontsize=9)
ax.annotate("1 V: output stays off", (0.51, 0.0005), xytext=(10, 6), textcoords="offset points", color=INK2,
            fontsize=9)
ax.axhline(0.9, color=CRIT, lw=1)
ax.text(0.02, 0.91, "hard cap 0.9 kV (GUI + firmware dacmax)", color=INK2, fontsize=9, va="bottom")
ax.set_xlim(0, 0.95)
ax.set_ylim(-0.03, 1.0)
ax.set_xlabel("DAC target (kV)")
ax.set_ylabel("measured output (kV)")
ax.set_title("HV output follows the target; HV_enable shuts it off")
ax.legend(loc="lower right", bbox_to_anchor=(1.0, 0.08), fontsize=9)
fig.text(0.0, -0.04, "TOF3000 low-voltage accuracy is not specified; this run only checks that output is present "
         "and switchable.", color=MUTED, fontsize=8.5)
save(fig, "fig1_output_vs_target.png")

# ---------------------------------------------------------------- fig 2: response times
# real-hardware runs of 2026-11-24 (watchdog + real SPD3303X; PCB without TOF3000 x2; stage 7 reports)
rows = [
    ("SPD3303X CH1 off confirmed after a trip", 0.045, "watchdog"),
    ("firmware sees 24 V lost (ADC fault 0x10)", 0.6, "firmware"),
    ("TOF3000 output < 60 V after DAC 0", 1.5, "TOF3000"),
    ("z sent after the watchdog process dies", 2.3, "watchdog"),
    ("CH1 cut after the control program stops", 3.0, "watchdog"),
    ("X-ray monitor lost -> 0x40 (xtimeout 5 s)", 4.6, "firmware"),
    ("host heartbeat lost -> 0x04 (timeout 10 s)", 10.0, "firmware"),
]
color = {"watchdog": S1, "firmware": S2, "TOF3000": S3}
fig, ax = plt.subplots(figsize=(7.4, 3.9))
y = list(range(len(rows)))[::-1]
for yi, (name, t, layer) in zip(y, rows):
    ax.barh(yi, t, height=0.55, color=color[layer], edgecolor=SURF, linewidth=2, zorder=3)
    ax.text(t * 1.08, yi, ("%.0f ms" % (t * 1000)) if t < 1 else ("%.1f s" % t), va="center", color=INK2,
            fontsize=9)
ax.set_yticks(y)
ax.set_yticklabels([r[0] for r in rows])
ax.set_xscale("log")
ax.set_xlim(0.02, 30)
ax.set_xticks([0.03, 0.1, 0.3, 1, 3, 10, 30])
ax.set_xticklabels(["30 ms", "0.1 s", "0.3 s", "1 s", "3 s", "10 s", "30 s"])
ax.grid(axis="y", visible=False)
ax.set_xlabel("measured time (log scale)")
ax.set_title("How fast each protection layer reacted")
handles = [plt.Rectangle((0, 0), 1, 1, color=color[k]) for k in ("watchdog", "firmware", "TOF3000")]
ax.legend(handles, ["outer interlock (psu_watchdog)", "firmware interlock", "TOF3000 discharge"],
          loc="upper right", fontsize=9)
fig.text(0.0, -0.05, "Heartbeat: fault seen at the 13 s check, setting 10 s. Times from the real hardware runs of "
         "2026-11-24.", color=MUTED, fontsize=8.5)
save(fig, "fig2_response_times.png")

# ---------------------------------------------------------------- fig 3: test tally
# all suites run on 2026-11-24 (Stage7_summary.md section 4)
suites = [
    ("spd3303x.py, simulated supply", 32, 0),
    ("spd3303x.py, real SPD3303X (unloaded, <= 5 V)", 24, 1),
    ("psu_watchdog, simulated supply + Arduino", 54, 0),
    ("psu_watchdog process, real SPD3303X", 13, 0),
    ("real PCB + SPD3303X, no TOF3000 (2 runs)", 62, 0),
    ("stage 7 sequence, simulated TOF3000", 23, 0),
    ("stage 7, real TOF3000, run 1", 22, 1),
    ("stage 7, real TOF3000, run 2", 23, 0),
    ("GUI remote control, simulated board", 13, 0),
]
fig, ax = plt.subplots(figsize=(7.4, 4.2))
y = list(range(len(suites)))[::-1]
for yi, (name, ok, bad) in zip(y, suites):
    ax.barh(yi, ok, height=0.55, color=S1, edgecolor=SURF, linewidth=2, zorder=3)
    if bad:
        ax.barh(yi, bad, left=ok, height=0.55, color=CRIT, edgecolor=SURF, linewidth=2, zorder=3)
    ax.text(ok + bad + 1, yi, "%d/%d" % (ok, ok + bad), va="center", color=INK2, fontsize=9)
ax.set_yticks(y)
ax.set_yticklabels([s[0] for s in suites])
ax.grid(axis="y", visible=False)
ax.set_xlim(0, 72)
ax.set_xlabel("checks")
ax.set_title("Test checks run on 2026-11-24")
ax.legend([plt.Rectangle((0, 0), 1, 1, color=S1), plt.Rectangle((0, 0), 1, 1, color=CRIT)],
          ["passed", "failed"], loc="lower right", fontsize=9)
fig.text(0.0, -0.06, "Failures: the SPD3303X does not report undefined commands in SYSTem:ERRor? (driver relies on "
         "read-back);" + NL + "GMC-800 port error 31 in stage 7 run 1 (fixed by a USB replug, run 2 passed).",
         color=MUTED, fontsize=8.5)
save(fig, "fig3_test_tally.png")

# ---------------------------------------------------------------- fig 4: system and interlock layers
fig, ax = plt.subplots(figsize=(10.4, 5.2))
ax.set_xlim(0, 100)
ax.set_ylim(-7, 58)
ax.axis("off")


def box(x, y, w, h, title, sub, edge):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.4,rounding_size=1.2", fc="white",
                                ec=edge, lw=1.6))
    ax.text(x + w / 2, y + h - 2.0, title, ha="center", va="top", fontsize=10, fontweight="bold", color=INK)
    ax.text(x + w / 2, y + h - 5.4, sub, ha="center", va="top", fontsize=8.2, color=INK2, linespacing=1.35)


def arrow(x0, y0, x1, y1, col):
    ax.annotate("", (x1, y1), (x0, y0),
                arrowprops=dict(arrowstyle="-|>", color=col, lw=1.4, shrinkA=3, shrinkB=3))


def label(x, y, text, ha="center", va="bottom"):
    ax.text(x, y, text, ha=ha, va=va, fontsize=8, color=INK2, linespacing=1.3)


ax.add_patch(FancyBboxPatch((1, 30), 49, 26, boxstyle="round,pad=0.4,rounding_size=1.5", fc="#f4f3f0", ec=GRID,
                            lw=1))
ax.text(2.5, 54.8, "Lab PC", fontsize=9, color=MUTED, va="top")
box(3, 35, 19, 16, "hv_bringup_gui.py",
    NL.join(["HVLink + permit", "(0.9 kV cap, watchdog)", "WatchdogClient", "remote :50734"]), S1)
box(30, 35, 18, 16, "psu_watchdog.py",
    NL.join(["own process,", "owns the SPD3303X,", "trips on lost proof", "of control (latched)"]), S1)
arrow(22.4, 43, 29.6, 43, S1)
label(26, 44, NL.join(["UDP", "0.5 s"]))
box(62, 39, 19, 12, "SPD3303X CH1", NL.join(["24 V, limit 1.5 A", "0.32-0.37 A measured"]), S1)
arrow(48.4, 45, 61.6, 45, S1)
label(55, 46, NL.join(["USB-TMC", "(VISA)"]))
box(3, 4, 19, 14, "GMC-800", NL.join(["Geiger counter, COM5", "background", "0.078 uSv/h"]), S2)
arrow(12.5, 18.4, 12.5, 34.6, S2)
label(13.5, 24, "dose, 1 Hz", ha="left")
box(26, 4, 22, 16, "HV_control PCB",
    NL.join(["Arduino Nano Every, COM4", "FW 2.8, board TOF+, Pol +", "firmware interlocks:",
             "heartbeat, X-ray, faults"]), S2)
arrow(17, 34.6, 33, 20.4, S2)
label(26, 27.5, NL.join(["USB serial", "s, d, e, x, z"]), ha="left")
box(58, 4, 16, 16, "TOF3000", NL.join(["HV supply,", "polarity set by", "the board (+)"]), S3)
arrow(48.4, 12, 57.6, 12, S2)
label(53, 10.5, NL.join(["J1: v_prog,", "HV_enable"]), va="top")
arrow(71.5, 38.6, 68, 20.4, S1)
arrow(64, 38.6, 44, 20.4, S1)
label(72, 29, "24 V", ha="left")
box(82, 4, 16, 16, "load", NL.join(["1200 MOhm (temp.)", "+ corona nut in", "insulated box"]), S3)
arrow(74.4, 12, 81.6, 12, S3)
label(78, 10.5, NL.join(["<= 0.8", "kV"]), va="top")
ax.text(1, -6, "Outer interlock (blue) cuts 24 V when the PC loses control of the Arduino. Firmware interlocks "
        "(orange) act inside the board." + NL + "The TOF3000 output (aqua) is off whenever HV_enable is high "
        "(test 1b).", fontsize=8.2, color=MUTED, va="bottom")
ax.set_title("Setup tested on 2026-11-24", loc="left")
save(fig, "fig4_setup.png")
