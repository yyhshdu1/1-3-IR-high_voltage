"""Plots of the April 2025 TOF3000 test data (Short IR electrodes test, NOT the Emma folder).

Usage:  python plot_old_tof3000_data.py [data_folder] [out_folder]
Conversions (user confirmed): 10 V = 30 kV (3 kV/V), I_mon 10 V = 400 uA (40 uA/V).
I_mon offsets (settled readings at 0 V, see OFF) are subtracted.
"""
import glob
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = sys.argv[1] if len(sys.argv) > 1 else r'C:\Users\yuanhang.yang\Documents\GitHub\1-3-IR-high_voltage\Short IR electrodes test'
OUT = sys.argv[2] if len(sys.argv) > 2 else os.path.join(os.path.dirname(os.path.abspath(__file__)), 'old_data_plots')
os.makedirs(OUT, exist_ok=True)

KV, UA = 3.0, 40.0
OFF = {'1': (0.0010, 0.0037), '2': (0.0008, 0.0053), '3': (0.0008, 0.0053)}   # (neg, pos) I_mon offset in V, by day (start of the up-ramp files)
CN, CP = '#1f77b4', '#d62728'

plt.rcParams.update({'font.size': 10, 'axes.grid': True, 'grid.alpha': 0.3, 'figure.dpi': 110})


def load(rel):
    d = np.genfromtxt(os.path.join(ROOT, rel), delimiter=',', names=True)
    day = '1'
    if '4-2-2025' in rel:
        day = '2'
    elif '4-3-2025' in rel:
        day = '3'
    on, op = OFF[day]
    return dict(t=d['DAQ_Elapsed_Time'], vn=d['V_mon_neg'] * KV, vp=d['V_mon_pos'] * KV,
                inn=(d['I_mon_neg'] - on) * UA, ip=(d['I_mon_pos'] - op) * UA)


def binned(v, i, edges):
    c, med, mx = [], [], []
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (v >= lo) & (v < hi)
        if m.sum() < 3:
            continue
        c.append(0.5 * (lo + hi))
        med.append(np.median(i[m]))
        mx.append(i[m].max())
    return np.array(c), np.array(med), np.array(mx)


slow = load('4-2-2025_test/ramp_and_record_0.000V_to_10.000V_step_0.001V_in_1800s_1.csv')
slow_dn = load('4-2-2025_test/ramp_and_record_10.000V_to_0.000V_step_0.001V_in_1800s_1.csv')
d3 = load('4-3-2025_test/ramp_and_record_0.000V_to_10.000V_step_0.01V_in_120s_0.csv')

# ------------------------------------------------------------------ Fig 1: 04-02 slow ramp
fig, ax = plt.subplots(3, 1, figsize=(11, 10), sharex=True, gridspec_kw={'height_ratios': [1, 2, 2]})
t = slow['t']
ax[0].plot(t, slow['vn'], color=CN, label='V_mon neg')
ax[0].plot(t, slow['vp'], color=CP, ls='--', label='V_mon pos')
ax[0].set_ylabel('HV (kV)')
ax[0].legend(loc='upper left')
ax[0].set_title('04-02: 1800 s ramp, 0 -> 30 kV, 0.001 V DAQ steps')
ax[1].plot(t, slow['inn'], color=CN, lw=0.6, label='I_mon neg')
ax[1].plot(t, slow['ip'], color=CP, lw=0.6, alpha=0.8, label='I_mon pos')
ax[1].set_ylabel('current (uA), full scale')
ax[1].legend(loc='upper left')
for tt, txt in ((1235, '20.4 kV\n117 uA'), (1810, '29.5-30 kV\n6 sparks')):
    ax[1].annotate(txt, (tt, 110), xytext=(tt - 250, 95), arrowprops=dict(arrowstyle='->'), fontsize=9)
ax[2].plot(t, slow['inn'], color=CN, lw=0.6)
ax[2].plot(t, slow['ip'], color=CP, lw=0.6, alpha=0.8)
ax[2].set_ylim(-1, 8)
ax[2].set_ylabel('current (uA), zoom')
ax[2].set_xlabel('time (s)')
ax[2].axvline(770, color='k', ls=':')
ax[2].text(640, 7, 'first micro-discharges 12.7 kV', fontsize=9, ha='left')
ax[2].axvline(806, color='k', ls=':')
ax[2].text(815, 6.2, 'bursts, median 3.5-4 uA, from 13.4 kV', fontsize=9)
ax[2].axvline(1816, color='g', ls='--')
ax[2].text(1500, 7, 'reaches 30 kV', fontsize=9, color='g')
fig.tight_layout()
fig.savefig(os.path.join(OUT, '1_slow_ramp_0402.png'))
plt.close(fig)

# ------------------------------------------------------------------ Fig 2: I vs V, all up-ramps by day
fig, ax = plt.subplots(1, 2, figsize=(13, 5.5), sharey=True)
edges = np.arange(0, 30.01, 1.0)
groups = [
    ('04-01  0.1 V steps, 120 s (5 ramps)', sorted(glob.glob(os.path.join(ROOT, '4-1-2025_test', 'ramp_and_record_0.000V_to_10.000V_step_0.1V_in_120s_*.csv'))), 'tab:green'),
    ('04-02  0.05 V steps, 90 s (3 ramps)', sorted(glob.glob(os.path.join(ROOT, '4-2-2025_test', 'ramp_and_record_0.000V_to_10.000V_step_0.05V_in_90s_*.csv'))), 'tab:olive'),
    ('04-02  0.001 V steps, 1800 s', [os.path.join(ROOT, '4-2-2025_test', 'ramp_and_record_0.000V_to_10.000V_step_0.001V_in_1800s_1.csv')], 'tab:red'),
    ('04-03  0.01 V steps, 120 s', [os.path.join(ROOT, '4-3-2025_test', 'ramp_and_record_0.000V_to_10.000V_step_0.01V_in_120s_0.csv')], 'tab:purple'),
]
for k, (side, key, vk) in enumerate((('neg', 'inn', 'vn'), ('pos', 'ip', 'vp'))):
    for label, files, col in groups:
        vs, is_ = [], []
        for f in files:
            d = load(os.path.relpath(f, ROOT))
            vs.append(d[vk])
            is_.append(d[key])
        v, i = np.concatenate(vs), np.concatenate(is_)
        c, med, mx = binned(v, i, edges)
        ax[k].plot(c, np.clip(med, 1e-2, None), '-o', ms=3, color=col, label=label + ' (median)')
        ax[k].plot(c, np.clip(mx, 1e-2, None), ':', color=col, alpha=0.7)
    ax[k].set_yscale('log')
    ax[k].set_xlabel('HV (kV)')
    ax[k].set_title('I_mon %s supply (solid: median per 1 kV, dotted: max)' % side)
ax[0].set_ylabel('current (uA), offset removed')
ax[0].legend(fontsize=8, loc='lower right')
ax[0].set_ylim(1e-2, 300)
fig.tight_layout()
fig.savefig(os.path.join(OUT, '2_current_vs_voltage_by_day.png'))
plt.close(fig)

# ------------------------------------------------------------------ Fig 3: 04-03 ramp and discharge decay
fig, ax = plt.subplots(1, 2, figsize=(13, 5))
ax[0].plot(d3['vn'], d3['inn'], color=CN, lw=0.8, label='I_mon neg')
ax[0].plot(d3['vp'], d3['ip'], color=CP, lw=0.8, label='I_mon pos')
ax[0].set_xlabel('HV (kV)')
ax[0].set_ylabel('current (uA)')
ax[0].set_title('04-03: 0.01 V steps, 120 s - jump at 21 kV, then smooth', fontsize=10)
ax[0].legend()
m = slow['t'] > 1810
tt = slow['t'][m] - 1816
ax[1].semilogy(tt, np.clip(slow['inn'][m], 0.05, None), color=CN, lw=0.7, label='I_mon neg')
ax[1].semilogy(tt, np.clip(slow['ip'][m], 0.05, None), color=CP, lw=0.7, label='I_mon pos')
ax[1].set_xlabel('time after reaching 30 kV (s)')
ax[1].set_ylabel('current (uA)')
ax[1].set_title('04-02: decay after reaching 30 kV (halves every ~20 s)', fontsize=10)
ax[1].legend()
fig.tight_layout()
fig.savefig(os.path.join(OUT, '3_0403_ramp_and_decay.png'))
plt.close(fig)

# ------------------------------------------------------------------ Fig 4: holds at 30 kV and 0 V offsets
holds = [('04-01 long test, 3600 s', '4-1-2025_long_test/rec_at_10__recording_3600s_0.csv'),
         ('04-02, 3600 s', '4-2-2025_test/rec_at_10__recording_3600s_0.csv')]
fig, ax = plt.subplots(2, 2, figsize=(13, 7), sharex='col')
for c, (title, rel) in enumerate(holds):
    d = load(rel)
    ax[0, c].plot(d['t'], d['vn'] - 30 * 0 , color=CN, lw=0.5, label='V_mon neg')
    ax[0, c].plot(d['t'], d['vp'], color=CP, lw=0.5, label='V_mon pos')
    ax[0, c].set_ylim(29.97, 30.03)
    ax[0, c].set_title(title + ' at 10 V (30 kV)')
    ax[0, c].set_ylabel('HV (kV)')
    ax[0, c].legend(loc='upper right')
    ax[1, c].plot(d['t'], d['inn'], color=CN, lw=0.5, label='I_mon neg')
    ax[1, c].plot(d['t'], d['ip'], color=CP, lw=0.5, label='I_mon pos')
    ax[1, c].set_ylabel('current (uA)')
    ax[1, c].set_xlabel('time (s)')
    ax[1, c].legend(loc='upper right')
    ax[1, c].set_ylim(-0.1, 0.6)
fig.tight_layout()
fig.savefig(os.path.join(OUT, '4_holds_at_30kV.png'))
plt.close(fig)

# ------------------------------------------------------------------ Fig 5: down ramps (bleed current vs ramp rate) and V tracking
fig, ax = plt.subplots(1, 2, figsize=(13, 5))
dn = [('0.1 V steps, 60 s (0.5 kV/s)', '4-1-2025_test/ramp_and_record_10.000V_to_0.000V_step_0.1V_in_60s_1.csv', 'tab:green'),
      ('0.05 V steps, 60 s (0.5 kV/s)', '4-2-2025_test/ramp_and_record_10.000V_to_0.000V_step_0.05V_in_60s_1.csv', 'tab:olive'),
      ('0.001 V steps, 1800 s (17 V/s)', '4-2-2025_test/ramp_and_record_10.000V_to_0.000V_step_0.001V_in_1800s_1.csv', 'tab:red')]
for label, rel, col in dn:
    d = load(rel)
    ax[0].plot(d['vn'], d['inn'], lw=0.7, color=col, label=label)
ax[0].set_xlabel('HV (kV)')
ax[0].set_ylabel('I_mon neg (uA)')
ax[0].set_title('Down ramps: I_mon follows the steps and the ramp rate (not sparks)', fontsize=10)
ax[0].legend(fontsize=8)
ax[0].set_ylim(-0.1, 8)
for label, rel, T, col in (('up 0.1 V steps, 120 s', '4-1-2025_test/ramp_and_record_0.000V_to_10.000V_step_0.1V_in_120s_1.csv', 120, 'tab:green'),
                          ('down 0.1 V steps, 60 s', '4-1-2025_test/ramp_and_record_10.000V_to_0.000V_step_0.1V_in_60s_1.csv', 60, 'tab:blue')):
    d = load(rel)
    tt = d['t']
    prog = np.interp(tt, [0, T], [0, 30] if 'up' in label else [30, 0])
    ax[1].plot(tt, d['vn'] - prog, color=col, label=label + ': V_mon - linear program')
ax[1].set_xlabel('time (s)')
ax[1].set_ylabel('difference (kV)')
ax[1].set_title('V_mon minus linear program: 0.3 kV staircase, lag under 1 s', fontsize=10)
ax[1].legend(fontsize=8)
fig.tight_layout()
fig.savefig(os.path.join(OUT, '5_down_ramps_and_tracking.png'))
plt.close(fig)

# ------------------------------------------------------------------ Fig 6: 0 V offsets and noise
fig, ax = plt.subplots(1, 2, figsize=(13, 4.5))
for day, folder, ls in (('04-01', '4-1-2025_test', '-'), ('04-02', '4-2-2025_test', '--')):
    fs = sorted(glob.glob(os.path.join(ROOT, folder, 'rec_at_0__recording_60s_*.csv')))
    d = np.genfromtxt(fs[0], delimiter=',', names=True)
    ax[0].plot(d['DAQ_Elapsed_Time'], d['V_mon_neg'] * 1000, color=CN, ls=ls, lw=0.8, label='V_mon neg ' + day)
    ax[0].plot(d['DAQ_Elapsed_Time'], d['V_mon_pos'] * 1000, color=CP, ls=ls, lw=0.8, label='V_mon pos ' + day)
    ax[1].plot(d['DAQ_Elapsed_Time'], d['I_mon_neg'] * 1000, color=CN, ls=ls, lw=0.8, label='I_mon neg ' + day)
    ax[1].plot(d['DAQ_Elapsed_Time'], d['I_mon_pos'] * 1000, color=CP, ls=ls, lw=0.8, label='I_mon pos ' + day)
ax[0].set_ylabel('V_mon (mV)')
ax[1].set_ylabel('I_mon at 0 V (mV)')
for a in ax:
    a.set_xlabel('time (s)')
    a.legend(fontsize=8)
ax[0].set_title('V_mon after programming 0 V (60 s): residual HV decays (1 mV = 3 V)', fontsize=10)
ax[0].axhline(6.0, color='k', ls=':')
ax[0].text(30, 6.6, 'settled offset about 6 mV (end of 1800 s ramp-down)', fontsize=8)
ax[1].set_title('I_mon after programming 0 V (1 mV = 0.04 uA)', fontsize=10)
fig.tight_layout()
fig.savefig(os.path.join(OUT, '6_zero_offsets.png'))
plt.close(fig)

print('saved to', OUT)
