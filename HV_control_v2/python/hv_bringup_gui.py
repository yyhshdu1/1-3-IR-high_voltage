"""
hv_bringup_gui.py - GUI for the first power-up procedure of the HV_control board
(firmware HV_control_v2).  Follows ../首次上电步骤.md, stages 3-8.

Run:   python hv_bringup_gui.py
Needs: pyserial, matplotlib (tkinter is part of Python).
"""
import collections
import datetime
import json
import os
import queue
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import messagebox, simpledialog, ttk

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402

import bringup_stages as bs  # noqa: E402
import psu_watchdog as pwd  # noqa: E402
import stage7_tests as s7  # noqa: E402
from hv_link import (CH_NAMES, FLT_BITS, RST_BITS, HVError, HVLink,  # noqa: E402
                     decode_bits)

matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

HERE = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(HERE, "logs")
REMOTE_ADDR = ("127.0.0.1", 50734)     # local remote control (hv_remote.py), off unless enabled

CONFIG_KEYS = [  # (key, label, note)
    ("board", "board", "A/B (改动需 HV 关且 DAC=0)"),
    ("pol", "pol", "开机极性: 1=正(引脚高), 0=负(引脚低); 每块板对应一台电源 (需 HV 关且 DAC=0, 改后点保存)"),
    ("name", "name", "这块板接的电源标签, 1-8 字符, 如 TOF+ / TOF-; '-' 清空"),
    ("pid", "pid", "1=允许接入积分器(r 0), 默认 0=锁住; 本项目不用 PID, 保持 0"),
    ("cable", "cable", "1=HV 开且 v_mon/i_mon 悬空到满量程(J1 线缆没插)时降压并锁故障 0x80; 阶段 7 起设 1"),
    ("scale", "scale", "主机单位→DAC 码系数 (需 HV 关且 DAC=0)"),
    ("dacmax", "dacmax", "主机单位上限 (需 HV 关且 DAC=0)"),
    ("up", "up", "上升速率 主机单位/s, 0=不限速(仅台架)"),
    ("down", "down", "下降速率 主机单位/s"),
    ("timeout", "timeout", "主机心跳超时 s, 0=关闭"),
    ("vprogmax", "vprogmax", "v_prog 过压阈值 V (1-12)"),
    ("vprogtol", "vprogtol", "DAC 回读容差 V"),
    ("ethr", "ethr", "低阈值 ADC 码(只计数, 'ev'); 1 码=15 nA, 100 码≈1.5 µA"),
    ("sthr", "sthr", "高阈值 ADC 码(打火 'sp', 触发回退); 2500 码≈37.5 µA (FW 2.5 是 1300≈20 µA)"),
    ("spark", "spark", "窗口内 N 次打火(高阈值)触发回退/关断, 0=只计数"),
    ("sparkwin", "sparkwin", "打火统计窗口 s"),
    ("backoff", "backoff", "打火回退幅度(主机单位), 0=改为关断"),
    ("xthr", "xthr", "X 射线告警阈值, 单位 0.01 µSv/h (40=0.40 µSv/h), 0=关闭"),
    ("xback", "xback", "X 射线每步回退(主机单位, 3936≈2 kV), 0=降到0并锁故障"),
    ("xtimeout", "xtimeout", "X 射线监测丢失超时 s (HV 开且 DAC>0 时), 0=关闭"),
]


class GuiCtx(object):
    """Context for stage7_tests.run(): what the test sequence may use from the GUI."""

    def __init__(self, app):
        self.app = app
        self.link = app.link
        self.stop = app.job_stop
        self.emit = app._emit_to(app.tree7)

    def log(self, msg):
        self.app.log(msg)

    def ask(self, msg):
        return self.app.ask(msg)

    def step(self, text):
        self.app.log("== %s" % text)
        self.app.ui(lambda: self.app.s7_step_var.set(text))

    def wd(self):
        return self.app.wd

    def wd_status(self):
        return pwd.request({"type": "status"}, timeout=1.0)

    def wd_restart(self):
        if not pwd.is_running():
            self.app._wd_spawn()
        if self.app.wd is None:
            return False
        self.app.wd.arm()
        time.sleep(1.0)
        try:
            return self.wd_status().get("state") == pwd.ARMED
        except pwd.WatchdogError:
            return False

    def wd_kill(self):
        pid = self.wd_status().get("pid")
        self.app.log("killing the watchdog process (pid %s) on purpose" % pid)
        os.kill(int(pid), 15)           # Windows: TerminateProcess, like Task Manager

    def wd_trip(self, reason):
        return pwd.request({"type": "trip", "client": "stage7-test", "reason": reason})

    def make_gmc(self, port):
        import xray_guard
        src = xray_guard.Gmc800Source(port)
        self.app.log("GMC-800: %s" % src.setup())
        return src

    def xray_start(self, src):
        import xray_guard
        self.xray_stop()
        g = xray_guard.XrayGuard(src, self.link, period=1.0, log=self.app.log, xthr_uSv=0.40)
        g.start()
        self.app.xsource, self.app.xguard = src, g
        self.app.log("X-ray forwarding started (%s)" % type(src).__name__)
        return g

    def xray_stop(self):
        self.app.x_disconnect()

    def xray_set_source(self, src):
        g, old = self.app.xguard, self.app.xsource
        g.source = src
        self.app.xsource = src
        self.app.log("X-ray source switched to %s" % type(src).__name__)
        if old is not src and hasattr(old, "close"):
            threading.Timer(2.0, old.close).start()     # let a read in progress finish first


class App(object):
    def __init__(self, root, opts=None):
        self.root = root
        root.title("HV_control 首次上电测试")
        root.geometry("1180x900")
        try:
            ttk.Style().configure(".", font=("Microsoft YaHei UI", 9))
        except tk.TclError:
            pass

        os.makedirs(LOG_DIR, exist_ok=True)
        self.session_ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_path = os.path.join(LOG_DIR, "bringup_%s.log" % self.session_ts)
        self.log_file = open(self.log_path, "a", encoding="utf-8")

        self.q = queue.Queue()
        self.link = HVLink(log=self.log)
        self.poll_stop = threading.Event()
        self.poll_thread = None
        self.user_disc = False
        self.job_thread = None
        self.job_stop = threading.Event()
        self.sweep_rows = []
        self.last_fit = None
        self.cfg_cache = {}
        self.history = collections.deque(maxlen=1500)
        self.t0 = time.time()
        self.last_flt = 0
        self.last_ev = 0
        self.stair_thread = None

        # 24 V supply outer interlock (psu_watchdog.py runs as its own process)
        self.wd = None                 # WatchdogClient for self.link, created on connect
        self.wd_require = True         # HV commands need an armed, healthy watchdog
        self.wd_last = (None, "")      # (last watchdog reply, client event)
        self.wd_event_shown = ""
        self.wd_status_busy = False
        self.hv_cap_units = None       # session cap on "d N" (host units), from --hv-max-kv / stage 7 tests
        self.link.permit = self._hv_permit
        self.opts = opts or {}

        self._build_ui()
        self.refresh_ports()
        self.set_connected(False)
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        root.after(50, self.drain)
        root.after(1000, self.plot_tick)

    # =========================================================== thread bridge
    def ui(self, fn):
        self.q.put(fn)

    def drain(self):
        try:
            for _ in range(300):
                fn = self.q.get_nowait()
                try:
                    fn()
                except Exception as e:  # never let one bad callback kill the loop
                    self._append_log("UI 回调错误: %r" % (e,))
        except queue.Empty:
            pass
        self.root.after(50, self.drain)

    def log(self, msg):
        stamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        line = "%s  %s" % (stamp, msg)
        try:
            self.log_file.write(line + "\n")
            self.log_file.flush()
        except Exception:
            pass
        self.ui(lambda: self._append_log(line))

    def _append_log(self, line):
        self.log_text.config(state="normal")
        self.log_text.insert("end", line + "\n")
        self.log_text.see("end")
        self.log_text.config(state="disabled")

    def ask(self, msg):
        """Yes/no question from a worker thread.  The dialog is opened from the Tk event loop
        (after(0)), NOT from inside drain(), so status/log updates keep running while it is open."""
        res = queue.Queue()

        def show():
            res.put(messagebox.askyesno("请确认", msg, parent=self.root))
        self.ui(lambda: self.root.after(0, show))
        return res.get()

    # ================================================================== UI
    def _build_ui(self):
        top = ttk.Frame(self.root, padding=4)
        top.pack(fill="x")
        ttk.Label(top, text="串口:").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(top, textvariable=self.port_var, width=34, state="readonly")
        self.port_box.pack(side="left", padx=2)
        ttk.Button(top, text="刷新", width=5, command=self.refresh_ports).pack(side="left")
        self.btn_connect = ttk.Button(top, text="连接", width=8, command=self.on_connect)
        self.btn_connect.pack(side="left", padx=4)
        self.conn_label = ttk.Label(top, text="未连接", foreground="gray")
        self.conn_label.pack(side="left", padx=6)

        tk.Button(top, text="急停 (z)", bg="#cc0000", fg="white", font=("Microsoft YaHei UI", 11, "bold"),
                  width=10, command=self.on_estop).pack(side="right", padx=4)
        ttk.Button(top, text="停止当前任务", command=self.on_stop_job).pack(side="right", padx=4)
        ttk.Button(top, text="清故障 (C)", command=lambda: self.quick("C")).pack(side="right", padx=4)
        ttk.Button(top, text="导出报告", command=self.export_report).pack(side="right", padx=4)

        # ---- status strip
        sf = ttk.LabelFrame(self.root, text="实时状态 (每 0.5 s 轮询; 轮询本身也是主机心跳)", padding=4)
        sf.pack(fill="x", padx=4)
        self.sv = {}
        fields = [("up", "运行时间(s)"), ("rst", "复位原因"), ("hv", "HV"), ("pol", "极性"),
                  ("rly", "继电器"), ("int", "积分器"), ("tgt", "目标"), ("cur", "当前"),
                  ("raw", "DAC码"), ("ev", "放电数"), ("sp", "打火数"), ("backoffs", "回退次数"),
                  ("dacfix", "DAC重写"), ("xr", "X射线.01µSv/h"), ("xh", "X告警"), ("xtrips", "X回退次数")]
        for i, (k, label) in enumerate(fields):
            row, idx = (0, i) if i < 9 else (2, i - 9)
            ttk.Label(sf, text=label + ":").grid(row=row, column=2 * idx, sticky="e", padx=(6, 1))
            self.sv[k] = tk.StringVar(value="-")
            ttk.Label(sf, textvariable=self.sv[k], width=7, foreground="#003399").grid(
                row=row, column=2 * idx + 1, sticky="w")
        ttk.Label(sf, text="故障:").grid(row=1, column=0, sticky="e", padx=(6, 1), pady=2)
        self.sv["flt"] = tk.StringVar(value="-")
        self.flt_label = ttk.Label(sf, textvariable=self.sv["flt"], foreground="#006600")
        self.flt_label.grid(row=1, column=1, columnspan=5, sticky="w")
        self.vv = []
        for i, n in enumerate(CH_NAMES):
            ttk.Label(sf, text="CH%d %s:" % (i, n)).grid(row=1, column=6 + 2 * i, sticky="e", padx=(6, 1))
            var = tk.StringVar(value="-")
            self.vv.append(var)
            ttk.Label(sf, textvariable=var, width=9, foreground="#003399").grid(row=1, column=7 + 2 * i, sticky="w")

        self._build_psu_strip()

        # ---- notebook + log
        pw = ttk.PanedWindow(self.root, orient="vertical")
        pw.pack(fill="both", expand=True, padx=4, pady=4)
        self.nb = ttk.Notebook(pw)
        pw.add(self.nb, weight=4)
        logf = ttk.Frame(pw)
        pw.add(logf, weight=1)
        self.log_text = tk.Text(logf, height=8, state="disabled", font=("Consolas", 9), wrap="none")
        sb = ttk.Scrollbar(logf, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=sb.set)
        self.log_text.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")

        self._build_stage3()
        self._build_stage4()
        self._build_stage5()
        self._build_stage6()
        self._build_stage78()
        self._build_xray()
        self._build_config()
        self._build_stage7tests()

    def _make_tree(self, parent, cols, widths, height=10):
        frame = ttk.Frame(parent)
        tree = ttk.Treeview(frame, columns=[c for c, _ in cols], show="headings", height=height)
        for (c, label), w in zip(cols, widths):
            tree.heading(c, text=label)
            tree.column(c, width=w, anchor="w")
        sb = ttk.Scrollbar(frame, command=tree.yview)
        tree.configure(yscrollcommand=sb.set)
        tree.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")
        tree.tag_configure("pass", foreground="#007700")
        tree.tag_configure("fail", foreground="#cc0000")
        tree.tag_configure("info", foreground="#444444")
        return frame, tree

    def _emit_to(self, tree):
        def emit(name, ok, detail):
            tag = "pass" if ok is True else ("fail" if ok is False else "info")
            res = "PASS" if ok is True else ("FAIL" if ok is False else "信息")
            self.log("[%s] %s | %s" % (res, name, detail))
            self.ui(lambda: tree.insert("", "end", values=(name, res, detail), tags=(tag,)))
        return emit

    def _clear_tree(self, tree):
        for i in tree.get_children():
            tree.delete(i)

    # ---------------------------------------------------------------- stage 3
    def _build_stage3(self):
        f = ttk.Frame(self.nb, padding=6)
        self.nb.add(f, text="阶段3 开机检查")
        ttk.Label(f, justify="left", text=(
            "前提: 已完成阶段 1-2 (不上电检查 / 轨电压), Arduino 已装好并烧录正式版 HV_control_v2.\n"
            "①自动检查开机状态; ②连续复位3次(会重新打开串口), 同时用示波器看 J1-12: 复位期间会有短暂高电平脉冲(R14上拉), 之后回到 0 V,\n"
            "并观察继电器 K1 在复位瞬间是否抖动。继电器实际极性在阶段 6 里人工确认。")).pack(anchor="w")
        bf = ttk.Frame(f)
        bf.pack(fill="x", pady=4)
        ttk.Button(bf, text="① 开机状态自动检查", command=self.run_stage3).pack(side="left", padx=2)
        ttk.Button(bf, text="② 复位测试 ×3", command=self.run_reset_test).pack(side="left", padx=2)
        ttk.Button(bf, text="清空", command=lambda: self._clear_tree(self.tree3)).pack(side="left", padx=2)
        fr, self.tree3 = self._make_tree(f, [("name", "项目"), ("res", "结果"), ("detail", "详情")],
                                         [260, 70, 700], 14)
        fr.pack(fill="both", expand=True)

    def run_stage3(self):
        emit = self._emit_to(self.tree3)
        self.run_job("阶段3 开机检查", lambda stop: bs.stage3_boot_check(self.link, emit, self.log))

    def run_reset_test(self):
        if not messagebox.askokcancel("复位测试", "将重新打开串口 3 次, Arduino 会复位 3 次。\n"
                                      "请确认 J1 未接 TOF3000, 并准备好观察 J1-12 / K1。继续?"):
            return
        emit = self._emit_to(self.tree3)
        self.run_job("复位测试", lambda stop: bs.stage3_reset_test(self.link, emit, self.log, 3))

    # ---------------------------------------------------------------- stage 4
    def _build_stage4(self):
        f = ttk.Frame(self.nb, padding=6)
        self.nb.add(f, text="阶段4 ADC标定")
        ttk.Label(f, justify="left", text=(
            "在 J1-5 (voltage_monitor) 或 J1-13 (current_monitor) 加已知电压 (台式电源, 万用表测准),\n"
            "选通道, 填入万用表读数, 点'采集并记录'。判据: |误差| <= max(1%, 30 mV)。\n"
            "'读取全部通道'用于零输入时看五个通道的偏置/噪声。")).pack(anchor="w")
        bf = ttk.Frame(f)
        bf.pack(fill="x", pady=4)
        ttk.Label(bf, text="通道:").pack(side="left")
        self.ch_var = tk.StringVar(value="0")
        ttk.Combobox(bf, textvariable=self.ch_var, width=22, state="readonly", values=[
            "0 v_mon (J1-5)", "1 i_mon (J1-13)", "2 v_prog", "3 v_div", "4 v_err"]).pack(side="left", padx=2)
        self.ch_var.set("0 v_mon (J1-5)")
        ttk.Label(bf, text="施加电压 V (万用表):").pack(side="left", padx=(10, 1))
        self.applied_var = tk.StringVar(value="0")
        ttk.Entry(bf, textvariable=self.applied_var, width=8).pack(side="left")
        ttk.Label(bf, text="采样数:").pack(side="left", padx=(10, 1))
        self.nsamp4_var = tk.StringVar(value="20")
        ttk.Entry(bf, textvariable=self.nsamp4_var, width=5).pack(side="left")
        ttk.Button(bf, text="采集并记录", command=self.run_adc_capture).pack(side="left", padx=8)
        ttk.Button(bf, text="读取全部通道", command=self.run_adc_all).pack(side="left", padx=2)
        ttk.Button(bf, text="清空", command=lambda: self._clear_tree(self.tree4)).pack(side="left", padx=2)
        fr, self.tree4 = self._make_tree(f, [
            ("ch", "通道"), ("applied", "施加(V)"), ("meas", "读数(V)"), ("err", "误差(mV)"),
            ("errp", "误差(%)"), ("sigma", "σ(mV)"), ("res", "结果")],
            [140, 90, 100, 90, 90, 90, 80], 14)
        fr.pack(fill="both", expand=True)

    def run_adc_capture(self):
        try:
            ch = int(self.ch_var.get().split()[0])
            applied = float(self.applied_var.get())
            n = max(3, int(self.nsamp4_var.get()))
        except ValueError:
            messagebox.showerror("输入错误", "施加电压和采样数必须是数字")
            return

        def job(stop):
            means, stds = bs.adc_capture(self.link, n=n, interval=0.1, stop=stop)
            m, s = means[ch], stds[ch]
            err = m - applied
            tol = max(0.01 * abs(applied), 0.030)
            ok = abs(err) <= tol
            pct = (100.0 * err / applied) if applied else float("nan")
            vals = ("CH%d %s" % (ch, CH_NAMES[ch]), "%.4f" % applied, "%.4f" % m, "%+.1f" % (err * 1000),
                    "%.2f" % pct if applied else "-", "%.2f" % (s * 1000), "PASS" if ok else "FAIL")
            self.log("ADC标定 %s: 施加 %.4f V 读数 %.4f V 误差 %+.1f mV" % (vals[0], applied, m, err * 1000))
            self.ui(lambda: self.tree4.insert("", "end", values=vals, tags=("pass" if ok else "fail",)))
        self.run_job("ADC 采集", job)

    def run_adc_all(self):
        def job(stop):
            means, stds = bs.adc_capture(self.link, n=20, interval=0.1, stop=stop)
            for i in range(5):
                vals = ("CH%d %s" % (i, CH_NAMES[i]), "-", "%.4f" % means[i], "-", "-",
                        "%.2f" % (stds[i] * 1000), "信息")
                self.ui(lambda v=vals: self.tree4.insert("", "end", values=v, tags=("info",)))
            self.log("全通道: " + "  ".join("%s=%.4f V" % (CH_NAMES[i], means[i]) for i in range(5)))
        self.run_job("ADC 全通道", job)

    # ---------------------------------------------------------------- stage 5
    def _build_stage5(self):
        f = ttk.Frame(self.nb, padding=6)
        self.nb.add(f, text="阶段5 DAC/v_prog标定")
        ttk.Label(f, justify="left", text=(
            "前提: J1 未接 TOF3000。本扫描在 HV 关闭 (e 0) 下进行, DAC 输出照常, 只是不使能隔离器。\n"
            "万用表接 J1-2 (voltage_program) 对 GND。扫描每个码稳定后记录 ADC CH2; 请在表里双击'万用表'列填入实测值。\n"
            "'拟合'给出 V_prog=a·d+b, 以及 10.0 V (=30 kV) 对应的码、建议 dacmax 和 vprogmax (=V(dacmax)+0.5 V)。")).pack(anchor="w")
        bf = ttk.Frame(f)
        bf.pack(fill="x", pady=4)
        ttk.Label(bf, text="扫描码(主机单位,逗号分隔):").pack(side="left")
        self.codes_var = tk.StringVar(value="0,5000,20000,40000,59055")
        ttk.Entry(bf, textvariable=self.codes_var, width=34).pack(side="left", padx=2)
        ttk.Label(bf, text="稳定 s:").pack(side="left", padx=(8, 1))
        self.settle_var = tk.StringVar(value="2.0")
        ttk.Entry(bf, textvariable=self.settle_var, width=5).pack(side="left")
        ttk.Button(bf, text="运行扫描", command=self.run_sweep).pack(side="left", padx=8)
        ttk.Button(bf, text="DAC 回零", command=lambda: self.quick("d 0")).pack(side="left", padx=2)
        ttk.Button(bf, text="拟合并给出建议", command=self.run_fit).pack(side="left", padx=2)
        ttk.Button(bf, text="应用建议到板子", command=self.apply_fit).pack(side="left", padx=2)
        fr, self.tree5 = self._make_tree(f, [
            ("code", "码 d"), ("raw", "DAC码"), ("ch2", "ADC CH2(V)"), ("sigma", "σ(mV)"),
            ("dmm", "万用表 J1-2 (V) 双击输入"), ("diff", "万用表-CH2 (mV)")],
            [90, 90, 120, 90, 200, 140], 9)
        fr.pack(fill="both", expand=True)
        self.tree5.bind("<Double-1>", self.on_tree5_dblclick)
        self.fit_var = tk.StringVar(value="(尚未拟合)")
        ttk.Label(f, textvariable=self.fit_var, justify="left", foreground="#003399").pack(anchor="w", pady=4)

    def run_sweep(self):
        try:
            codes = [int(x) for x in self.codes_var.get().replace(" ", "").split(",") if x != ""]
            settle = float(self.settle_var.get())
        except ValueError:
            messagebox.showerror("输入错误", "码必须是整数, 稳定时间是数字")
            return
        if not codes or any(c < 0 for c in codes):
            messagebox.showerror("输入错误", "至少一个非负码")
            return
        if not messagebox.askokcancel("DAC 扫描", "确认 J1 未接 TOF3000 (或 J1-2 未接入任何负载)?\n"
                                      "DAC 将按固件速率爬升到最大码 %d。" % max(codes)):
            return
        self._clear_tree(self.tree5)
        self.sweep_rows = []
        self.fit_var.set("(扫描中...)")

        def row_cb(row):
            idx = len(self.sweep_rows)
            self.sweep_rows.append(dict(row, dmm=None))
            self.ui(lambda: self.tree5.insert("", "end", iid=str(idx), values=(
                row["code"], row["raw"], "%.4f" % row["ch2"], "%.2f" % (row["std"] * 1000), "", "")))

        def job(stop):
            bs.dac_sweep(self.link, codes, self.log, stop, settle=settle, nsamp=15, row_cb=row_cb)
            self.log("扫描结束, DAC 已回零。请填写万用表读数后点'拟合'。")
        self.run_job("DAC 扫描", job)

    def on_tree5_dblclick(self, event):
        iid = self.tree5.identify_row(event.y)
        col = self.tree5.identify_column(event.x)
        if not iid or col != "#5":
            return
        idx = int(iid)
        row = self.sweep_rows[idx]
        val = simpledialog.askfloat("万用表读数", "码 d=%d 时 J1-2 对地电压 (V):" % row["code"], parent=self.root)
        if val is None:
            return
        row["dmm"] = val
        vals = list(self.tree5.item(iid, "values"))
        vals[4] = "%.4f" % val
        vals[5] = "%+.1f" % ((val - row["ch2"]) * 1000)
        self.tree5.item(iid, values=vals)

    def run_fit(self):
        if len(self.sweep_rows) < 2:
            messagebox.showinfo("拟合", "至少需要 2 个扫描点")
            return

        def job():
            cfg = self.link.config()
            self.cfg_cache = cfg
            u30 = int(float(self.units30_var.get()))
            res = bs.analyse_dac(self.sweep_rows, scale=float(cfg["scale"]), units30=u30)
            self.last_fit = res
            if "error" in res:
                text = "拟合失败: %s" % res["error"]
            else:
                text = ("拟合来源: %s    V_prog = %.6g·d + %.4f V    最大残差 %.1f mV\n"
                        "当前 scale=%s 时, d=%d (=30 kV) 对应 V_prog=%.3f V; 达到 10.0 V 需要 d=%.0f (DAC 码 %.0f)\n"
                        "建议: scale = %s (使 d=%d <-> 10.0 V),  dacmax = %d,  vprogmax = %.2f V%s") % (
                    res["source"], res["slope"], res["offset"], res["resid"] * 1000,
                    cfg["scale"], u30, res["v_at_units30_now"], res["code_fs"], res["raw_fs"],
                    res["rec_scale"], u30, res["rec_dacmax"], res["rec_vprogmax"],
                    "" if res["scale_ok"] else "\n⚠ 建议 scale 超出固件允许范围 (0.05-3) 或 DAC 码 >65535, 需检查 DAC 增益/硬件")
            self.log(text.replace("\n", " | "))
            self.ui(lambda: self.fit_var.set(text))
        self.quick_fn(job)

    def apply_fit(self):
        if not self.last_fit or "rec_dacmax" not in self.last_fit:
            messagebox.showinfo("应用", "请先拟合")
            return
        r = self.last_fit
        if not r.get("scale_ok"):
            messagebox.showerror("应用", "建议值超出允许范围, 不能应用。请检查硬件/接线后重新扫描。")
            return
        if not messagebox.askokcancel("应用建议", "将执行:\n  d 0, e 0\n  c scale %s\n  c dacmax %d\n  c vprogmax %.2f\n"
                                      "(不会自动 c save, 确认无误后到'配置'页保存)\n继续?" % (
                                          r["rec_scale"], r["rec_dacmax"], r["rec_vprogmax"])):
            return

        def job(stop):
            self.link.cmd_ok("d 0")
            bs.wait_dac_settled(self.link, timeout=60, stop=stop)
            self.link.cmd_ok("e 0")
            self.link.cmd_ok("c scale %s" % r["rec_scale"])
            self.link.cmd_ok("c dacmax %d" % r["rec_dacmax"])
            self.link.cmd_ok("c vprogmax %.2f" % r["rec_vprogmax"])
            self.log("已应用 (RAM)。请重新扫描一次确认 d=%d 对应 10.0 V, 再到'配置'页保存到 EEPROM。" % r["rec_dacmax"])
        self.run_job("应用标定", job)

    # ---------------------------------------------------------------- stage 6
    def _build_stage6(self):
        f = ttk.Frame(self.nb, padding=6)
        self.nb.add(f, text="阶段6 联锁")
        ttk.Label(f, justify="left", text=(
            "前提: J1 未接 TOF3000 (本测试会在 HV 使能状态下驱动 DAC 到 3000 个单位)。\n"
            "自动检查: r/e/p/d 的联锁与错误回复、e 0 后 DAC 清零、校验和; 另有 FW 2.1 的'误差大时拒绝接入积分器'。\n"
            "J1 空载时 v_err≈-3 V, 积分器一接入就会把 v_prog 顶到电源轨 (实测 <0.1 s), 所以固件会拒绝 r 0。\n"
            "K1 线圈电压请用 spi_probe 固件测 ('w r 0' 吸合 / 'w r 1' 释放, 见首次上电步骤.md)。\n"
            "FW 2.2: HV_enable 为低有效。测试中会两次弹窗, 请用万用表量 J1-12 对 GND: e 1 时约 0 V, e 0 时约 4.5 V。")).pack(anchor="w")
        bf = ttk.Frame(f)
        bf.pack(fill="x", pady=4)
        ttk.Button(bf, text="运行联锁测试", command=self.run_stage6).pack(side="left", padx=2)
        ttk.Button(bf, text="清空", command=lambda: self._clear_tree(self.tree6)).pack(side="left", padx=2)
        self.sum6_var = tk.StringVar(value="")
        ttk.Label(bf, textvariable=self.sum6_var).pack(side="left", padx=12)
        fr, self.tree6 = self._make_tree(f, [("name", "项目"), ("res", "结果"), ("detail", "详情")],
                                         [300, 70, 660], 16)
        fr.pack(fill="both", expand=True)

    def run_stage6(self):
        if not messagebox.askokcancel("联锁测试", "确认 J1 已与 TOF3000 断开?"):
            return
        self._clear_tree(self.tree6)
        emit = self._emit_to(self.tree6)

        def job(stop):
            bs.stage6_interlocks(self.link, emit, self.ask, self.log, stop)

            def summarize():
                kids = [self.tree6.item(i, "values")[1] for i in self.tree6.get_children()]
                self.sum6_var.set("PASS %d / FAIL %d" % (kids.count("PASS"), kids.count("FAIL")))
            self.ui(summarize)
        self.run_job("联锁测试", job)

    # ------------------------------------------------------------- stage 7/8
    def _build_stage78(self):
        f = ttk.Frame(self.nb, padding=6)
        self.nb.add(f, text="阶段7/8 升压运行")
        self.gate_var = tk.IntVar(value=0)
        ttk.Checkbutton(f, variable=self.gate_var, command=self._update_gate, text=(
            "我已完成阶段 3-6, 并确认本次试验的接线状态 (J1/HV 线缆/电极) 符合计划, 现场有人看守")
        ).pack(anchor="w")
        self.ctrl_frame = ttk.Frame(f)
        self.ctrl_frame.pack(fill="x", pady=4)
        cf = self.ctrl_frame

        r1 = ttk.Frame(cf)
        r1.pack(fill="x", pady=2)
        ttk.Label(r1, text="软件限压 (kV):").pack(side="left")
        self.limit_var = tk.StringVar(value="5.0")
        ttk.Entry(r1, textvariable=self.limit_var, width=6).pack(side="left", padx=2)
        ttk.Label(r1, text="30 kV 对应主机单位:").pack(side="left", padx=(10, 1))
        self.units30_var = tk.StringVar(value="59055")
        ttk.Entry(r1, textvariable=self.units30_var, width=8).pack(side="left")
        ttk.Label(r1, text="v_mon kV/V:").pack(side="left", padx=(10, 1))
        self.kvv_var = tk.StringVar(value="3.0")
        ttk.Entry(r1, textvariable=self.kvv_var, width=5).pack(side="left")
        ttk.Label(r1, text="i_mon µA/V:").pack(side="left", padx=(10, 1))
        self.uav_var = tk.StringVar(value="40.0")
        ttk.Entry(r1, textvariable=self.uav_var, width=5).pack(side="left")
        ttk.Label(r1, text="(监视量程请按 TOF3000 手册核对)", foreground="gray").pack(side="left", padx=6)

        r2 = ttk.Frame(cf)
        r2.pack(fill="x", pady=2)
        self.gated = []
        for text, cmd in [("极性 0 (p 0)", lambda: self.quick("p 0")), ("极性 1 (p 1)", lambda: self.quick("p 1")),
                          ("启用 HV (e 1)", self.on_hv_on), ("关闭 HV (e 0)", lambda: self.quick("e 0")),
                          ("积分器开 (r 0)", lambda: self.quick("r 0")), ("积分器关 (r 1)", lambda: self.quick("r 1"))]:
            b = ttk.Button(r2, text=text, command=cmd)
            b.pack(side="left", padx=2)
            self.gated.append(b)

        r3 = ttk.Frame(cf)
        r3.pack(fill="x", pady=2)
        ttk.Label(r3, text="目标 (kV):").pack(side="left")
        self.target_var = tk.StringVar(value="0.0")
        ttk.Entry(r3, textvariable=self.target_var, width=7).pack(side="left", padx=2)
        for text, cmd in [("设定目标", self.on_set_target), ("降到 0", lambda: self.quick("d 0"))]:
            b = ttk.Button(r3, text=text, command=cmd)
            b.pack(side="left", padx=2)
            self.gated.append(b)
        ttk.Label(r3, text="步进 kV:").pack(side="left", padx=(12, 1))
        self.step_var = tk.StringVar(value="0.5")
        ttk.Entry(r3, textvariable=self.step_var, width=5).pack(side="left")
        for text, sign in [("− 步进", -1), ("+ 步进", 1)]:
            b = ttk.Button(r3, text=text, command=lambda s=sign: self.on_step(s))
            b.pack(side="left", padx=2)
            self.gated.append(b)

        r4 = ttk.Frame(cf)
        r4.pack(fill="x", pady=2)
        ttk.Label(r4, text="阶梯爬升 起点 kV:").pack(side="left")
        self.st_start = tk.StringVar(value="1.0")
        ttk.Entry(r4, textvariable=self.st_start, width=5).pack(side="left", padx=2)
        ttk.Label(r4, text="步长:").pack(side="left")
        self.st_step = tk.StringVar(value="1.0")
        ttk.Entry(r4, textvariable=self.st_step, width=5).pack(side="left", padx=2)
        ttk.Label(r4, text="终点:").pack(side="left")
        self.st_end = tk.StringVar(value="5.0")
        ttk.Entry(r4, textvariable=self.st_end, width=5).pack(side="left", padx=2)
        ttk.Label(r4, text="每步保持 s:").pack(side="left")
        self.st_hold = tk.StringVar(value="30")
        ttk.Entry(r4, textvariable=self.st_hold, width=5).pack(side="left", padx=2)
        b = ttk.Button(r4, text="开始阶梯", command=self.on_staircase)
        b.pack(side="left", padx=6)
        self.gated.append(b)
        ttk.Button(r4, text="停止阶梯", command=self.on_stop_job).pack(side="left")
        self.stair_var = tk.StringVar(value="")
        ttk.Label(r4, textvariable=self.stair_var, foreground="#003399").pack(side="left", padx=8)

        # plot
        self.fig = Figure(figsize=(8, 3.6), dpi=90)
        self.ax1 = self.fig.add_subplot(211)
        self.ax2 = self.fig.add_subplot(212, sharex=self.ax1)
        self.fig.subplots_adjust(left=0.08, right=0.98, top=0.95, bottom=0.12, hspace=0.25)
        self.canvas = FigureCanvasTkAgg(self.fig, master=f)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        self._update_gate()

    def _update_gate(self):
        st = "normal" if self.gate_var.get() else "disabled"
        for b in self.gated:
            b.configure(state=st)

    def _units_per_kv(self):
        return float(self.units30_var.get()) / 30.0

    def on_hv_on(self):
        if not messagebox.askokcancel("启用 HV", "即将执行 e 1 (使能 HV)。\n请确认接线与安全措施。继续?"):
            return
        self.quick("e 1")

    def on_set_target(self):
        try:
            kv = float(self.target_var.get())
            limit = float(self.limit_var.get())
            units = int(round(kv * self._units_per_kv()))
        except ValueError:
            messagebox.showerror("输入错误", "目标/限压/换算系数必须是数字")
            return
        if kv < 0:
            messagebox.showerror("输入错误", "目标不能为负 (极性用 p 命令切换)")
            return
        if kv > limit + 1e-9:
            messagebox.showerror("超过软件限压", "目标 %.2f kV 超过软件限压 %.2f kV, 先在上面提高限压" % (kv, limit))
            return
        self.quick("d %d" % units)

    def on_step(self, sign):
        try:
            kv = float(self.target_var.get()) + sign * float(self.step_var.get())
        except ValueError:
            return
        self.target_var.set("%.3f" % max(0.0, kv))
        self.on_set_target()

    def on_staircase(self):
        try:
            a, s, b = float(self.st_start.get()), float(self.st_step.get()), float(self.st_end.get())
            hold = float(self.st_hold.get())
            limit = float(self.limit_var.get())
            upk = self._units_per_kv()
        except ValueError:
            messagebox.showerror("输入错误", "阶梯参数必须是数字")
            return
        if s <= 0 or b < a or hold < 1:
            messagebox.showerror("输入错误", "需要 步长>0, 终点>=起点, 保持>=1 s")
            return
        if b > limit + 1e-9:
            messagebox.showerror("超过软件限压", "终点 %.2f kV 超过软件限压 %.2f kV" % (b, limit))
            return
        targets_kv = []
        v = a
        while v <= b + 1e-9:
            targets_kv.append(v)
            v += s
        if not messagebox.askokcancel("阶梯爬升", "将依次到: %s kV, 每步保持 %.0f s。\n"
                                      "HV 必须已启用 (e 1)。出现故障自动降压。继续?" % (
                                          ", ".join("%.2f" % t for t in targets_kv), hold)):
            return
        units = [int(round(t * upk)) for t in targets_kv]

        def job(stop):
            bs.run_staircase(self.link, units, hold, self.log, stop,
                             on_step=lambda i, t: self.ui(lambda: self.stair_var.set(
                                 "台阶 %d/%d: %.2f kV" % (i + 1, len(units), targets_kv[i]))))
            self.ui(lambda: self.stair_var.set(""))
        self.run_job("阶梯爬升", job)

    def plot_tick(self):
        try:
            if self.history and self.nb.index(self.nb.select()) == 4:
                self._redraw()
        finally:
            self.root.after(1000, self.plot_tick)

    def _redraw(self):
        h = list(self.history)
        t = [x[0] for x in h]
        try:
            kvv = float(self.kvv_var.get())
            uav = float(self.uav_var.get())
            upk = self._units_per_kv()
        except ValueError:
            return
        self.ax1.clear()
        self.ax2.clear()
        self.ax1.plot(t, [x[1] * kvv for x in h], label="v_mon (kV)", color="tab:blue")
        self.ax1.plot(t, [x[3] / upk for x in h], label="目标 (kV)", color="tab:gray", linestyle="--")
        self.ax1.plot(t, [x[4] / upk for x in h], label="DAC 当前 (kV)", color="tab:green", linestyle=":")
        self.ax1.set_ylabel("kV")
        self.ax1.legend(loc="upper left", fontsize=7)
        self.ax2.plot(t, [x[2] * uav for x in h], color="tab:red", label="i_mon (µA)")
        self.ax2.set_ylabel("µA")
        self.ax2.set_xlabel("t (s)")
        prev = None
        for x in h:
            if prev is not None and x[5] > prev:
                self.ax1.axvline(x[0], color="orange", alpha=0.6)
                self.ax2.axvline(x[0], color="orange", alpha=0.6)
            prev = x[5]
        self.canvas.draw_idle()

    # ----------------------------------------------------------------- config
    # ================================================================ X-ray guard
    def _build_xray(self):
        f = ttk.Frame(self.nb, padding=6)
        self.nb.add(f, text="X射线监测")
        self.xguard = None
        self.xsource = None
        self.sim_dose = 0.1
        ttk.Label(f, justify="left", text=(
            "剂量计 (RadEye G20 或 GMC-800 盖革计数器) 由本程序读取, 约每秒把读数发给固件 ('x N', N 单位 0.01 µSv/h)。\n"
            "固件判断: 读数 ≥ 阈值 → 目标降到当前输出以下 xback 单位(最短间隔 3 s, 持续超标则继续降); "
            "告警期间 d 只能降不能升 (ERR XRAY); 读数 < 阈值的 80 % 才解除。\n"
            "监测丢失: 读数停止到达超过 xtimeout 秒 (HV 开且 DAC>0) → 自动降压并锁故障 0x40。所以 RadEye 掉线或本程序崩溃都会降压。\n"
            "本底约 0.1 µSv/h; 旧数据里 X 射线出现时读数升到 0.4–0.7 µSv/h (两路电源同时在约 16–19 kV 以上)。\n"
            "GMC-800 只给计数: 本程序取最近 10 s 的每秒计数平均, 再按设备自带的标定换算 (本底约 0.1 µSv/h, 单次波动约 0.06, 一次计数约 0.04 µSv/h); 对短脉冲的反应比 RadEye 慢约半个窗口, 阈值要重新确认。\n"
            "串口填 SIM 可用模拟读数 (台架调试用, 不接剂量计)。")).pack(anchor="w")

        r1 = ttk.Frame(f)
        r1.pack(fill="x", pady=4)
        ttk.Label(r1, text="剂量计:").pack(side="left")
        self.xsrc_var = tk.StringVar(value="RadEye G20")
        ttk.Combobox(r1, textvariable=self.xsrc_var, values=("RadEye G20", "GMC-800"), width=11, state="readonly").pack(side="left", padx=4)
        ttk.Label(r1, text="串口:").pack(side="left")
        self.xport_var = tk.StringVar(value="COM8")
        ttk.Entry(r1, textvariable=self.xport_var, width=10).pack(side="left", padx=4)
        ttk.Button(r1, text="连接并开始转发", command=self.x_connect).pack(side="left", padx=2)
        ttk.Button(r1, text="断开", command=self.x_disconnect).pack(side="left", padx=2)
        ttk.Label(r1, text="  模拟读数 µSv/h:").pack(side="left")
        self.xsim_var = tk.StringVar(value="0.10")
        ttk.Entry(r1, textvariable=self.xsim_var, width=7).pack(side="left", padx=2)
        ttk.Button(r1, text="设置", command=self.x_set_sim).pack(side="left")

        r2 = ttk.Frame(f)
        r2.pack(fill="x", pady=4)
        self.xlive = tk.StringVar(value="未连接")
        ttk.Label(r2, textvariable=self.xlive, font=("Microsoft YaHei UI", 11, "bold"),
                  foreground="#003399").pack(side="left")

        r3 = ttk.LabelFrame(f, text="写入固件 (点'应用'发送 c xthr / xback / xtimeout; 要保留请再点'保存到 EEPROM')", padding=6)
        r3.pack(fill="x", pady=6)
        self.xthr_var = tk.StringVar(value="0.40")
        self.xback_var = tk.StringVar(value="3936")
        self.xto_var = tk.StringVar(value="5")
        for i, (label, var, w, note) in enumerate([
                ("告警阈值 µSv/h", self.xthr_var, 8, "写入 xthr = 值×100; 0 = 关闭 X 射线保护"),
                ("每步回退(主机单位)", self.xback_var, 8, "3936 ≈ 2 kV; 0 = 降到 0 并锁故障 0x20"),
                ("监测丢失超时 s", self.xto_var, 8, "0 = 关闭; 建议 5")]):
            ttk.Label(r3, text=label + ":").grid(row=i, column=0, sticky="e", padx=4, pady=1)
            ttk.Entry(r3, textvariable=var, width=w).grid(row=i, column=1, sticky="w")
            ttk.Label(r3, text=note, foreground="gray").grid(row=i, column=2, sticky="w", padx=8)
        bf = ttk.Frame(r3)
        bf.grid(row=3, column=0, columnspan=3, sticky="w", pady=4)
        ttk.Button(bf, text="应用", command=self.x_apply).pack(side="left", padx=2)
        ttk.Button(bf, text="保存到 EEPROM (c save)", command=lambda: self.quick("c save")).pack(side="left", padx=2)
        self.root.after(500, self._xray_tick)

    def x_set_sim(self):
        try:
            self.sim_dose = float(self.xsim_var.get())
            self.log("模拟 X 射线读数 = %.2f µSv/h" % self.sim_dose)
        except ValueError:
            messagebox.showerror("模拟读数", "请输入数字")

    def x_connect(self):
        if self.xguard is not None:
            messagebox.showinfo("X射线监测", "已在运行, 先断开")
            return
        if not self.link.is_open:
            messagebox.showwarning("未连接", "请先连接 HV_control 串口")
            return
        port = self.xport_var.get().strip()
        gmc = self.xsrc_var.get().startswith("GMC")
        try:
            thr = float(self.xthr_var.get())
        except ValueError:
            thr = None

        def worker():
            try:
                if port.upper() == "SIM":
                    app = self

                    class Sim(object):
                        def read_dose(self):
                            time.sleep(0.05)
                            return app.sim_dose
                    src = Sim()
                else:
                    import xray_guard
                    if gmc:
                        src = xray_guard.Gmc800Source(port)
                        self.log("GMC-800: %s" % src.setup())
                    else:
                        src = xray_guard.RadEyeG20(port)
                        src.setup()
                import xray_guard as xg
                g = xg.XrayGuard(src, self.link, period=1.0, log=self.log,
                                 on_alarm=lambda d: self.log("!! X 射线读数 %.2f µSv/h 超过阈值" % d),
                                 xthr_uSv=thr)
                g.start()
                self.xsource, self.xguard = src, g
                self.log("X 射线监测已启动 (%s)" % port)
            except Exception as e:
                self.log("!! X 射线监测启动失败: %r" % (e,))
        threading.Thread(target=worker, daemon=True).start()

    def x_disconnect(self):
        g, src = self.xguard, self.xsource
        self.xguard = self.xsource = None
        if g is not None:
            g.stop()
        if src is not None and hasattr(src, "close"):
            src.close()
        if g is not None:
            self.log("X 射线监测已断开 (固件在 xtimeout 后会因监测丢失而降压)")

    def x_apply(self):
        try:
            thr = float(self.xthr_var.get())
            back = int(float(self.xback_var.get()))
            tmo = int(float(self.xto_var.get()))
        except ValueError:
            messagebox.showerror("X射线", "请输入数字")
            return
        if thr < 0 or thr > 655 or back < 0 or back > 65535 or tmo < 0 or tmo > 65535:
            messagebox.showerror("X射线", "数值超范围 (阈值 0–655 µSv/h)")
            return
        if self.xguard is not None:
            self.xguard.xthr_uSv = thr if thr > 0 else None

        def job():
            for k, v in (("xthr", int(round(thr * 100))), ("xback", back), ("xtimeout", tmo)):
                reply = self.link.query("c %s %d" % (k, v), quiet=False)
                if reply != "OK":
                    self.log("设置 %s=%d 失败: %s" % (k, v, reply))
            self.cfg_read()
        self.quick_fn(job)

    def _xray_tick(self):
        g = self.xguard
        if g is None:
            self.xlive.set("X 射线监测: 未运行")
        else:
            age = time.time() - g.t_dose if g.t_dose else None
            if g.last_error:
                self.xlive.set("X 射线: 读数异常 — %s  (成功 %d / 失败 %d)" % (g.last_error, g.n_ok, g.n_bad))
            elif g.dose is None:
                self.xlive.set("X 射线: 等待第一个读数…")
            else:
                self.xlive.set("X 射线: %.2f µSv/h%s   (成功 %d / 失败 %d, %.1f s 前)" % (
                    g.dose, "  ⚠ 超过阈值" if g.alarm else "", g.n_ok, g.n_bad, age))
        self.root.after(500, self._xray_tick)

    def _build_config(self):
        f = ttk.Frame(self.nb, padding=6)
        self.nb.add(f, text="配置")
        ttk.Label(f, justify="left", text=(
            "读取/修改固件配置。'应用'只发送与上次读取值不同的项; 改动在板子复位后丢失, 点'保存到 EEPROM'才永久。\n"
            "board/scale/dacmax 仅在 HV 关且 DAC=0 时可改。")).pack(anchor="w")
        bf = ttk.Frame(f)
        bf.pack(fill="x", pady=4)
        ttk.Button(bf, text="读取 (c)", command=self.cfg_read).pack(side="left", padx=2)
        ttk.Button(bf, text="应用", command=self.cfg_apply).pack(side="left", padx=2)
        ttk.Button(bf, text="保存到 EEPROM (c save)", command=lambda: self.quick("c save")).pack(side="left", padx=2)
        ttk.Button(bf, text="恢复默认 (c defaults)", command=self.cfg_defaults).pack(side="left", padx=2)
        grid = ttk.Frame(f)
        grid.pack(fill="x", pady=6)
        self.cfg_vars = {}
        for i, (key, label, note) in enumerate(CONFIG_KEYS):
            ttk.Label(grid, text=label + ":").grid(row=i, column=0, sticky="e", padx=4, pady=1)
            var = tk.StringVar()
            self.cfg_vars[key] = var
            ttk.Entry(grid, textvariable=var, width=12).grid(row=i, column=1, sticky="w")
            ttk.Label(grid, text=note, foreground="gray").grid(row=i, column=2, sticky="w", padx=8)

    # ------------------------------------------------------- stage 7 tests (<= 1 kV)
    def _build_stage7tests(self):
        f = ttk.Frame(self.nb, padding=6)
        self.tab7 = f
        self.nb.add(f, text="Stage 7 tests (<1 kV)")
        ttk.Label(f, justify="left", text=(
            "Automatic stage 7 sequence with the TOF3000 connected, never above 0.8 kV "
            "(GUI cap and firmware dacmax 0.9 kV, abort if v_mon > 0.95 kV):\n"
            "0 preparation  1 HV_enable shutdown (1b)  2 firmware cap  3 staircase 0.2-0.8 kV  "
            "4 X-ray interlock (GMC-800 background + simulated alarm)  5 24 V outer interlock at HV.\n"
            "The red E-stop (z) and 'stop current job' work at any time. Live values: status strip above, "
            "plot on the stage 7/8 tab.")).pack(anchor="w")
        bf = ttk.Frame(f)
        bf.pack(fill="x", pady=4)
        ttk.Button(bf, text="Run stage 7 tests", command=self.run_stage7_tests).pack(side="left", padx=2)
        ttk.Button(bf, text="Stop", command=self.on_stop_job).pack(side="left", padx=2)
        ttk.Label(bf, text="  X-ray meter port:").pack(side="left")
        self.s7_xport_var = tk.StringVar(value=self.opts.get("xport") or "COM5")
        ttk.Entry(bf, textvariable=self.s7_xport_var, width=8).pack(side="left", padx=2)
        self.s7_step_var = tk.StringVar(value="not running")
        tk.Label(f, textvariable=self.s7_step_var, anchor="w", fg="#003399",
                 font=("Microsoft YaHei UI", 12, "bold")).pack(fill="x", pady=4)
        tf, self.tree7 = self._make_tree(f, [("name", "Test"), ("res", "Result"), ("detail", "Detail")],
                                         [430, 60, 640], height=18)
        tf.pack(fill="both", expand=True)

    def run_stage7_tests(self):
        if self.job_thread is not None and self.job_thread.is_alive():
            messagebox.showwarning("Stage 7 tests", "Another job is running")
            return
        if self.wd is None:
            messagebox.showwarning("Stage 7 tests", "Connect the HV_control board first")
            return
        if not messagebox.askokcancel("Stage 7 tests", (
                "Run the stage 7 test sequence now?\n\n"
                "- TOF3000 connected, output in the closed insulated box, nobody touching HV parts\n"
                "- max 0.8 kV; hard caps 0.9 kV (GUI + firmware); abort above 0.95 kV\n"
                "- the 24 V supply will be switched off and on once (outer interlock test)\n"
                "- the watchdog window will be killed and restarted once\n"
                "- X-ray meter on %s (real background), alarms simulated\n\nE-stop (z) stays available."
                % self.s7_xport_var.get().strip()), parent=self.root):
            return
        cap = int(0.9 * s7.UNITS_PER_KV)
        self.hv_cap_units = cap if self.hv_cap_units is None else min(self.hv_cap_units, cap)
        self.limit_var.set("%.2f" % (self.hv_cap_units / s7.UNITS_PER_KV))
        self.log("session HV cap: %d units (%.2f kV)" % (self.hv_cap_units, self.hv_cap_units / s7.UNITS_PER_KV))
        self._clear_tree(self.tree7)
        self.nb.select(self.tab7)
        ctx = GuiCtx(self)
        xport = self.s7_xport_var.get().strip()
        sim = s7.SimDose()

        def job(stop):
            path, aborted = s7.run(ctx, sim, LOG_DIR, xport=xport)
            self.log("stage 7 tests %s; report %s" % ("ABORTED: " + aborted if aborted else "finished", path))
        self.run_job("Stage 7 tests", job)

    def _autorun(self, tries=[0]):
        """--port / --autorun: connect, wait for the armed watchdog, then start the stage 7 tests."""
        port = self.opts.get("port")
        if tries[0] == 0 and port:
            match = [p for p in self.port_box["values"] if p.split()[0] == port]
            if match:
                self.port_var.set(match[0])
                self.on_connect()
            else:
                self.log("!! --port %s not found" % port)
                return
        tries[0] += 1
        ready = self.link.is_open and self.wd is not None and self.wd.armed and self.wd.reply is not None
        if self.opts.get("autorun") and ready:
            self.root.after(500, self.run_stage7_tests)
            return
        if tries[0] < 60:
            self.root.after(500, self._autorun)
        elif self.opts.get("autorun"):
            self.log("!! autorun: board / watchdog not ready after 30 s")

    def cfg_read(self):
        def job():
            cfg = self.link.config(quiet=False)
            self.cfg_cache = cfg
            self.ui(lambda: [self.cfg_vars[k].set(str(cfg.get(k, ""))) for k, _, _ in CONFIG_KEYS])
        self.quick_fn(job)

    def cfg_apply(self):
        wanted = dict((k, self.cfg_vars[k].get().strip()) for k, _, _ in CONFIG_KEYS)
        cache = dict(self.cfg_cache)

        def job():
            changed = 0
            for key, _, _ in CONFIG_KEYS:
                new = wanted[key]
                old = str(cache.get(key, ""))
                if new == "" or new == old:
                    continue
                try:
                    if float(new) == float(old):
                        continue
                except ValueError:
                    pass
                reply = self.link.query("c %s %s" % (key, new), quiet=False)
                changed += 1
                if reply != "OK":
                    self.log("设置 %s=%s 失败: %s" % (key, new, reply))
            self.log("已发送 %d 项改动" % changed)
            self.cfg_read()
        self.quick_fn(job)

    def cfg_defaults(self):
        if messagebox.askokcancel("恢复默认", "执行 c defaults (只改 RAM, 不保存)。继续?"):
            self.quick("c defaults")
            self.root.after(500, self.cfg_read)

    # ============================================================ job control
    def run_job(self, name, fn):
        if not self.link.is_open:
            messagebox.showwarning("未连接", "请先连接串口")
            return
        if self.job_thread is not None and self.job_thread.is_alive():
            messagebox.showwarning("任务进行中", "已有任务在运行, 请等待或点'停止当前任务'")
            return
        self.job_stop.clear()

        def target():
            self.log("== 开始: %s" % name)
            try:
                fn(self.job_stop)
            except bs.Aborted:
                self.log("== 已中止: %s" % name)
            except HVError as e:
                self.log("!! %s 出错: %s" % (name, e))
            except Exception as e:
                self.log("!! %s 异常: %r" % (name, e))
            else:
                self.log("== 完成: %s" % name)
        self.job_thread = threading.Thread(target=target, daemon=True)
        self.job_thread.start()

    def quick(self, cmd):
        """Send one command in a short thread and log the reply."""
        def job():
            reply = self.link.query(cmd, quiet=False)
            if reply.startswith("ERR"):
                self.ui(lambda: self.conn_label.config(text="命令被拒: %s -> %s" % (cmd, reply), foreground="#cc6600"))
        self.quick_fn(job)

    def quick_fn(self, fn):
        if not self.link.is_open:
            messagebox.showwarning("未连接", "请先连接串口")
            return

        def target():
            try:
                fn()
            except HVError as e:
                self.log("!! %s" % e)
            except Exception as e:
                self.log("!! 异常: %r" % (e,))
        threading.Thread(target=target, daemon=True).start()

    def on_stop_job(self):
        self.job_stop.set()
        self.log("请求停止当前任务")

    def on_estop(self):
        self.job_stop.set()

        def target():
            try:
                reply = self.link.emergency_stop()
                self.log("!! 急停 z -> %s" % reply)
            except HVError as e:
                self.log("!! 急停失败: %s  (请直接断开 HV/24 V!)" % e)
        threading.Thread(target=target, daemon=True).start()

    # ============================================= 24 V supply / outer interlock
    def _build_psu_strip(self):
        pf = ttk.LabelFrame(self.root, padding=4, text=(
            "24 V supply (SPD3303X CH1 -> PCB + TOF3000) - outer interlock: psu_watchdog.py cuts CH1 "
            "when this program loses control of the Arduino"))
        pf.pack(fill="x", padx=4, pady=(2, 0))
        self.psu_state_var = tk.StringVar(value="Watchdog: not checked yet")
        self.psu_state_label = tk.Label(pf, textvariable=self.psu_state_var, width=26, anchor="w",
                                        font=("Microsoft YaHei UI", 10, "bold"), fg="gray")
        self.psu_state_label.pack(side="left")
        self.psu_meas_var = tk.StringVar(value="CH1: -")
        ttk.Label(pf, textvariable=self.psu_meas_var, width=36, foreground="#003399").pack(side="left", padx=4)
        for text, fn in (("Start watchdog", self.wd_start), ("Arm", self.wd_arm),
                         ("24 V ON", self.wd_power_on), ("24 V OFF", self.wd_power_off),
                         ("Reset interlock", self.wd_reset)):
            ttk.Button(pf, text=text, command=fn).pack(side="left", padx=2)
        self.wd_require_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(pf, text="HV (e 1 / d>0) only while armed", variable=self.wd_require_var,
                        command=self.wd_require_changed).pack(side="left", padx=6)
        self.remote_var = tk.BooleanVar(value=bool(self.opts.get("remote")))
        ttk.Checkbutton(pf, text="Local remote control (127.0.0.1:%d)" % REMOTE_ADDR[1], variable=self.remote_var,
                        command=self.remote_changed).pack(side="left", padx=6)
        self.psu_reason_var = tk.StringVar(value="")
        ttk.Label(pf, textvariable=self.psu_reason_var, foreground="#cc0000").pack(side="left", padx=4)
        self.root.after(500, self._psu_tick)
        self.remote_on = bool(self.opts.get("remote"))
        threading.Thread(target=self._remote_server, daemon=True).start()

    # --------------------------------------------------------- local remote control
    def remote_changed(self):
        self.remote_on = bool(self.remote_var.get())
        self.log("local remote control %s" % ("ENABLED" if self.remote_on else "disabled"))

    def _remote_allowed(self, cmd):
        """Whitelist for remote commands; None if allowed, else the reason."""
        parts = cmd.split()
        if not parts:
            return "empty command"
        op = parts[0]
        if op in ("?", "s", "b", "a", "D", "E", "P", "m", "C", "z", "k") and len(parts) == 1:
            return None
        if op == "c" and len(parts) == 1:
            return None
        if op == "e" and len(parts) == 2 and parts[1] in ("0", "1"):
            return None
        if op == "d" and len(parts) == 2:
            return None                    # the session cap + watchdog permit still apply in HVLink
        if op == "c" and len(parts) == 3 and parts[1] == "cable" and parts[2] in ("0", "1"):
            return None
        if op == "c" and len(parts) == 3 and parts[1] == "dacmax":
            try:
                n = int(parts[2])
            except ValueError:
                return "bad dacmax"
            if self.hv_cap_units is not None and n > self.hv_cap_units:
                return "dacmax %d above the session cap %d" % (n, self.hv_cap_units)
            return None
        return "command not allowed remotely"

    def _remote_handle(self, msg):
        if not self.remote_on:
            return {"ok": False, "error": "remote control is disabled in the GUI"}
        cmd = str(msg.get("cmd", "")).strip()
        if cmd == "#status":
            reply, event = self.wd_last
            out = {"ok": True, "connected": self.link.is_open, "watchdog": reply, "wd_event": event,
                   "cap_units": self.hv_cap_units, "job_running": bool(self.job_thread and self.job_thread.is_alive())}
            if self.link.is_open:
                try:
                    out.update(status=self.link.status(), volts=self.link.volts(), D=self.link.query("D", quiet=True))
                except HVError as e:
                    out.update(ok=False, error=str(e))
            return out
        why = self._remote_allowed(cmd)
        if why:
            self.log("[remote] refused %r: %s" % (cmd, why))
            return {"ok": False, "error": why}
        if not self.link.is_open:
            return {"ok": False, "error": "not connected"}
        self.log("[remote] %s" % cmd)
        try:
            reply = self.link.query(cmd, quiet=False)
        except HVError as e:
            return {"ok": False, "error": str(e)}
        return {"ok": True, "reply": reply}

    def _remote_server(self):
        try:
            sock = pwd.bind_socket(REMOTE_ADDR)
        except OSError as e:
            self.log("!! remote control: cannot listen on %s:%d (%s)" % (REMOTE_ADDR[0], REMOTE_ADDR[1], e))
            return
        while True:
            try:
                data, peer = sock.recvfrom(4096)
            except OSError:
                continue
            try:
                msg = json.loads(data.decode("utf-8"))
                out = self._remote_handle(msg if isinstance(msg, dict) else {})
            except Exception as e:  # noqa: BLE001
                out = {"ok": False, "error": repr(e)}
            try:
                sock.sendto(json.dumps(out).encode("utf-8"), peer)
            except OSError:
                pass

    def _hv_permit(self, cmd):
        """HVLink.permit: called inside link.query (worker threads); cached state only."""
        parts = cmd.split()
        if self.hv_cap_units is not None and len(parts) == 2 and parts[0] == "d":
            try:
                n = float(parts[1])
            except ValueError:
                n = 0
            if n > self.hv_cap_units:
                return "d %s is above the session HV cap (%d units = %.2f kV)" % (
                    parts[1], self.hv_cap_units, self.hv_cap_units / s7.UNITS_PER_KV)
        if not self.wd_require:
            return None
        if self.wd is None:
            return "supply watchdog client not running (connect first)"
        return self.wd.permit(cmd)

    def wd_require_changed(self):
        want = bool(self.wd_require_var.get())
        if not want and not messagebox.askokcancel(
                "Outer interlock",
                "Allow e 1 / d>0 WITHOUT an armed supply watchdog?\n"
                "Only for bench work without the TOF3000 (e.g. stage 6 with J1 disconnected)."):
            self.wd_require_var.set(True)
            return
        self.wd_require = want
        self.log("HV permit from the supply watchdog: %s" % ("REQUIRED" if want else "NOT required (bench mode)"))

    def _wd_update(self, reply, event):
        """WatchdogClient callback (client thread)."""
        self.wd_last = (reply, event)
        if event and event != self.wd_event_shown:
            self.wd_event_shown = event
            self.log("!!! OUTER INTERLOCK: %s" % event)

            def show(msg=event):
                messagebox.showwarning("Outer interlock", "%s\n\nThe Arduino was sent z (if it answers).\n"
                                       "Check the cause, then 'Reset interlock' and '24 V ON'." % msg,
                                       parent=self.root)
            self.ui(lambda: self.root.after(0, show))
        elif not event:
            self.wd_event_shown = ""

    def _psu_tick(self):
        reply, event = self.wd_last
        if self.wd is None and not self.wd_status_busy:
            # no client yet (Arduino not connected): ask the watchdog directly, off the UI thread
            self.wd_status_busy = True

            def worker():
                try:
                    r = pwd.request({"type": "status"}, timeout=0.5)
                except pwd.WatchdogError:
                    r = None
                if self.wd is None:
                    self.wd_last = (r, "")
                self.wd_status_busy = False
            threading.Thread(target=worker, daemon=True).start()
        stale = self.wd is not None and time.monotonic() - self.wd.t_reply > 2.0
        if reply is None or stale:
            self.psu_state_var.set("Watchdog: NOT RUNNING")
            self.psu_state_label.config(fg="#cc0000" if self.wd is not None else "gray")
            self.psu_meas_var.set("CH1: -")
        else:
            st = reply.get("state")
            if self.wd is not None and not self.wd.armed and st != pwd.TRIPPED:
                st_txt, color = "%s (this GUI not armed)" % st, "#cc6600"
            else:
                st_txt, color = st, {pwd.ARMED: "#007700", pwd.TRIPPED: "#cc0000"}.get(st, "#cc6600")
            if not reply.get("psu_ok"):
                st_txt, color = st_txt + ", NO SPD3303X", "#cc0000"
            self.psu_state_var.set("Watchdog: " + st_txt)
            self.psu_state_label.config(fg=color)
            m = reply.get("meas") or {}
            if m:
                self.psu_meas_var.set("%s: %s  %.3f V  %.3f A%s   set %s V / %s A" % (
                    reply.get("ch"), "ON " if m.get("on") else "OFF", m.get("v") or 0.0, m.get("i") or 0.0,
                    "  CC!" if m.get("cc") else "", reply.get("v_set"), reply.get("i_set")))
        why = event or (reply or {}).get("reason", "") or (reply or {}).get("error", "")
        self.psu_reason_var.set(why[:160])
        self.root.after(500, self._psu_tick)

    def wd_start(self):
        def worker():
            if pwd.is_running():
                self.log("psu_watchdog is already running")
                return
            self._wd_spawn()
        threading.Thread(target=worker, daemon=True).start()

    def _wd_spawn(self):
        """Start psu_watchdog.py in its own console window (it must outlive this GUI)."""
        path = os.path.join(HERE, "psu_watchdog.py")
        flags = getattr(subprocess, "CREATE_NEW_CONSOLE", 0)
        try:
            subprocess.Popen([sys.executable, "-X", "utf8", path], cwd=HERE, creationflags=flags)
        except OSError as e:
            self.log("!! could not start psu_watchdog.py: %s" % e)
            return False
        self.log("started psu_watchdog.py (own window; it keeps running when this GUI closes)")
        for _ in range(50):
            if pwd.is_running():
                return True
            time.sleep(0.1)
        self.log("!! psu_watchdog.py did not answer within 5 s (see its window)")
        return False

    def _wd_attach(self, banner):
        """After a successful connect: start (or reuse) the client and arm it."""
        name = banner.split("Name ")[1].split(".")[0].strip() if "Name " in banner else ""
        if not name or name == "-":
            name = self.link.port or "HV_control"
        if not pwd.is_running():
            self.log("psu_watchdog not running -> starting it")
            self._wd_spawn()
        if self.wd is None or not self.wd.is_alive() or self.wd.name != name:
            if self.wd is not None:
                self.wd.stop()
            self.wd = pwd.WatchdogClient(self.link, name, log=self.log, on_update=self._wd_update, board=name)
            self.wd.start()
        self.wd_arm()

    def _wd_detach(self):
        """Clean release before the port is closed: the watchdog trips if HV is not off."""
        wd, self.wd = self.wd, None
        if wd is None:
            return
        try:
            r = wd.disarm()
            self.log("supply watchdog: disarmed (state %s%s)" % (
                r.get("state"), (", " + r.get("reason")) if r.get("reason") else ""))
        except pwd.WatchdogError as e:
            self.log("!! supply watchdog disarm: %s" % e)
        wd.stop()

    def _wd_call(self, what, fn):
        def worker():
            try:
                r = fn()
            except pwd.WatchdogError as e:
                self.log("!! supply watchdog %s: %s" % (what, e))
                return
            if r.get("ok") and not r.get("error"):
                self.log("supply watchdog %s: OK (state %s)" % (what, r.get("state")))
            else:
                self.log("supply watchdog %s: %s" % (what, r.get("error") or r.get("reason")))
        threading.Thread(target=worker, daemon=True).start()

    def wd_arm(self):
        if self.wd is None:
            messagebox.showwarning("Outer interlock", "Connect the HV_control board first")
            return
        self._wd_call("arm", self.wd.arm)

    def wd_reset(self):
        if self.wd is None:
            messagebox.showwarning("Outer interlock", "Connect the HV_control board first")
            return
        self._wd_call("reset", self.wd.reset)

    def wd_power_on(self):
        if self.wd is None:
            messagebox.showwarning("Outer interlock", "Connect the HV_control board first (24 V ON needs an armed watchdog)")
            return
        if not messagebox.askokcancel("24 V ON", "Switch the SPD3303X CH1 output ON (24 V to the PCB and TOF3000)?\n"
                                      "Needs: watchdog ARMED, Arduino answering, HV off, DAC 0."):
            return
        self._wd_call("power_on", self.wd.power_on)

    def wd_power_off(self):
        if not messagebox.askokcancel("24 V OFF", "Switch the SPD3303X CH1 output OFF (PCB and TOF3000 lose 24 V)?"):
            return
        if self.wd is not None:
            self._wd_call("power_off", self.wd.power_off)
        else:
            self._wd_call("power_off", lambda: pwd.request({"type": "power_off", "client": "gui"}))

    # ======================================================== connection / poll
    def refresh_ports(self):
        ports = ["%s  %s" % (d, desc) for d, desc in HVLink.list_ports()]
        self.port_box["values"] = ports
        pref = [p for p in ports if "Arduino" in p or "Nano" in p]
        if pref:
            self.port_var.set(pref[0])
        elif ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def set_connected(self, ok, text=None):
        self.btn_connect.config(text="断开" if ok else "连接", state="normal")
        self.conn_label.config(text=text or ("已连接" if ok else "未连接"),
                               foreground="#007700" if ok else "gray")

    def on_connect(self):
        if self.link.is_open:
            self.user_disc = True
            self.poll_stop.set()
            self.btn_connect.config(state="disabled")

            def closer():
                self._wd_detach()          # before closing: the watchdog trips if HV is not off
                self.link.close()
                self.log("已断开")
                self.ui(lambda: self.set_connected(False))
            threading.Thread(target=closer, daemon=True).start()
            return
        port = self.port_var.get().split()[0] if self.port_var.get() else ""
        if not port:
            messagebox.showwarning("串口", "请选择串口")
            return
        self.btn_connect.config(state="disabled", text="连接中...")
        self.conn_label.config(text="打开串口并等待 Arduino 复位 (约 2 s)...", foreground="#cc6600")

        def worker():
            try:
                banner = self.link.open(port)
            except Exception as e:
                self.log("连接失败: %s" % e)
                self.ui(lambda: self.set_connected(False, "连接失败"))
                return
            self.user_disc = False
            self.poll_stop.clear()
            self.t0 = time.time()
            self.last_flt = 0
            self.last_ev = 0
            self.ui(self.history.clear)
            self.ui(lambda: self.set_connected(True))
            self.poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
            self.poll_thread.start()
            try:
                self._wd_attach(banner)
            except Exception as e:  # never let the interlock setup kill the connect
                self.log("!! supply watchdog setup failed: %r" % (e,))
        threading.Thread(target=worker, daemon=True).start()

    def _poll_loop(self):
        prev_up = None
        prev_gen = self.link.generation
        cycle = 0
        err_count = 0
        silent = False
        while not self.poll_stop.is_set():
            try:
                if self.link.generation != prev_gen:
                    prev_gen = self.link.generation
                    prev_up = None
                st = self.link.status()
                v = self.link.volts()
                d = self.link.dstate()
                cycle += 1
                m = None
                if cycle % 5 == 0:
                    m = self.link.window()
                if silent:
                    self.log("Arduino 重新响应")
                    silent = False
                err_count = 0
                if prev_up is not None and st.get("up", 0) + 1 < prev_up:
                    self.log("!! 检测到 Arduino 复位: rst=0x%X (%s) n_por=%s n_bor=%s n_ext=%s n_wdt=%s" % (
                        st.get("rst", 0), decode_bits(st.get("rst", 0), RST_BITS), st.get("n_por"),
                        st.get("n_bor"), st.get("n_ext"), st.get("n_wdt")))
                prev_up = st.get("up", 0)
                now = time.time() - self.t0
                self.ui(lambda s=st, vv=v, dd=d, tt=now: self.on_data(s, vv, dd, tt))
            except HVError as e:
                if self.poll_stop.is_set() or self.user_disc:
                    break
                if not self.link.is_open:
                    self.log("!! 串口已断开 (USB 链路丢失?): %s" % e)
                    self.ui(lambda: self.set_connected(False, "串口丢失!"))
                    break
                err_count += 1
                if err_count == 3 and not silent:
                    silent = True
                    self.log("!! Arduino 连续 3 次无回复 (固件挂死?)")
                    self.ui(lambda: self.conn_label.config(text="无回复!", foreground="#cc0000"))
            self.poll_stop.wait(0.5)

    def on_data(self, st, v, d, now):
        self.history.append((now, v[0], v[1], d[0], d[1], st.get("ev", 0)))
        for k in ("up", "hv", "pol", "rly", "int", "tgt", "cur", "raw", "ev", "sp", "backoffs", "dacfix",
                  "xr", "xh", "xtrips"):
            self.sv[k].set(str(st.get(k, "-")))
        sp = st.get("sp", 0)
        if sp != getattr(self, "last_sp", 0):
            if sp > getattr(self, "last_sp", 0):
                self.log("打火事件(高阈值): 总计 %d" % sp)
            self.last_sp = sp
        xt = st.get("xtrips", 0)
        if xt != getattr(self, "last_xtrips", 0):
            if xt > getattr(self, "last_xtrips", 0):
                self.log("!! X 射线告警回退: 第 %d 次, 读数=%s×0.01 µSv/h, 目标=%s" % (xt, st.get("xr"), st.get("tgt")))
            self.last_xtrips = xt
        rst = st.get("rst", 0)
        self.sv["rst"].set("0x%X" % rst)
        flt = st.get("flt", 0)
        if flt:
            self.sv["flt"].set("0x%X  %s" % (flt, decode_bits(flt, FLT_BITS)))
            self.flt_label.configure(foreground="#cc0000")
        else:
            self.sv["flt"].set("无")
            self.flt_label.configure(foreground="#006600")
        if flt != self.last_flt:
            self.log("故障变化: 0x%X -> 0x%X (%s)" % (self.last_flt, flt, decode_bits(flt, FLT_BITS)))
            self.last_flt = flt
        ev = st.get("ev", 0)
        if ev != self.last_ev:
            if ev > self.last_ev:
                self.log("放电事件: 总计 %d" % ev)
            self.last_ev = ev
        for i in range(5):
            self.vv[i].set("%.3f V" % v[i])

    # ================================================================== misc
    def export_report(self):
        lines = ["# HV_control 首次上电测试报告", "",
                 "生成时间: %s" % datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                 "会话日志: %s" % self.log_path, ""]
        for title, tree in [("阶段3 开机检查/复位", self.tree3), ("阶段4 ADC标定", self.tree4),
                            ("阶段5 DAC扫描", self.tree5), ("阶段6 联锁", self.tree6)]:
            lines.append("## %s" % title)
            cols = tree["columns"]
            lines.append("| " + " | ".join(tree.heading(c)["text"] for c in cols) + " |")
            lines.append("|" + "---|" * len(cols))
            for i in tree.get_children():
                lines.append("| " + " | ".join(str(x) for x in tree.item(i, "values")) + " |")
            lines.append("")
        lines.append("## 阶段5 拟合")
        lines.append(self.fit_var.get())
        path = os.path.join(LOG_DIR, "bringup_report_%s.md" % datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
        with open(path, "w", encoding="utf-8") as fh:
            fh.write("\n".join(lines) + "\n")
        self.log("报告已保存: %s" % path)
        messagebox.showinfo("导出报告", "已保存:\n%s" % path)

    def on_close(self):
        if self.link.is_open:
            if not messagebox.askokcancel("退出", "退出前将执行 z (DAC 归零、关闭 HV、释放继电器)。继续?"):
                return
            try:
                self.link.emergency_stop()
            except HVError:
                pass
        self.poll_stop.set()
        self.job_stop.set()
        self.x_disconnect()
        self._wd_detach()      # clean release: 24 V stays on, the watchdog keeps running
        self.link.close()
        try:
            self.log_file.close()
        except Exception:
            pass
        self.root.destroy()


def main(argv=None):
    import argparse
    ap = argparse.ArgumentParser(description="HV_control bring-up GUI")
    ap.add_argument("--port", help="connect to this port at start, e.g. COM4")
    ap.add_argument("--xport", help="X-ray meter port for the stage 7 tests (default COM5)")
    ap.add_argument("--hv-max-kv", type=float, help="session cap on d targets, kV")
    ap.add_argument("--autorun", action="store_true", help="after connecting, start the stage 7 tests (asks first)")
    ap.add_argument("--remote", action="store_true",
                    help="enable local remote control on 127.0.0.1:%d (hv_remote.py)" % REMOTE_ADDR[1])
    args = ap.parse_args(argv)
    root = tk.Tk()
    app = App(root, opts=vars(args))
    if args.hv_max_kv is not None:
        app.hv_cap_units = int(args.hv_max_kv * s7.UNITS_PER_KV)
        app.limit_var.set("%.2f" % args.hv_max_kv)
        app.log("session HV cap from --hv-max-kv: %d units (%.2f kV)" % (app.hv_cap_units, args.hv_max_kv))
    if args.port:
        root.after(800, app._autorun)
    root.mainloop()


if __name__ == "__main__":
    main()
