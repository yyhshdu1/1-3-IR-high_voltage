"""
bringup_stages.py - GUI independent logic for the first power-up procedure
(see ../首次上电步骤.md).  Every function takes an HVLink plus callbacks:

    emit(name, ok, detail)   report one result row; ok = True / False / None (info only)
    log(msg)                 free text for the log pane
    ask(msg) -> bool         ask the human a yes/no question (blocks)
    stop                     threading.Event; set it to abort a running job

so that the same code can be tested from a script and driven from the GUI.
"""
import time

from hv_link import (HVCommandError, HVError, decode_bits, RST_BITS, FLT_BITS,
                     CH_NAMES)


class Aborted(Exception):
    pass


def _check_stop(stop):
    if stop is not None and stop.is_set():
        raise Aborted()


def _sleep(seconds, stop):
    t_end = time.time() + seconds
    while time.time() < t_end:
        _check_stop(stop)
        time.sleep(min(0.1, max(0.0, t_end - time.time())))


def wait_dac_settled(link, timeout=120.0, stop=None, interval=0.25):
    """Wait until the firmware slew limiter has reached the target. Returns (tgt, cur, raw)."""
    t0 = time.time()
    while True:
        _check_stop(stop)
        tgt, cur, raw = link.dstate()
        if cur == tgt:
            return tgt, cur, raw
        if time.time() - t0 > timeout:
            raise HVError("等待 DAC 到达目标超时 (tgt=%d cur=%d)" % (tgt, cur))
        time.sleep(interval)


def mean_std(vals):
    n = float(len(vals))
    m = sum(vals) / n
    var = sum((v - m) ** 2 for v in vals) / n if n > 1 else 0.0
    return m, var ** 0.5


# ----------------------------------------------------------------------------
# Stage 3: boot state
# ----------------------------------------------------------------------------
def stage3_boot_check(link, emit, log):
    banner = link.query("?")
    emit("固件版本", "FW 2." in banner, banner)

    st = link.status(quiet=False)
    checks = [
        ("hv", 0, "HV 关闭 (hv=0)"),
        ("rly", 1, "继电器在释放电平 (rly=1)"),
        ("int", 0, "积分器关闭 (int=0)"),
        ("tgt", 0, "DAC 目标 = 0"),
        ("cur", 0, "DAC 当前 = 0"),
        ("raw", 0, "DAC 码 = 0"),
        ("flt", 0, "无故障 (flt=0)"),
    ]
    for key, want, label in checks:
        got = st.get(key)
        detail = "%s=%s (期望 %s)" % (key, got, want)
        if key == "flt" and got:
            detail += "  " + decode_bits(got, FLT_BITS)
        emit(label, got == want, detail)

    adc = link.counts()
    volts = link.volts()
    stuck = sum(1 for c in adc if c <= 16 or c >= 65520) >= 3
    emit("ADC 读数有效(非全0/全1)", not stuck,
         "  ".join("%s=%.3f V" % (n, v) for n, v in zip(CH_NAMES, volts)))
    rst = st.get("rst", 0)
    emit("复位原因(信息)", None, "rst=0x%X %s | POR=%s BOR=%s EXT=%s WDT=%s SW=%s" % (
        rst, decode_bits(rst, RST_BITS), st.get("n_por"), st.get("n_bor"),
        st.get("n_ext"), st.get("n_wdt"), st.get("n_sw")))


def stage3_reset_test(link, emit, log, n=3):
    for i in range(1, n + 1):
        link.reopen(boot_wait=2.0)
        st = link.status(quiet=False)
        ok = (st.get("hv") == 0 and st.get("raw") == 0 and st.get("flt") == 0)
        emit("复位 #%d" % i, ok, "up=%s rst=0x%X hv=%s rly=%s raw=%s flt=0x%X" % (
            st.get("up"), st.get("rst", 0), st.get("hv"), st.get("rly"),
            st.get("raw"), st.get("flt", 0)))
        time.sleep(0.5)
    log("请用示波器看 J1-12(HV_enable_iso, 低有效): 复位期间应一直为高(HV 关), 不应出现向低的毛刺; "
        "若看到向低脉冲说明 HV 被短暂使能(记下脉宽并停止); 并观察 K1 在复位瞬间是否抖动。")


# ----------------------------------------------------------------------------
# Stage 4: ADC capture
# ----------------------------------------------------------------------------
def adc_capture(link, n=20, interval=0.1, stop=None):
    """Read all 5 channels n times. Returns (means, stds) lists in volts."""
    cols = [[] for _ in range(5)]
    for _ in range(n):
        _check_stop(stop)
        v = link.volts()
        for i in range(5):
            cols[i].append(v[i])
        time.sleep(interval)
    ms = [mean_std(c) for c in cols]
    return [m for m, _ in ms], [s for _, s in ms]


# ----------------------------------------------------------------------------
# Stage 5: DAC / v_prog sweep and fit
# ----------------------------------------------------------------------------
def dac_sweep(link, codes, log, stop, settle=2.0, nsamp=15, row_cb=None):
    """Step the DAC through 'codes' (host units) with HV left OFF, record v_prog (CH2).
    Returns list of dict(code, raw, ch2, std)."""
    st = link.status()
    if st.get("hv"):
        raise HVError("HV 处于开启状态, 扫描前请先关闭 HV (e 0)")
    if st.get("flt"):
        raise HVError("有故障 flt=0x%X, 请先排除并清除" % st["flt"])
    cfg = link.config()
    up = max(int(cfg.get("up", 1000)), 1)
    rows = []
    try:
        for code in codes:
            _check_stop(stop)
            link.cmd_ok("d %d" % code)
            cur_t = link.dstate()[1]
            timeout = abs(code - cur_t) / float(up) * 1.5 + 10
            tgt, cur, raw = wait_dac_settled(link, timeout=timeout, stop=stop)
            _sleep(settle, stop)
            vals = []
            for _ in range(nsamp):
                _check_stop(stop)
                vals.append(link.volts()[2])
                time.sleep(0.05)
            m, s = mean_std(vals)
            raw = link.dstate()[2]
            row = dict(code=code, raw=raw, ch2=m, std=s)
            rows.append(row)
            log("d=%d raw=%d  v_prog(CH2)=%.4f V (σ=%.1f mV)" % (code, raw, m, s * 1000))
            if row_cb:
                row_cb(row)
    finally:
        try:
            link.query("d 0")
            wait_dac_settled(link, timeout=120)    # leave the DAC really at 0 (e 1 needs it)
        except HVError:
            pass
    return rows


def linear_fit(xs, ys):
    n = float(len(xs))
    mx = sum(xs) / n
    my = sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx == 0:
        raise ValueError("degenerate fit")
    a = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    b = my - a * mx
    resid = max(abs(y - (a * x + b)) for x, y in zip(xs, ys))
    return a, b, resid


def analyse_dac(rows, scale, units30=59055, full_scale_v=10.0, margin_v=0.5, use_dmm=True):
    """rows: dict(code, ch2, dmm or None).

    Convention (as in the v1 notebooks): host units 'units30' (59055) = 30 kV = full_scale_v
    on voltage_program.  The host unit -> DAC code factor is 'scale', so the calibration
    knob is the SCALE; dacmax stays at units30.  Returns the fit and recommendations."""
    have_dmm = use_dmm and all(r.get("dmm") is not None for r in rows)
    ys = [r["dmm"] if have_dmm else r["ch2"] for r in rows]
    xs = [float(r["code"]) for r in rows]
    a, b, resid = linear_fit(xs, ys)
    out = dict(source="万用表" if have_dmm else "ADC CH2", slope=a, offset=b, resid=resid)
    if a <= 0:
        out["error"] = "斜率 <= 0, 数据不对"
        return out
    code_fs = (full_scale_v - b) / a                 # host units giving full_scale_v with the CURRENT scale
    raw_fs = code_fs * scale                         # DAC code that gives full_scale_v
    new_scale = raw_fs / float(units30)              # makes units30 <-> full_scale_v
    ok = 0.05 < new_scale < 3.0 and raw_fs <= 65535.0
    out.update(code_fs=code_fs, raw_fs=raw_fs, units30=units30,
               rec_scale=round(new_scale, 4), rec_dacmax=int(units30),
               rec_vprogmax=round(min(12.0, max(1.0, full_scale_v + margin_v)), 2),
               scale_ok=ok, v_at_units30_now=a * units30 + b)
    return out


# ----------------------------------------------------------------------------
# Stage 6: interlocks
# ----------------------------------------------------------------------------
def _expect(emit, name, reply, want, contains=False):
    ok = (want in reply) if contains else (reply == want)
    emit(name, ok, "回复 %r (期望 %s%r)" % (reply, "含 " if contains else "", want))
    return ok


def stage6_interlocks(link, emit, ask, log, stop):
    """Needs J1 disconnected from the TOF3000 (it drives the DAC with HV enabled)."""
    def zero_everything():
        link.query("d 0", quiet=True)
        try:
            wait_dac_settled(link, timeout=60, stop=None)
        except HVError:
            pass
        link.query("e 0", quiet=True)
        link.query("r 1", quiet=True)

    link.query("C")
    zero_everything()
    st = link.status()
    if st.get("flt"):
        emit("前置条件: 无故障", False, "flt=0x%X %s" % (st["flt"], decode_bits(st["flt"], FLT_BITS)))
        return
    pol0 = int(link.query("P"))

    try:
        _check_stop(stop)
        _expect(emit, "r 0 被拒 (PID 锁住, FW 2.5)", link.query("r 0"), "ERR PIDOFF")

        # v1 raw-pin commands: "h0" meant ON there, so FW 2.7 refuses h/H outright and changes nothing
        _expect(emit, "旧命令 h 0 被拒 (v1 的 h0 = 开)", link.query("h 0"), "ERR USE_E")
        _expect(emit, "旧命令 h 1 被拒", link.query("h 1"), "ERR USE_E")
        _expect(emit, "旧命令 H 被拒", link.query("H"), "ERR USE_E")
        _expect(emit, "被拒后 HV 仍关 (E = 0)", link.query("E"), "0")

        _expect(emit, "e 1 (DAC=0) 允许", link.query("e 1"), "OK")
        _expect(emit, "E 回读 = 1", link.query("E"), "1")
        ok = ask("【人工测量】现在 HV 已使能 (e 1)。\n用万用表量 J1-12 (HV_enable_iso) 对 GND。\n"
                 "HV_enable 是低有效, 预期约 0 V。\n读数是否约 0 V (< 0.5 V)? (此时 J1 不应接 TOF3000)")
        emit("J1-12 在 e 1 时为低 (低有效, 人工)", bool(ok),
             "若'否': 引脚极性与固件假设相反, 先不要接 TOF3000, 见首次上电步骤.md 阶段 3")

        # ---- the integrator is locked out in FW 2.5 (c pid 0): r 0 must be refused even with HV on ----
        _expect(emit, "HV 开着时 r 0 仍被拒 (PID 锁住)", link.query("r 0"), "ERR PIDOFF")
        if link.status().get("flt"):
            emit("r 0 之后无故障", False, "flt=0x%X" % link.status()["flt"])
            return
        emit("K1 线圈实测", None, "积分器被固件锁住, 本测试不吸合 K1; 线圈电压 (吸合约 5 V / 释放 0 V) 如需验证请用 spi_probe 固件单独测")
        _expect(emit, "HV 开时 p 1 被拒", link.query("p 1"), "ERR HV_ON")

        _expect(emit, "d 3000 允许", link.query("d 3000"), "OK")
        time.sleep(0.5)
        _expect(emit, "DAC≠0 时 e 1 被拒", link.query("e 1"), "ERR DAC_NZ")

        _expect(emit, "e 0 允许", link.query("e 0"), "OK")
        ok = ask("【人工测量】现在 HV 已关闭 (e 0)。\n用万用表量 J1-12 (HV_enable_iso) 对 GND。\n"
                 "低有效时 HV 关 = 高电平, 预期约 4.5 V。\n读数是否约 4–5 V?")
        emit("J1-12 在 e 0 时为高 (低有效, 人工)", bool(ok),
             "若'否': 引脚极性与固件假设相反, 先不要接 TOF3000")
        d = link.query("D")
        emit("e 0 后 DAC 立即清零 (D=0,0,0)", d == "0,0,0", "D=%r" % d)

        other = 0 if pol0 else 1
        _expect(emit, "HV 关且 DAC=0 时 p 允许", link.query("p %d" % other), "OK")
        _expect(emit, "P 回读极性已变", link.query("P"), str(other))
        _expect(emit, "恢复原极性", link.query("p %d" % pol0), "OK")

        _expect(emit, "d 99999 超范围", link.query("d 99999"), "ERR RANGE")
        _expect(emit, "d -5 超范围", link.query("d -5"), "ERR RANGE")
        _expect(emit, "d abc 参数错误", link.query("d abc"), "ERR ARG")
        _expect(emit, "错误校验和被拒", link.query("?*00", raw=True), "ERR CHK")
        _expect(emit, "正确校验和通过", link.query("?", raw=False), "HV_control", contains=True)
    except Aborted:
        log("已中止")
    finally:
        try:
            zero_everything()
        except HVError as e:
            log("收尾失败: %s" % e)
            try:
                link.emergency_stop()
            except HVError:
                pass


# ----------------------------------------------------------------------------
# Stage 7/8: staircase ramp
# ----------------------------------------------------------------------------
def run_staircase(link, targets_units, hold_s, log, stop, on_step=None):
    """Go through the target list (host units), hold each for hold_s seconds.
    Aborts and ramps down on any firmware fault.  If the firmware backs the voltage off by itself
    (X-ray alarm, or a spark back-off) the climb stops but the voltage is left at the backed-off
    level (the firmware keeps refusing increases while the X-ray alarm is active).
    HV must already be enabled."""
    keep = False
    try:
        for i, target in enumerate(targets_units):
            _check_stop(stop)
            if on_step:
                on_step(i, target)
            try:
                link.cmd_ok("d %d" % target)
            except HVCommandError as e:
                if "XRAY" in str(e.reply):
                    log("X 射线告警中, 固件拒绝升压 (ERR XRAY), 停止爬升并保持当前电压")
                    keep = True
                    raise Aborted()
                raise
            cur = link.dstate()[1]
            cfg = link.config()
            rate = max(int(cfg.get("up", 1000)), 1) if target >= cur else max(int(cfg.get("down", 4000)), 1)
            st0 = link.status()
            ev0, xt0, bo0 = st0.get("ev", 0), st0.get("xtrips", 0), st0.get("backoffs", 0)
            wait_dac_settled(link, timeout=abs(target - cur) / float(rate) * 1.5 + 10, stop=stop)
            t_end = time.time() + hold_s
            while time.time() < t_end:
                _check_stop(stop)
                st = link.status()
                if st.get("flt"):
                    log("故障 flt=0x%X (%s), 终止爬升并降压" % (st["flt"], decode_bits(st["flt"], FLT_BITS)))
                    raise Aborted()
                if st.get("xtrips", 0) > xt0:
                    log("台阶 %d: X 射线告警, 固件已回退 (目标现为 %s, 读数 %s×0.01 µSv/h), 停止爬升并保持" % (
                        i + 1, st.get("tgt"), st.get("xr")))
                    keep = True
                    raise Aborted()
                if st.get("backoffs", 0) > bo0:
                    log("台阶 %d: 打火触发回退 (目标现为 %s), 停止爬升并保持" % (i + 1, st.get("tgt")))
                    keep = True
                    raise Aborted()
                if st.get("ev", 0) > ev0:
                    log("台阶 %d: 放电事件 +%d (总 %d)" % (i + 1, st["ev"] - ev0, st["ev"]))
                    ev0 = st["ev"]
                time.sleep(0.5)
        log("阶梯爬升完成")
    except Aborted:
        log("阶梯爬升已中止")
    finally:
        if not keep:
            try:
                link.query("d 0")
                wait_dac_settled(link, timeout=120)
            except HVError:
                pass
