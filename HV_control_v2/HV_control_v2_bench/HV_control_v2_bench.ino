// BENCH-SIM BUILD: for a bare Nano Every only. Never flash to the real HV_control board.
#define HV_BENCH_SIM 1
/*
 * HV_control_v2.ino  --  firmware for the CeNTREX HV_control v1.3 board
 *                        (Arduino Nano Every / ATmega4809 + MAX5216 DAC + MAX1300 ADC)
 *
 * Differences from HV_control.ino (v1):
 *   - Boots into a SAFE state: HV disabled, DAC = 0, integrator relay released.
 *     (v1 also booted with HV_enable HIGH = off, but did not zero the DAC, which keeps its code
 *     through an Arduino reset.)
 *   - HV enable is the NEW command "e" (e 1 = HV on, e 0 = off).  The v1 commands "h" / "H" are
 *     REMOVED and answer "ERR USE_E": in v1 "h0" meant ENABLE (raw active-low pin, 0 = on), so an old
 *     notebook or driver talking to this firmware would otherwise turn the HV on with what it thinks is
 *     "disable".
 *   - Line-based command protocol with strict validation and optional XOR checksum.
 *     A stray byte can no longer change polarity / HV_enable / DAC (v1 used parseInt()).
 *   - Hardware watchdog (about 2 s) and reset-cause logging (RSTCTRL.RSTFR + counters
 *     that survive watchdog/software resets).
 *   - DAC setpoint limit, slew-rate limit, interlocks (HV can only be enabled at DAC = 0,
 *     polarity can only change with HV off and DAC = 0).
 *   - Periodic DAC refresh and read-back check through ADC CH2 (voltage_program_buf).
 *   - voltage_program over-limit trip (stops integrator wind-up from reaching the TOF3000).
 *   - ADC-stuck detection (dead SPI / isolator failure -> shutdown).
 *   - Fast ADC sampling with spark (current-event) counting, peak hold, optional auto back-off.
 *   - Optional host-heartbeat timeout (default OFF, see "c timeout").
 *   - Configuration (board A/B, DAC scale, limits) stored in EEPROM.
 *
 * NOT YET COMPILED OR TESTED ON HARDWARE.  Build with e.g.
 *   arduino-cli compile --fqbn arduino:megaavr:nona4809 HV_control_v2
 * Bench-test with the HV cable disconnected before using on the real system.
 *
 * ---------------------------------------------------------------------------
 * FW 2.8: fixed random false trips of "c timeout" / "c xtimeout" (clock read before the serial port, see loop()).
 *
 * COMMAND SET  (one command per line, terminated by \n or \r\n, 9600 baud)
 *   Syntax is "<letter><integer>" as in v1, spaces allowed ("d 1234").
 *   An optional checksum may be appended: "<cmd>*HH" where HH is the 2-digit hex XOR
 *   of every character before '*'.  Example (Python):
 *       def cs(s): x=0; [x:=x^ord(c) for c in s]; return f"{s}*{x:02X}\n"
 *   Replies: queries print their value; set-commands print "OK" only if "c ack 1";
 *   errors always print "ERR <reason>".
 *
 *   ?        identification string
 *   a        raw ADC counts "v_mon,i_mon,v_prog,v_div,v_err"   (same format as v1)
 *   b        same five channels in volts
 *   d N      set DAC target in HOST units (same units as v1: 59055 = 30 kV);
 *            the output follows with the configured slew rate
 *   D        print "target,current,raw_dac_code"
 *   z        EMERGENCY: DAC = 0 now, integrator off, HV_enable off
 *   e 0|1    HV on/off (logical, NOT the v1 raw-pin meaning).  e 1 only allowed when DAC output is 0 and
 *            no fault is latched.  The PIN is active-low: e 1 drives HV_enable LOW, e 0 drives it HIGH.
 *   E        print the commanded HV state 1=on 0=off (NOT a read-back of the pin/isolator)
 *   h, H     removed (v1 raw-pin commands, h0 = ON there): reply "ERR USE_E", nothing is changed
 *   p 0|1    polarity pin.  Only allowed with HV disabled, DAC = 0 and |v_mon| <= 0.1 V (ERR VMON);
 *            a floating v_mon input (J1 cable off, reads full scale) does not block it
 *   P        print polarity pin state
 *   r 0|1    relay_ctl pin level (raw, as in v1).  See RELAY_* below; the level that
 *            activates the integrator is refused with ERR PIDOFF unless "c pid 1" (default 0:
 *            this project does not use the PID), and only accepted while HV is enabled
 *   R        print relay_ctl pin level
 *   s        status line (uptime, reset cause + counters, state, faults, ...)
 *   m        spark statistics since last "m"  (events, peak current, min voltage)
 *   C        clear latched faults
 *   t N      host-heartbeat timeout in seconds (0 = off).  Any valid command is a heartbeat
 *   k        keep-alive (does nothing else)
 *   c        list configuration;  "c <key> <value>" set;  "c save" / "c load" / "c defaults"
 *       keys: board(A|B) scale dacmax up down timeout vprogmax vprogtol ethr sthr spark
 *             sparkwin backoff xthr xback xtimeout pol name pid cable strict ack
 *       pid    1 = integrator may be engaged (default 0).   cable  1 = fault 0x80 when v_mon / i_mon
 *       float at full scale (J1 cable off) while HV is on (default 0; set it from the TOF3000 stage on)
 *   x N      X-ray dose-rate report from the host, N in units of 0.01 uSv/h (see below)
 *
 * CURRENT EVENTS (I_mon, 40 uA per volt = 0.375 mV per ADC count = 15 nA per count)
 *   ethr  LOW threshold (counts above a slowly tracking baseline): events are only COUNTED
 *         ("ev" in s, "ev" in m).  Every event above sthr is also counted here.
 *   sthr  HIGH threshold: a "spark" (default 2500 counts = 37.5 uA above baseline since FW 2.6; it was
 *         1300 = 20 uA, which the 20-25 uA ramp transients of the lab data exceed).  A value saved in
 *         EEPROM is NOT changed by a re-flash: set it with "c sthr 2500" + "c save" on boards already in use.
 *         With "spark N" > 0, N sparks within "sparkwin" seconds
 *         back the target off by "backoff" host units (backoff 0 = ramp down, lock fault 0x08).
 *
 * X-RAY GUARD  (the dose-rate monitor, e.g. RadEye G20, is read by the PC, which forwards it)
 *   The PC sends "x N" about once per second.  xthr = alarm level (0 = guard off).
 *   N >= xthr : the target is lowered below the CURRENT output by xback units (0 = ramp down
 *               and lock fault 0x20), repeated at most every 3 s while the level stays high;
 *               raising the target with "d" is refused (ERR XRAY) until N < 80 % of xthr.
 *   xtimeout  seconds without any "x" report while HV is on and DAC > 0 (0 = off): ramp down and
 *               lock fault 0x40 (monitor lost).  The first report is expected within xtimeout of "e 1".
 * ---------------------------------------------------------------------------
 */

#include <SPI.h>
#include <EEPROM.h>
#include <avr/io.h>
#include <string.h>
#include <stdlib.h>

#if !defined(__AVR_ATmega4809__)
#error "This firmware targets the Arduino Nano Every (ATmega4809)."
#endif

#define FW_VERSION      "2.8"
// 'r 0' (integrator on) is refused unless |error_voltage_buf| is below this many volts
// (error = V_div - 3 V; 0.30 V is about 3 kV from the 30 kV set point).
#define ERR_ENGAGE_MAX_V 0.30f
#define POL_VMON_MAX_V   0.10f   // polarity may only change when |v_mon| is below this (about 300 V residual)
#define CABLE_RAIL_ADC   64000   // CH0/CH1 above this (about +12 V) = monitor input floating (J1 cable not plugged)
#define CABLE_TRIP_CNT   12      // consecutive 20 ms samples (about 250 ms) before the cable fault trips
#define DEFAULT_BOARD   'B'      // used only if EEPROM has no valid configuration

// ---------------------------------------------------------------------------
// Pins (unchanged from v1)
// ---------------------------------------------------------------------------
const uint8_t CS_ADC           = 3;
const uint8_t CS_DAC           = 4;
const uint8_t relay_ctl        = 2;
const uint8_t polarity_control = 5;
const uint8_t HV_enable        = 6;

// HV_enable is ACTIVE-LOW at the supply: pin LOW = HV enabled, pin HIGH = HV disabled
// (fail-safe: cable break, power loss, floating input and the isolator/pull-up default
// level are all HIGH = off).  Host commands use the logical meaning: "e 1" = HV on (v1 "h0" = on is gone, see "e").
// If the supply's manual says otherwise, swap these two constants and nothing else.
const uint8_t HV_ENABLE_ON_LEVEL  = LOW;
const uint8_t HV_ENABLE_OFF_LEVEL = HIGH;

// relay_ctl -> ADuM7440 -> 74LVC1GU04 inverter -> NUD3124 -> relay K1.
// Reading of the v1.3 schematic: pin HIGH = relay released = integrator OFF (hold),
// pin LOW = relay energised = integrator ACTIVE.  VERIFY ON THE BENCH.
// If it turns out to be the other way round, swap these two constants.
const uint8_t RELAY_OFF_LEVEL    = HIGH;
const uint8_t RELAY_ACTIVE_LEVEL = LOW;

// ---------------------------------------------------------------------------
// ADC / timing constants
// ---------------------------------------------------------------------------
const uint8_t  NUM_CH            = 5;      // CH0 v_mon, CH1 i_mon, CH2 v_prog_buf, CH3 v_div_buf, CH4 v_err_buf
const uint8_t  CH_VMON = 0, CH_IMON = 1, CH_VPROG = 2, CH_VDIV = 3, CH_VERR = 4;
const uint16_t ADC_PERIOD_MS     = 20;     // all 5 channels every 20 ms
const uint16_t SLEW_PERIOD_MS    = 20;
const uint16_t DAC_REFRESH_MS    = 250;    // re-send DAC code (heals corrupted SPI writes)
const uint16_t LINE_TIMEOUT_MS   = 2000;   // drop an incomplete command line after this long
const float    ADC_FULLSCALE_V   = 2.0f * 3.0f * 4.096f;   // 24.576 V span, as in v1 Python code

// ---------------------------------------------------------------------------
// Fault flags
// ---------------------------------------------------------------------------
const uint8_t F_VPROG   = 0x01;   // voltage_program above limit
const uint8_t F_DACMISM = 0x02;   // DAC read-back does not follow the written code
const uint8_t F_TIMEOUT = 0x04;   // host heartbeat lost
const uint8_t F_SPARK   = 0x08;   // too many sparks, no back-off configured
const uint8_t F_ADC     = 0x10;   // ADC returns stuck values
const uint8_t F_XRAY    = 0x20;   // X-ray alarm with xback = 0 (ramped down)
const uint8_t F_XMON    = 0x40;   // X-ray monitor reports stopped arriving (xtimeout)
const uint8_t F_CABLE   = 0x80;   // v_mon / i_mon input floating at full scale while HV is on (J1 cable off)

// ---------------------------------------------------------------------------
// Reset-cause capture (runs before main(); RAM survives watchdog / software resets)
// ---------------------------------------------------------------------------
static uint8_t g_rstfr __attribute__((section(".noinit")));

void capture_reset_flags(void) __attribute__((naked, used, section(".init3")));
void capture_reset_flags(void)
{
  g_rstfr = RSTCTRL.RSTFR;
  RSTCTRL.RSTFR = 0xFF;          // write 1 to clear
}

struct Persist {
  uint32_t magic;
  uint16_t n_por, n_bor, n_ext, n_wdt, n_sw;
};
static Persist g_persist __attribute__((section(".noinit")));
const uint32_t PERSIST_MAGIC = 0x48564332UL;   // "HVC2"

// ---------------------------------------------------------------------------
// Configuration (EEPROM)
// ---------------------------------------------------------------------------
struct __attribute__((packed)) Config {
  uint16_t magic;
  uint8_t  version;
  char     board;          // 'A' or 'B'
  uint8_t  ack;            // print "OK" after set-commands
  uint8_t  strict;         // require checksum on every line
  uint8_t  spark_n;        // sparks within spark_win_s that trigger back-off (0 = off, max 16)
  float    scale;          // host unit -> raw DAC code
  float    vprog_max_v;    // trip if voltage_program above this
  float    vprog_tol_v;    // allowed DAC read-back deviation
  uint16_t dac_max;        // max accepted "d" value (host units)
  uint16_t up_rate;        // host units / s, 0 = unlimited
  uint16_t down_rate;      // host units / s, 0 = unlimited
  uint16_t timeout_s;      // host heartbeat timeout, 0 = off
  uint16_t ethr;           // spark threshold: ADC counts of I_mon above baseline
  uint16_t spark_win_s;    // window for spark counting
  uint16_t backoff;        // host units to reduce target by on spark trip (0 = full shutdown ramp)
  // ---- added in config version 3 ----
  uint16_t sthr;           // spark (back-off) threshold: ADC counts of I_mon above baseline
  uint16_t xthr;           // X-ray alarm level, 0.01 uSv/h units (0 = guard off)
  uint16_t xback;          // host units to back off by per X-ray alarm step (0 = ramp down + fault)
  uint16_t xtimeout_s;     // X-ray monitor-lost timeout in seconds (0 = off)
  // ---- added in config version 4 ----
  uint8_t  pol;            // polarity pin level applied at boot: 1 = HIGH (positive), 0 = LOW (negative)
  char     name[9];        // free label of the supply this board is wired to, e.g. "TOF+" / "TOF-"
  // ---- added in config version 5 ----
  uint8_t  pid;            // 1 = the K1 integrator ("r 0") may be engaged; default 0 = locked out
  uint8_t  cable;          // 1 = trip F_CABLE when v_mon/i_mon float at full scale while HV is on
  uint16_t sum;
};
// Layout of config version 4 (FW 2.4), kept only to migrate saved settings.
struct __attribute__((packed)) ConfigV4 {
  uint16_t magic; uint8_t version; char board; uint8_t ack; uint8_t strict; uint8_t spark_n;
  float scale; float vprog_max_v; float vprog_tol_v;
  uint16_t dac_max; uint16_t up_rate; uint16_t down_rate; uint16_t timeout_s;
  uint16_t ethr; uint16_t spark_win_s; uint16_t backoff;
  uint16_t sthr; uint16_t xthr; uint16_t xback; uint16_t xtimeout_s;
  uint8_t pol; char name[9]; uint16_t sum;
};
// Layout of config version 3 (FW 2.3), kept only to migrate saved settings.
struct __attribute__((packed)) ConfigV3 {
  uint16_t magic; uint8_t version; char board; uint8_t ack; uint8_t strict; uint8_t spark_n;
  float scale; float vprog_max_v; float vprog_tol_v;
  uint16_t dac_max; uint16_t up_rate; uint16_t down_rate; uint16_t timeout_s;
  uint16_t ethr; uint16_t spark_win_s; uint16_t backoff;
  uint16_t sthr; uint16_t xthr; uint16_t xback; uint16_t xtimeout_s; uint16_t sum;
};
// Layout of config version 2 (FW 2.0 - 2.2), kept only to migrate saved calibrations.
struct __attribute__((packed)) ConfigV2 {
  uint16_t magic; uint8_t version; char board; uint8_t ack; uint8_t strict; uint8_t spark_n;
  float scale; float vprog_max_v; float vprog_tol_v;
  uint16_t dac_max; uint16_t up_rate; uint16_t down_rate; uint16_t timeout_s;
  uint16_t ethr; uint16_t spark_win_s; uint16_t backoff; uint16_t sum;
};
const uint16_t CFG_MAGIC   = 0x4856;
const uint8_t  CFG_VERSION = 5;
static Config cfg;

// NOTE: takes void* (not Config&) because the Arduino IDE auto-generates function
// prototypes above the struct definition; a Config parameter would not compile there.
static uint16_t cfg_sum(const void *vp, size_t len)
{
  const uint8_t *p = (const uint8_t *)vp;
  uint16_t s = 0x1234;
  for (size_t i = 0; i < len - sizeof(uint16_t); i++) s = (uint16_t)(s * 31 + p[i]);
  return s;
}

static void cfg_defaults(char board)
{
  memset(&cfg, 0, sizeof(cfg));
  cfg.magic       = CFG_MAGIC;
  cfg.version     = CFG_VERSION;
  cfg.board       = board;
  cfg.scale       = (board == 'A') ? 0.961f : 0.726f;   // calibration constants from v1
  cfg.vprog_max_v = 11.0f;     // set ~0.5 V above the value measured at 30 kV ("b" command)
  cfg.vprog_tol_v = 0.10f;
  cfg.dac_max     = 59055;     // 30 kV in the v1 notebooks
  cfg.up_rate     = 1000;      // ~0.5 kV/s
  cfg.down_rate   = 4000;      // ~2 kV/s
  cfg.timeout_s   = 0;
  cfg.ethr        = 100;       // LOW threshold, count only: 100 counts = 37 mV = 1.5 uA on I_mon
  cfg.sthr        = 2500;      // HIGH threshold (spark): 2500 counts = 0.94 V = 37.5 uA above baseline
                               // (FW 2.5 had 1300 = 20 uA, but every 10-20 s ramp of the 2025 lab data has a 20-25 uA
                               //  turn-on transient; real sparks were 44 uA and up)
  cfg.spark_n     = 0;
  cfg.spark_win_s = 60;
  cfg.backoff     = 1968;      // ~1 kV
  cfg.xthr        = 0;         // X-ray guard off until configured
  cfg.xback       = 3936;      // ~2 kV per alarm step
  cfg.xtimeout_s  = 0;
  cfg.pol         = (board == 'A') ? 1 : 0;   // v1 board defaults; set per supply with "c pol 1|0" + "c save"
  cfg.name[0]     = 0;
  cfg.pid         = 0;         // integrator locked out
  cfg.cable       = 0;         // cable check off until enabled (J1 is open during the early bring-up stages)
}

static void cfg_save()
{
  cfg.sum = cfg_sum(&cfg, sizeof(Config));
  EEPROM.put(0, cfg);
}

static bool cfg_sane()
{
  if (cfg.board != 'A' && cfg.board != 'B') return false;
  if (!(cfg.scale > 0.05f && cfg.scale < 3.0f)) return false;
  if (cfg.pol > 1) return false;
  cfg.name[sizeof(cfg.name) - 1] = 0;
  return true;
}

// Version 4 -> 5: keep everything, add pid = 0 and cable = 0.
static bool cfg_migrate_v4()
{
  ConfigV4 o;
  EEPROM.get(0, o);
  if (o.magic != CFG_MAGIC || o.version != 4 || o.sum != cfg_sum(&o, sizeof(ConfigV4))) return false;
  if (o.board != 'A' && o.board != 'B') return false;
  if (!(o.scale > 0.05f && o.scale < 3.0f)) return false;
  cfg_defaults(o.board);
  cfg.ack = o.ack; cfg.strict = o.strict; cfg.spark_n = o.spark_n;
  cfg.scale = o.scale; cfg.vprog_max_v = o.vprog_max_v; cfg.vprog_tol_v = o.vprog_tol_v;
  cfg.dac_max = o.dac_max; cfg.up_rate = o.up_rate; cfg.down_rate = o.down_rate;
  cfg.timeout_s = o.timeout_s; cfg.spark_win_s = o.spark_win_s; cfg.backoff = o.backoff;
  cfg.ethr = o.ethr; cfg.sthr = o.sthr; cfg.xthr = o.xthr; cfg.xback = o.xback; cfg.xtimeout_s = o.xtimeout_s;
  cfg.pol = (o.pol > 1) ? ((o.board == 'A') ? 1 : 0) : o.pol;
  memcpy(cfg.name, o.name, sizeof(cfg.name));
  cfg.name[sizeof(cfg.name) - 1] = 0;
  cfg_save();
  return true;
}

// Version 3 -> 4: keep everything, add pol (from the board default) and an empty name.
static bool cfg_migrate_v3()
{
  ConfigV3 o;
  EEPROM.get(0, o);
  if (o.magic != CFG_MAGIC || o.version != 3 || o.sum != cfg_sum(&o, sizeof(ConfigV3))) return false;
  if (o.board != 'A' && o.board != 'B') return false;
  if (!(o.scale > 0.05f && o.scale < 3.0f)) return false;
  cfg_defaults(o.board);
  cfg.ack = o.ack; cfg.strict = o.strict; cfg.spark_n = o.spark_n;
  cfg.scale = o.scale; cfg.vprog_max_v = o.vprog_max_v; cfg.vprog_tol_v = o.vprog_tol_v;
  cfg.dac_max = o.dac_max; cfg.up_rate = o.up_rate; cfg.down_rate = o.down_rate;
  cfg.timeout_s = o.timeout_s; cfg.spark_win_s = o.spark_win_s; cfg.backoff = o.backoff;
  cfg.ethr = o.ethr; cfg.sthr = o.sthr; cfg.xthr = o.xthr; cfg.xback = o.xback; cfg.xtimeout_s = o.xtimeout_s;
  cfg_save();
  return true;
}

// Version 2 -> 4: keep the saved calibration and limits, add the new fields with defaults.
static bool cfg_migrate_v2()
{
  ConfigV2 o;
  EEPROM.get(0, o);
  if (o.magic != CFG_MAGIC || o.version != 2 || o.sum != cfg_sum(&o, sizeof(ConfigV2))) return false;
  if (o.board != 'A' && o.board != 'B') return false;
  if (!(o.scale > 0.05f && o.scale < 3.0f)) return false;
  cfg_defaults(o.board);
  cfg.ack = o.ack; cfg.strict = o.strict; cfg.spark_n = o.spark_n;
  cfg.scale = o.scale; cfg.vprog_max_v = o.vprog_max_v; cfg.vprog_tol_v = o.vprog_tol_v;
  cfg.dac_max = o.dac_max; cfg.up_rate = o.up_rate; cfg.down_rate = o.down_rate;
  cfg.timeout_s = o.timeout_s; cfg.spark_win_s = o.spark_win_s; cfg.backoff = o.backoff;
  cfg.ethr = (o.ethr == 200) ? 100 : o.ethr;     // 200 was the old default (it counted sparks); 100 is the new count-only default
  cfg_save();
  return true;
}

static bool cfg_load()
{
  Config c;
  EEPROM.get(0, c);
  if (c.magic == CFG_MAGIC && c.version == CFG_VERSION && c.sum == cfg_sum(&c, sizeof(Config))) {
    cfg = c;
    if (cfg_sane()) return true;
    return false;
  }
  return cfg_migrate_v4() || cfg_migrate_v3() || cfg_migrate_v2();
}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
static bool     hv_en = false;
static uint8_t  pol_level = LOW;
static uint8_t  relay_level = RELAY_OFF_LEVEL;
static float    dac_tgt = 0, dac_cur = 0;      // host units
static uint16_t dac_raw = 0;
static uint8_t  faults = 0;
static bool     pending_disable = false;       // disable HV once the DAC has ramped down to 0
static uint32_t last_host_ms = 0;

static uint16_t adc[NUM_CH];
static uint32_t t_adc = 0, t_slew = 0, t_refresh = 0;

// spark statistics
static float    ibase = 0;
static bool     ibase_init = false;
static bool     in_event = false;              // above the LOW threshold (ethr)
static bool     in_spark = false;              // above the HIGH threshold (sthr)
static uint32_t ev_total = 0;                  // events above ethr (includes sparks)
static uint16_t ev_win = 0;
static uint32_t sp_total = 0;                  // events above sthr
static uint16_t sp_win = 0;
static uint16_t imax_win = 0, vmin_win = 0xFFFF;
static uint32_t ev_ts[16];                     // time stamps of recent SPARKS (sthr)
static uint8_t  ev_cnt = 0;
static uint16_t spark_backoffs = 0, dac_fixes = 0;

// X-ray guard (values come from the PC via the "x" command)
const uint32_t X_STEP_GAP_MS = 3000;           // min time between two X-ray back-off steps (detector averaging lags)
static uint16_t x_level = 0;                   // last reported level, 0.01 uSv/h
static bool     x_hold = false;                // alarm active: target may not be raised
static bool     x_seen = false;                // at least one report received since boot
static uint32_t x_last_ms = 0;                 // time of the last report (or of "e 1")
static uint32_t x_step_ms = 0;                 // time of the last back-off step
static bool     x_step_valid = false;
static uint16_t x_trips = 0;                   // number of X-ray back-off steps / ramp-downs

// interlock counters
static uint8_t  adc_bad_cnt = 0, vprog_cnt = 0, mism_cnt = 0, cable_cnt = 0;
static uint16_t ver_code = 0xFFFF, ref_val = 0;
static uint32_t ver_t0 = 0;
static bool     ref_ok = false;

// ---------------------------------------------------------------------------
// BENCH SIMULATION (compile-time only; NEVER define this for the real board).
// With HV_BENCH_SIM the ADC is replaced by a software model so that the protocol,
// interlocks, watchdog and reset logging can be tested on a bare Nano Every.
// Extra commands (bench builds only):
//   X <ms>  inject a current spark of <ms> length     Y <ms>  corrupt CH2 read-back for <ms>
//   N <ms>  ADC returns 0xFFFF for <ms>                V <cnt> add <cnt> ADC counts to CH2 (0 = off)
//   W <ms>  CH0 and CH1 return full scale for <ms> (floating J1 monitor inputs)
//   U <cnt> add <cnt> ADC counts to CH0 (v_mon offset, 0 = off)
//   Q       hang the main loop (watchdog must reset the chip)
// ---------------------------------------------------------------------------
#ifdef HV_BENCH_SIM
static uint32_t sim_spark_until = 0, sim_glitch_until = 0, sim_stuck_until = 0, sim_rail_until = 0;
static uint16_t sim_vprog_add = 0, sim_vmon_add = 0;
#endif

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------
static inline void wdt_kick() { __asm__ __volatile__("wdr"); }

static void reply_ok()  { if (cfg.ack) Serial.println(F("OK")); }
static void reply_err(const __FlashStringHelper *why) { Serial.print(F("ERR ")); Serial.println(why); }

static bool parse_long(const char *s, long &out)
{
  while (*s == ' ') s++;
  if (!*s) return false;
  char *e;
  long v = strtol(s, &e, 10);
  while (*e == ' ') e++;
  if (*e) return false;
  out = v;
  return true;
}

static bool parse_float(const char *s, float &out)
{
  while (*s == ' ') s++;
  if (!*s) return false;
  char *e;
  float v = (float)strtod(s, &e);
  while (*e == ' ') e++;
  if (*e) return false;
  out = v;
  return true;
}

static float adc_to_volts(uint16_t raw) { return ((float)raw - 32768.0f) / 65535.0f * ADC_FULLSCALE_V; }
static uint16_t volts_to_adc(float v)
{
  float r = 32768.0f + v * 65535.0f / ADC_FULLSCALE_V;
  if (r < 0) r = 0;
  if (r > 65535) r = 65535;
  return (uint16_t)r;
}

// ---------------------------------------------------------------------------
// SPI devices
// ---------------------------------------------------------------------------
static uint16_t adc_read(uint8_t ch)       // MAX1300BEUG+ : same transaction as v1
{
#ifdef HV_BENCH_SIM
  uint32_t t = millis();
  if ((int32_t)(sim_stuck_until - t) > 0) return 0xFFFF;
  uint32_t base = 32768UL + dac_raw / 2;               // about 8 V "program" at the 30 kV code of board B
  uint32_t v;
  switch (ch) {
    case CH_VMON:  v = ((int32_t)(sim_rail_until - t) > 0) ? 65535UL : base + sim_vmon_add; break;
    case CH_IMON:  v = ((int32_t)(sim_rail_until - t) > 0) ? 65535UL : 32768UL + 100 + (((int32_t)(sim_spark_until - t) > 0) ? 1500 : 0); break;
    case CH_VPROG: v = base + sim_vprog_add + (((int32_t)(sim_glitch_until - t) > 0) ? 1300 : 0); break;
    case CH_VDIV:  v = base; break;
    default:       v = 32768UL; break;
  }
  return (uint16_t)(v > 65535UL ? 65535UL : v);
#else
  uint8_t d[4] = { (uint8_t)(0x80 + (ch << 4)), 0, 0, 0 };
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS_ADC, LOW);
  SPI.transfer(d, 4);
  digitalWrite(CS_ADC, HIGH);
  SPI.endTransaction();
  return ((uint16_t)d[2] << 8) | d[3];
#endif
}

static void dac_write(uint16_t code)       // MAX5216BGUA+ : same frame as v1, but unsigned
{
  uint8_t d[3] = {
    (uint8_t)(((code >> 10) & 0x3F) | 0x40),
    (uint8_t)((code >> 2) & 0xFF),
    (uint8_t)((code & 0x03) << 6)
  };
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  digitalWrite(CS_DAC, LOW);
  SPI.transfer(d, 3);
  digitalWrite(CS_DAC, HIGH);
  SPI.endTransaction();
  dac_raw = code;
}

static uint16_t host_to_raw(float host)
{
  float r = host * cfg.scale + 0.5f;
  if (r < 0) r = 0;
  if (r > 65535.0f) r = 65535.0f;
  return (uint16_t)r;
}

// ---------------------------------------------------------------------------
// Outputs
// ---------------------------------------------------------------------------
static void set_hv(bool on)          { hv_en = on; digitalWrite(HV_enable, on ? HV_ENABLE_ON_LEVEL : HV_ENABLE_OFF_LEVEL); }
static void set_relay(uint8_t level) { relay_level = level; digitalWrite(relay_ctl, level); }
static void set_pol(uint8_t level)   { pol_level = level; digitalWrite(polarity_control, level); }
static bool integrator_active()      { return relay_level == RELAY_ACTIVE_LEVEL; }

static void safe_shutdown(uint8_t flag)
{
  faults |= flag;
  pending_disable = false;
  dac_tgt = 0;
  dac_cur = 0;
  dac_write(0);
  set_relay(RELAY_OFF_LEVEL);
  set_hv(false);
}

// Lower the target by 'by' units BELOW what is already being output.  (Subtracting from the
// target alone would keep ramping up while the output is still below an old, higher target.)
static void lower_target(uint16_t by)
{
  float base = (dac_cur < dac_tgt) ? dac_cur : dac_tgt;
  float t = base - (float)by;
  dac_tgt = (t > 0.0f) ? t : 0.0f;
}

// ---------------------------------------------------------------------------
// Periodic work
// ---------------------------------------------------------------------------
static uint8_t recent_events(uint32_t now)
{
  uint8_t n = 0;
  uint32_t win = (uint32_t)cfg.spark_win_s * 1000UL;
  for (uint8_t i = 0; i < ev_cnt; i++) if (now - ev_ts[i] <= win) n++;
  return n;
}

static void push_event(uint32_t now)
{
  if (ev_cnt < 16) ev_ts[ev_cnt++] = now;
  else { for (uint8_t i = 1; i < 16; i++) ev_ts[i - 1] = ev_ts[i]; ev_ts[15] = now; }
}

static void sample_adc_and_stats(uint32_t now)
{
  for (uint8_t ch = 0; ch < NUM_CH; ch++) adc[ch] = adc_read(ch);

  // ---- spark / current-event statistics (CH1 = current monitor) ----
  uint16_t i = adc[CH_IMON];
  if (!ibase_init) { ibase = i; ibase_init = true; }
  if (i > imax_win) imax_win = i;
  if (adc[CH_VMON] < vmin_win) vmin_win = adc[CH_VMON];

  float dev = (float)i - ibase;
  bool  active = (hv_en && dac_raw > 0);

  // LOW threshold: count only (micro-discharges, corona bursts, ...).
  if (!in_event && dev > (float)cfg.ethr) {
    in_event = true;
    if (active) { ev_total++; ev_win++; }
  } else if (in_event && dev < (float)cfg.ethr * 0.5f) {
    in_event = false;
  }

  // HIGH threshold: a spark.  Counted, and (with "spark N") it triggers the back-off.
  if (!in_spark && dev > (float)cfg.sthr) {
    in_spark = true;
    if (active) {
      sp_total++; sp_win++;
      push_event(now);
      if (cfg.spark_n > 0 && recent_events(now) >= (cfg.spark_n > 16 ? 16 : cfg.spark_n)) {
        ev_cnt = 0;
        if (cfg.backoff > 0) {
          lower_target(cfg.backoff);
          spark_backoffs++;
        } else {
          faults |= F_SPARK;
          dac_tgt = 0;
          pending_disable = true;
          set_relay(RELAY_OFF_LEVEL);
        }
      }
    }
  } else if (in_spark && dev < (float)cfg.sthr * 0.5f) {
    in_spark = false;
  }

  if (!in_event && !in_spark) ibase += dev * (1.0f / 64.0f);
}

// ---------------------------------------------------------------------------
// X-ray guard
// ---------------------------------------------------------------------------
static void x_alarm(uint32_t now)
{
  x_hold = true;
  if (!hv_en || (dac_tgt <= 0 && dac_cur <= 0)) return;
  if (x_step_valid && (now - x_step_ms) < X_STEP_GAP_MS) return;
  x_step_ms = now; x_step_valid = true;
  x_trips++;
  if (cfg.xback > 0) {
    lower_target(cfg.xback);
  } else {
    faults |= F_XRAY;
    dac_tgt = 0;
    pending_disable = true;
    set_relay(RELAY_OFF_LEVEL);
  }
}

static void x_report(uint16_t level, uint32_t now)
{
  x_level = level; x_seen = true; x_last_ms = now;
  if (cfg.xthr == 0) { x_hold = false; return; }
  if (level >= cfg.xthr) x_alarm(now);
  else if ((uint32_t)level * 5UL < (uint32_t)cfg.xthr * 4UL) x_hold = false;     // hysteresis: below 80 %
}

static void check_xray_monitor(uint32_t now)
{
  if (cfg.xthr && cfg.xtimeout_s && hv_en && dac_raw > 0 && !(faults & F_XMON) &&
      (int32_t)(now - x_last_ms) > (int32_t)((uint32_t)cfg.xtimeout_s * 1000UL)) {
    faults |= F_XMON;
    dac_tgt = 0;
    pending_disable = true;
    set_relay(RELAY_OFF_LEVEL);
  }
}

static void check_interlocks(uint32_t now)
{
  // ---- ADC sanity: dead SPI / failed isolator shows up as all-0 or all-1 words ----
  uint8_t bad = 0;
  for (uint8_t ch = 0; ch < NUM_CH; ch++) if (adc[ch] <= 16 || adc[ch] >= 65520) bad++;
  if (bad >= 3) { if (adc_bad_cnt < 255) adc_bad_cnt++; } else adc_bad_cnt = 0;
  if (adc_bad_cnt >= 5 && !(faults & F_ADC)) safe_shutdown(F_ADC);

  // ---- J1 monitor cable: an unplugged v_mon / i_mon input floats to full scale (about +12.3 V) ----
  if (cfg.cable && hv_en && dac_raw > 0 && (adc[CH_VMON] >= CABLE_RAIL_ADC || adc[CH_IMON] >= CABLE_RAIL_ADC)) {
    if (cable_cnt < 255) cable_cnt++;
  } else cable_cnt = 0;
  if (cable_cnt >= CABLE_TRIP_CNT && !(faults & F_CABLE)) {
    faults |= F_CABLE;
    dac_tgt = 0;
    pending_disable = true;
    set_relay(RELAY_OFF_LEVEL);
  }

  // ---- voltage_program limit (stops integrator wind-up / DAC garbage from reaching the TOF3000) ----
  if (adc[CH_VPROG] > volts_to_adc(cfg.vprog_max_v)) { if (vprog_cnt < 255) vprog_cnt++; } else vprog_cnt = 0;
  if (vprog_cnt >= 3 && !(faults & F_VPROG)) safe_shutdown(F_VPROG);

  // ---- DAC read-back check (only meaningful while the integrator is off) ----
  uint16_t tol = (uint16_t)(cfg.vprog_tol_v * 65535.0f / ADC_FULLSCALE_V);
  if (integrator_active() || dac_raw != ver_code) {
    ver_code = dac_raw; ver_t0 = now; ref_ok = false; mism_cnt = 0;
  } else if (!ref_ok) {
    if (now - ver_t0 >= 500) { ref_val = adc[CH_VPROG]; ref_ok = true; }
  } else {
    long diff = (long)adc[CH_VPROG] - (long)ref_val;
    if (diff < 0) diff = -diff;
    if (diff > tol) {
      if (mism_cnt == 0) dac_fixes++;
      if (mism_cnt < 255) mism_cnt++;
      dac_write(dac_raw);                       // rewrite immediately
      if (mism_cnt >= 5 && !(faults & F_DACMISM)) safe_shutdown(F_DACMISM);
    } else {
      mism_cnt = 0;
    }
  }
}

// Polarity may only change after the supply output has decayed.  A floating v_mon input (cable off) counts as
// "nothing connected", so bench work with J1 open is not blocked.
static bool vmon_decayed()
{
  uint16_t r = adc[CH_VMON];
  if (r >= CABLE_RAIL_ADC) return true;
  return fabsf(adc_to_volts(r)) <= POL_VMON_MAX_V;
}

static void update_slew(uint32_t dt_ms)
{
  float dt = dt_ms / 1000.0f;
  if (dac_cur < dac_tgt) {
    dac_cur = (cfg.up_rate == 0) ? dac_tgt : dac_cur + cfg.up_rate * dt;
    if (dac_cur > dac_tgt) dac_cur = dac_tgt;
  } else if (dac_cur > dac_tgt) {
    dac_cur = (cfg.down_rate == 0) ? dac_tgt : dac_cur - cfg.down_rate * dt;
    if (dac_cur < dac_tgt) dac_cur = dac_tgt;
  }
  uint16_t raw = host_to_raw(dac_cur);
  if (raw != dac_raw) dac_write(raw);

  if (pending_disable && dac_raw == 0) {
    set_relay(RELAY_OFF_LEVEL);
    set_hv(false);
    pending_disable = false;
  }
}

static void check_host_timeout(uint32_t now)
{
  if (cfg.timeout_s && hv_en && !(faults & F_TIMEOUT) &&
      (int32_t)(now - last_host_ms) > (int32_t)((uint32_t)cfg.timeout_s * 1000UL)) {
    faults |= F_TIMEOUT;
    dac_tgt = 0;
    pending_disable = true;
    set_relay(RELAY_OFF_LEVEL);
  }
}

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------
static void print_status(uint32_t now)
{
  Serial.print(F("S up="));      Serial.print(now / 1000UL);
  Serial.print(F(" rst=0x"));    Serial.print(g_rstfr, HEX);
  Serial.print(F(" n_por="));    Serial.print(g_persist.n_por);
  Serial.print(F(" n_bor="));    Serial.print(g_persist.n_bor);
  Serial.print(F(" n_ext="));    Serial.print(g_persist.n_ext);
  Serial.print(F(" n_wdt="));    Serial.print(g_persist.n_wdt);
  Serial.print(F(" n_sw="));     Serial.print(g_persist.n_sw);
  Serial.print(F(" hv="));       Serial.print(hv_en ? 1 : 0);
  Serial.print(F(" pol="));      Serial.print(pol_level);
  Serial.print(F(" rly="));      Serial.print(relay_level);
  Serial.print(F(" int="));      Serial.print(integrator_active() ? 1 : 0);
  Serial.print(F(" tgt="));      Serial.print(dac_tgt, 0);
  Serial.print(F(" cur="));      Serial.print(dac_cur, 0);
  Serial.print(F(" raw="));      Serial.print(dac_raw);
  Serial.print(F(" flt=0x"));    Serial.print(faults, HEX);
  Serial.print(F(" ev="));       Serial.print(ev_total);
  Serial.print(F(" sp="));       Serial.print(sp_total);
  Serial.print(F(" backoffs=")); Serial.print(spark_backoffs);
  Serial.print(F(" xr="));       Serial.print(x_level);
  Serial.print(F(" xh="));       Serial.print(x_hold ? 1 : 0);
  Serial.print(F(" xtrips="));   Serial.print(x_trips);
  Serial.print(F(" dacfix="));   Serial.println(dac_fixes);
}

static void print_config()
{
  Serial.print(F("CFG board="));     Serial.print(cfg.board);
  Serial.print(F(" scale="));        Serial.print(cfg.scale, 4);
  Serial.print(F(" dacmax="));       Serial.print(cfg.dac_max);
  Serial.print(F(" up="));           Serial.print(cfg.up_rate);
  Serial.print(F(" down="));         Serial.print(cfg.down_rate);
  Serial.print(F(" timeout="));      Serial.print(cfg.timeout_s);
  Serial.print(F(" vprogmax="));     Serial.print(cfg.vprog_max_v, 2);
  Serial.print(F(" vprogtol="));     Serial.print(cfg.vprog_tol_v, 2);
  Serial.print(F(" ethr="));         Serial.print(cfg.ethr);
  Serial.print(F(" sthr="));         Serial.print(cfg.sthr);
  Serial.print(F(" spark="));        Serial.print(cfg.spark_n);
  Serial.print(F(" sparkwin="));     Serial.print(cfg.spark_win_s);
  Serial.print(F(" backoff="));      Serial.print(cfg.backoff);
  Serial.print(F(" xthr="));         Serial.print(cfg.xthr);
  Serial.print(F(" xback="));        Serial.print(cfg.xback);
  Serial.print(F(" xtimeout="));     Serial.print(cfg.xtimeout_s);
  Serial.print(F(" pid="));          Serial.print(cfg.pid);
  Serial.print(F(" cable="));        Serial.print(cfg.cable);
  Serial.print(F(" pol="));          Serial.print(cfg.pol);
  Serial.print(F(" name="));         Serial.print(cfg.name[0] ? cfg.name : "-");
  Serial.print(F(" strict="));       Serial.print(cfg.strict);
  Serial.print(F(" ack="));          Serial.println(cfg.ack);
}

static void handle_config(char *arg)
{
  if (!*arg) { print_config(); return; }

  char *val = arg;
  while (*val && *val != ' ') val++;
  if (*val) { *val++ = 0; while (*val == ' ') val++; }

  if (!strcmp(arg, "save"))     { cfg_save(); reply_ok(); return; }
  if (!strcmp(arg, "load"))     { if (cfg_load()) reply_ok(); else reply_err(F("EEPROM")); return; }
  if (!strcmp(arg, "defaults")) {
    if (hv_en || dac_raw != 0) { reply_err(F("BUSY")); return; }
    cfg_defaults(cfg.board); reply_ok(); return;
  }

  long  n = 0;
  float f = 0;
  bool  is_cal = (!strcmp(arg, "board") || !strcmp(arg, "scale") || !strcmp(arg, "dacmax") || !strcmp(arg, "pol"));
  if (is_cal && (hv_en || dac_raw != 0)) { reply_err(F("BUSY")); return; }

  if (!strcmp(arg, "board")) {
    if ((val[0] == 'A' || val[0] == 'B') && !val[1]) {
      cfg.board = val[0];
      cfg.scale = (val[0] == 'A') ? 0.961f : 0.726f;
      reply_ok();
    } else reply_err(F("ARG"));
  }
  else if (!strcmp(arg, "pol")) {
    // Polarity applied at every boot AND now (HV is off and DAC = 0 here, checked above).
    if (parse_long(val, n) && (n == 0 || n == 1)) {
      if (!vmon_decayed()) { reply_err(F("VMON")); return; }
      cfg.pol = (uint8_t)n; set_pol(n ? HIGH : LOW); reply_ok();
    }
    else reply_err(F("ARG"));
  }
  else if (!strcmp(arg, "pid"))       { if (parse_long(val, n) && (n == 0 || n == 1)) { cfg.pid = (uint8_t)n; if (!n) set_relay(RELAY_OFF_LEVEL); reply_ok(); } else reply_err(F("ARG")); }
  else if (!strcmp(arg, "cable"))     { if (parse_long(val, n) && (n == 0 || n == 1)) { cfg.cable = (uint8_t)n; reply_ok(); } else reply_err(F("ARG")); }
  else if (!strcmp(arg, "name")) {
    // 1..8 characters, letters digits + - _ ; "-" alone clears it.
    size_t L = strlen(val);
    bool ok = (L >= 1 && L <= sizeof(cfg.name) - 1);
    for (size_t i = 0; ok && i < L; i++) {
      char ch = val[i];
      ok = (ch >= '0' && ch <= '9') || (ch >= 'A' && ch <= 'Z') || (ch >= 'a' && ch <= 'z') || ch == '+' || ch == '-' || ch == '_';
    }
    if (ok) {
      memset(cfg.name, 0, sizeof(cfg.name));
      if (!(L == 1 && val[0] == '-')) memcpy(cfg.name, val, L);
      reply_ok();
    } else reply_err(F("ARG"));
  }
  else if (!strcmp(arg, "scale"))     { if (parse_float(val, f) && f > 0.05f && f < 3.0f) { cfg.scale = f; reply_ok(); } else reply_err(F("ARG")); }
  else if (!strcmp(arg, "vprogmax"))  { if (parse_float(val, f) && f > 1.0f && f < 12.0f) { cfg.vprog_max_v = f; reply_ok(); } else reply_err(F("ARG")); }
  else if (!strcmp(arg, "vprogtol"))  { if (parse_float(val, f) && f >= 0.01f && f < 2.0f) { cfg.vprog_tol_v = f; reply_ok(); } else reply_err(F("ARG")); }
  else if (!strcmp(arg, "dacmax"))    { if (parse_long(val, n) && n >= 0 && n <= 65535) { cfg.dac_max = (uint16_t)n; reply_ok(); } else reply_err(F("ARG")); }
  else if (!strcmp(arg, "up"))        { if (parse_long(val, n) && n >= 0 && n <= 65535) { cfg.up_rate = (uint16_t)n; reply_ok(); } else reply_err(F("ARG")); }
  else if (!strcmp(arg, "down"))      { if (parse_long(val, n) && n >= 0 && n <= 65535) { cfg.down_rate = (uint16_t)n; reply_ok(); } else reply_err(F("ARG")); }
  else if (!strcmp(arg, "timeout"))   { if (parse_long(val, n) && n >= 0 && n <= 65535) { cfg.timeout_s = (uint16_t)n; reply_ok(); } else reply_err(F("ARG")); }
  else if (!strcmp(arg, "ethr"))      { if (parse_long(val, n) && n >= 1 && n <= 30000) { cfg.ethr = (uint16_t)n; reply_ok(); } else reply_err(F("ARG")); }
  else if (!strcmp(arg, "sthr"))      { if (parse_long(val, n) && n >= 1 && n <= 30000) { cfg.sthr = (uint16_t)n; reply_ok(); } else reply_err(F("ARG")); }
  else if (!strcmp(arg, "xthr"))      { if (parse_long(val, n) && n >= 0 && n <= 65535) { cfg.xthr = (uint16_t)n; reply_ok(); } else reply_err(F("ARG")); }
  else if (!strcmp(arg, "xback"))     { if (parse_long(val, n) && n >= 0 && n <= 65535) { cfg.xback = (uint16_t)n; reply_ok(); } else reply_err(F("ARG")); }
  else if (!strcmp(arg, "xtimeout"))  { if (parse_long(val, n) && n >= 0 && n <= 65535) { cfg.xtimeout_s = (uint16_t)n; reply_ok(); } else reply_err(F("ARG")); }
  else if (!strcmp(arg, "spark"))     { if (parse_long(val, n) && n >= 0 && n <= 16) { cfg.spark_n = (uint8_t)n; reply_ok(); } else reply_err(F("ARG")); }
  else if (!strcmp(arg, "sparkwin"))  { if (parse_long(val, n) && n >= 1 && n <= 3600) { cfg.spark_win_s = (uint16_t)n; reply_ok(); } else reply_err(F("ARG")); }
  else if (!strcmp(arg, "backoff"))   { if (parse_long(val, n) && n >= 0 && n <= 65535) { cfg.backoff = (uint16_t)n; reply_ok(); } else reply_err(F("ARG")); }
  else if (!strcmp(arg, "strict"))    { if (parse_long(val, n) && (n == 0 || n == 1)) { cfg.strict = (uint8_t)n; reply_ok(); } else reply_err(F("ARG")); }
  else if (!strcmp(arg, "ack"))       { if (parse_long(val, n) && (n == 0 || n == 1)) { cfg.ack = (uint8_t)n; reply_ok(); } else reply_err(F("ARG")); }
  else reply_err(F("KEY"));
}

static void process_line(char *ln)
{
  uint32_t now = millis();

  // ---- optional checksum: "<text>*HH", HH = XOR of all characters before '*' ----
  char *star = strchr(ln, '*');
  if (star) {
    if (strlen(star + 1) != 2) { reply_err(F("CHK")); return; }
    uint8_t x = 0;
    for (char *p = ln; p < star; p++) x ^= (uint8_t)*p;
    char *e;
    long v = strtol(star + 1, &e, 16);
    if (*e || (uint8_t)v != x) { reply_err(F("CHK")); return; }
    *star = 0;
  } else if (cfg.strict) {
    reply_err(F("CHK"));
    return;
  }

  while (*ln == ' ') ln++;
  if (!*ln) return;
  char  cmd = *ln;
  char *arg = ln + 1;
  while (*arg == ' ') arg++;

  if (cmd == 'h' || cmd == 'H') { reply_err(F("USE_E")); return; }     // v1 raw-pin command, removed: h0 meant ON there

#ifdef HV_BENCH_SIM
  if (!strchr("?abdDzeEpPrRsmCtkcxXYNVQWU", cmd)) { reply_err(F("CMD")); return; }
#else
  if (!strchr("?abdDzeEpPrRsmCtkcx", cmd)) { reply_err(F("CMD")); return; }
#endif
  last_host_ms = now;                      // any recognised command counts as a heartbeat

  long n = 0;
  switch (cmd) {
    case '?':
      Serial.print(F("HV_control v1.3 ready. Board ")); Serial.print(cfg.board);
      Serial.print(F(". FW " FW_VERSION));
      Serial.print(F(". Name ")); Serial.print(cfg.name[0] ? cfg.name : "-");
      Serial.print(F(". Pol "));  Serial.println(pol_level == HIGH ? '+' : '-');
      break;

    case 'a':
      for (uint8_t i = 0; i < NUM_CH; i++) {
        Serial.print(adc[i]);
        if (i < NUM_CH - 1) Serial.print(','); else Serial.println();
      }
      break;

    case 'b':
      for (uint8_t i = 0; i < NUM_CH; i++) {
        Serial.print(adc_to_volts(adc[i]), 4);
        if (i < NUM_CH - 1) Serial.print(','); else Serial.println();
      }
      break;

    case 'd':
      if (!parse_long(arg, n)) { reply_err(F("ARG")); break; }
      if (n < 0 || n > cfg.dac_max) { reply_err(F("RANGE")); break; }
      if (n > 0 && faults) { reply_err(F("FAULT")); break; }
      if (x_hold && cfg.xthr && (float)n > dac_tgt) { reply_err(F("XRAY")); break; }     // alarm: no increases
      dac_tgt = (float)n;
      reply_ok();
      break;

    case 'D':
      Serial.print(dac_tgt, 0); Serial.print(','); Serial.print(dac_cur, 0); Serial.print(','); Serial.println(dac_raw);
      break;

    case 'z':
      pending_disable = false;
      dac_tgt = 0; dac_cur = 0;
      dac_write(0);
      set_relay(RELAY_OFF_LEVEL);
      set_hv(false);
      reply_ok();
      break;

    case 'e':
      if (!parse_long(arg, n) || (n != 0 && n != 1)) { reply_err(F("ARG")); break; }
      if (n == 1) {
        if (faults)       { reply_err(F("FAULT")); break; }
        if (dac_raw != 0) { reply_err(F("DAC_NZ")); break; }
        pending_disable = false;
        x_last_ms = now;                     // the X-ray monitor gets xtimeout seconds to report
        set_hv(true);
      } else {
        set_relay(RELAY_OFF_LEVEL);
        set_hv(false);
        pending_disable = false;
        dac_tgt = 0; dac_cur = 0;            // HV is off: program voltage must not keep ramping
        dac_write(0);
      }
      reply_ok();
      break;

    case 'E':
      Serial.println(hv_en ? 1 : 0);
      break;

    case 'p':
      if (!parse_long(arg, n) || (n != 0 && n != 1)) { reply_err(F("ARG")); break; }
      if (hv_en)        { reply_err(F("HV_ON")); break; }
      if (dac_raw != 0) { reply_err(F("DAC_NZ")); break; }
      if (!vmon_decayed()) { reply_err(F("VMON")); break; }      // supply output has not decayed yet
      set_pol(n ? HIGH : LOW);
      reply_ok();
      break;

    case 'P':
      Serial.println(pol_level == HIGH ? 1 : 0);
      break;

    case 'r':
      if (!parse_long(arg, n) || (n != 0 && n != 1)) { reply_err(F("ARG")); break; }
      if ((n ? HIGH : LOW) == RELAY_ACTIVE_LEVEL && !cfg.pid) { reply_err(F("PIDOFF")); break; }   // "c pid 1" unlocks it
      if ((n ? HIGH : LOW) == RELAY_ACTIVE_LEVEL && (!hv_en || faults)) { reply_err(F("INTERLOCK")); break; }
      // Measured on the real board (J1 open, error = -3 V): engaging the integrator slams v_prog to the
      // supply rail (>12 V) within ~60 ms.  Only allow it when the HV divider is already near the 3 V reference.
      if ((n ? HIGH : LOW) == RELAY_ACTIVE_LEVEL && fabsf(adc_to_volts(adc[CH_VERR])) > ERR_ENGAGE_MAX_V) {
        reply_err(F("ERRBIG")); break;
      }
      set_relay(n ? HIGH : LOW);
      reply_ok();
      break;

    case 'R':
      Serial.println(relay_level == HIGH ? 1 : 0);
      break;

    case 's':
      print_status(now);
      break;

    case 'm':
      Serial.print(F("M ev="));      Serial.print(ev_win);
      Serial.print(F(" sp="));       Serial.print(sp_win);
      Serial.print(F(" imax="));     Serial.print(imax_win);
      Serial.print(F(" vmin="));     Serial.print(vmin_win);
      Serial.print(F(" ibase="));    Serial.print((uint16_t)ibase);
      Serial.print(F(" evtotal="));  Serial.print(ev_total);
      Serial.print(F(" sptotal="));  Serial.println(sp_total);
      ev_win = 0; sp_win = 0; imax_win = 0; vmin_win = 0xFFFF;
      break;

    case 'C':
      faults = 0;
      pending_disable = false;
      ev_cnt = 0;
      adc_bad_cnt = 0; vprog_cnt = 0; mism_cnt = 0; cable_cnt = 0;
      ver_code = 0xFFFF;                   // force a fresh read-back reference after clearing faults
      x_last_ms = now;                     // do not re-trip the X-ray monitor-lost check immediately
      reply_ok();
      break;

    case 'x':
      if (!parse_long(arg, n) || n < 0 || n > 65535) { reply_err(F("ARG")); break; }
      x_report((uint16_t)n, now);
      reply_ok();
      break;

    case 't':
      if (!parse_long(arg, n) || n < 0 || n > 65535) { reply_err(F("ARG")); break; }
      cfg.timeout_s = (uint16_t)n;
      reply_ok();
      break;

    case 'k':
      reply_ok();
      break;

    case 'c':
      handle_config(arg);
      break;

#ifdef HV_BENCH_SIM
    case 'X': if (parse_long(arg, n) && n > 0) { sim_spark_until  = now + (uint32_t)n; reply_ok(); } else reply_err(F("ARG")); break;
    case 'Y': if (parse_long(arg, n) && n > 0) { sim_glitch_until = now + (uint32_t)n; reply_ok(); } else reply_err(F("ARG")); break;
    case 'N': if (parse_long(arg, n) && n > 0) { sim_stuck_until  = now + (uint32_t)n; reply_ok(); } else reply_err(F("ARG")); break;
    case 'V': if (parse_long(arg, n) && n >= 0 && n <= 30000) { sim_vprog_add = (uint16_t)n; reply_ok(); } else reply_err(F("ARG")); break;
    case 'W': if (parse_long(arg, n) && n > 0) { sim_rail_until = now + (uint32_t)n; reply_ok(); } else reply_err(F("ARG")); break;
    case 'U': if (parse_long(arg, n) && n >= 0 && n <= 30000) { sim_vmon_add = (uint16_t)n; reply_ok(); } else reply_err(F("ARG")); break;
    case 'Q': Serial.println(F("hanging")); Serial.flush(); while (true) { } break;
#endif
  }
}

// ---------------------------------------------------------------------------
// Serial line reader (non-blocking)
// ---------------------------------------------------------------------------
const uint8_t LINE_BUF_SIZE = 64;
static char     line_buf[LINE_BUF_SIZE];
static uint8_t  line_len = 0;
static uint8_t  line_err = 0;            // 0 ok, 1 bad character, 2 too long
static uint32_t line_t0 = 0;

static void serial_poll()
{
  while (Serial.available()) {
    int c = Serial.read();
    if (c < 0) break;

    if (c == '\n' || c == '\r') {
      if (line_err == 1)      reply_err(F("CHR"));
      else if (line_err == 2) reply_err(F("LEN"));
      else if (line_len > 0) { line_buf[line_len] = 0; process_line(line_buf); }
      line_len = 0; line_err = 0;
      continue;
    }

    if (line_len == 0 && line_err == 0) line_t0 = millis();
    if (c < 0x20 || c > 0x7E) { line_err = 1; continue; }
    if (line_len < LINE_BUF_SIZE - 1) line_buf[line_len++] = (char)c;
    else line_err = 2;
  }

  if ((line_len || line_err) && (millis() - line_t0) > LINE_TIMEOUT_MS) {   // stale partial line (noise)
    line_len = 0; line_err = 0;
  }
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
void setup()
{
  // 1) Safe output levels first.
  digitalWrite(HV_enable, HV_ENABLE_OFF_LEVEL); pinMode(HV_enable, OUTPUT); digitalWrite(HV_enable, HV_ENABLE_OFF_LEVEL);
  digitalWrite(CS_DAC, HIGH);                pinMode(CS_DAC, OUTPUT);      digitalWrite(CS_DAC, HIGH);
  digitalWrite(CS_ADC, HIGH);                pinMode(CS_ADC, OUTPUT);      digitalWrite(CS_ADC, HIGH);
  digitalWrite(relay_ctl, RELAY_OFF_LEVEL);  pinMode(relay_ctl, OUTPUT);   digitalWrite(relay_ctl, RELAY_OFF_LEVEL);
  hv_en = false;
  relay_level = RELAY_OFF_LEVEL;

  Serial.begin(9600);
  SPI.begin();

  // 2) Configuration.
  if (!cfg_load()) cfg_defaults(DEFAULT_BOARD);

  // 3) Polarity from the saved config ("c pol"), HV is still disabled.
  //    Defaults follow v1 (board A = HIGH, board B = LOW) until "c pol" + "c save" is done.
  pinMode(polarity_control, OUTPUT);
  set_pol(cfg.pol ? HIGH : LOW);

  // 4) DAC to zero (do not trust whatever it held before this reset).
  dac_write(0);

  // 5) Reset-cause bookkeeping.
  if (g_persist.magic != PERSIST_MAGIC) { memset(&g_persist, 0, sizeof(g_persist)); g_persist.magic = PERSIST_MAGIC; }
  if (g_rstfr & RSTCTRL_PORF_bm)  g_persist.n_por++;
  if (g_rstfr & RSTCTRL_BORF_bm)  g_persist.n_bor++;
  if (g_rstfr & RSTCTRL_EXTRF_bm) g_persist.n_ext++;
  if (g_rstfr & RSTCTRL_WDRF_bm)  g_persist.n_wdt++;
  if (g_rstfr & RSTCTRL_SWRF_bm)  g_persist.n_sw++;

  // 6) Prime the ADC values so "a" is meaningful right away.
  for (uint8_t ch = 0; ch < NUM_CH; ch++) adc[ch] = adc_read(ch);

  uint32_t now = millis();
  t_adc = t_slew = t_refresh = last_host_ms = x_last_ms = now;

  // 7) Hardware watchdog, about 2 s (1 kHz ULP clock, 2K cycles).
  _PROTECTED_WRITE(WDT.CTRLA, WDT_PERIOD_2KCLK_gc);
}

void loop()
{
  wdt_kick();

  // Serial first, clock second: process_line() stamps last_host_ms / x_last_ms with its own millis(), which must
  // never be newer than this loop's 'now' (FW 2.7 and older read 'now' first: a 1 ms tick in between made
  // now - last_host_ms underflow to ~49 days and tripped the heartbeat / X-ray-monitor timeouts at random).
  serial_poll();
  uint32_t now = millis();

  if (now - t_adc >= ADC_PERIOD_MS) {
    t_adc = now;
    sample_adc_and_stats(now);
    check_interlocks(now);
  }

  if (now - t_slew >= SLEW_PERIOD_MS) {
    uint32_t dt = now - t_slew;
    t_slew = now;
    update_slew(dt);
  }

  if (now - t_refresh >= DAC_REFRESH_MS) {
    t_refresh = now;
    dac_write(dac_raw);
  }

  check_host_timeout(now);
  check_xray_monitor(now);
}
