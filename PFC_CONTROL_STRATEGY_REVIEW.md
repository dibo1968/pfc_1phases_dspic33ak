# PFC Control Strategy Review — 2026-08-02

Scope: the control strategy and its implementation in `project/pfc/` plus the
control-relevant parts of `project/hal/` (PWM, ADC, board service) and the
design script `SimulinkProject/mchp_pfc_foc_dsPIC33A_data.m`.

This complements — does not repeat — `PFC_CODE_REVIEW.md` (2026-07-23 correctness
review; its findings are annotated fixed in this tree) and `PFC_CONTROL_THEORY.md`
(the derivation document). Everything below was found against the current
`claude/dcm-mcm-wip` working tree.

Severity legend: **[CRIT]** can damage hardware or prevent operation ·
**[HIGH]** wrong behaviour in a realistic scenario · **[MED]** degrades
performance or robustness · **[LOW]** hygiene / polish.

---

## 1. What is sound (keep as-is)

For balance, the parts of the strategy that review well:

- **Mode-dependent duty feed-forward** (`PFC_DutyFeedForward`, pfc.c:828) — the
  CCM/DCM branches meet exactly at the boundary, the Vg floor at the zero
  crossing is the right call (falling back to the CCM branch there would
  command near-max duty), and the measured result (THD 31 % → 1.5 %) confirms
  the velocity-error diagnosis.
- **Valley-estimation conduction-mode detect** (`PFC_ConductionModeDetect`,
  pfc.c:740) — comparing a predicted valley against a *fixed* zero threshold
  instead of a clamp-derived ratio removes boundary chatter by construction.
  Matches Nair & Narasamma (JESTPE 2020) eq. (4)/(6).
- **Vdc notch + anti-noise pole** (`PFC_VdcFilterUpdate`, pfc.c:249) — running
  the biquad at the decimated rate to avoid float32 pole-radius cancellation is
  a subtle and correct decision; the filter-reset-to-DC-fixed-point trick is
  valid because DC gain is exactly 1.
- **Load-current feed-forward with gain < 1** (userparams:197-217) — the
  analysis of why gain = 1.0 destroys the bus's self-regulation term is correct
  and SiL-verified; burst control testing `powerCommand` rather than
  `piVoltage.output` closes the 11.4 Hz relaxation-oscillation trap.
- **Back-calculation anti-windup on the summed duty** (pfc.c:929-938).
- **Precharge referred to a fraction of the measured input peak**
  (userparams:341-360) — works across the declared input range and minimises
  relay surge.
- **The `.m` ↔ header cross-check table** (mchp_pfc_foc_dsPIC33A_data.m:168) —
  cheap drift protection; the spurious ×2 in the voltage plant is now corrected
  and annotated.

---

## 2. Actual bugs (confirmed in code)

> **Status 2026-08-02: all four §2 items are FIXED** (same day, on
> `claude/dcm-mcm-wip`). Each header below carries its resolution; the original
> analyses are retained. Host-compiled clean (gcc + SiL stubs, `-Wall -Wextra`),
> MEX rebuilt, and **verified in a full SiL run** (profile 1: precharge from
> 0 V, 1.5 kW step at t = 0.8 s, 1.2 s; results in
> `SimulinkProject/sil_run_bugfix.mat`, runner `run_sil_scenario.m`):
> precharge completes at 0.25 s (timeout untripped), WAIT_1CYCLE dwells exactly
> 10.00 ms and first PWM comes 37.3 ms after the relay-close command (§2.1),
> `faultStatus` = 0 for the whole run, relay-close surge 5.7 A, load-step dip
> 10.1 V (369.9 V min vs the 310 V OP_UV trip), final bus 380.0 V ± 4.5 V
> ripple, current-PI trim within [−0.05, +0.04] of its ±0.25 bound, input
> current THD 0.35 % / PF 1.000 at 1.5 kW. The only |iL| > 16.97 A excursions
> are the passive t < 15 ms inrush with PWM off, where OCP is deliberately not
> armed. The 2.2 runtime-toggle path is covered by review + compile only — a
> mid-sim toggle needs an S-function input port or a debugger write on HW.

### 2.1 [HIGH] `PFC_WAIT_1CYCLE` never waits — sticky `status` flag  **[FIXED 2026-08-02]**

> **Resolution.** `PFC_StateOffsetMeas` now restarts the RMS window
> (`sum`/`samples`/`status` cleared) on the transition out, so
> `PFC_WAIT_1CYCLE` waits one complete fresh window — a deterministic 10 ms —
> before enabling PWM. Total delay from relay-close command to first PWM pulse
> is now ~26 ms (16 ms offset window + 10 ms RMS window), covering typical
> relay operate + bounce. Restarting the accumulator mid-window costs no
> accuracy (vac² has period T/2, so any starting phase integrates to the same
> RMS). An explicit `PFC_RELAY_SETTLE` state (§6.2) remains a nice-to-have.

`PFC_StateWait1Cycle` (pfc.c:446) gates on `vacRMS.status == 1`, but
`PFC_SquaredRMSCalculate` (pfc.c:1067) sets `status` at the first completed
window and **never clears it**. By the time `PFC_OFFSET_MEAS` finishes, the
flag has been 1 for tens of milliseconds, so the state falls straight through
in one ISR tick.

Consequence: PWM is enabled ~16 ms after the inrush-relay close *command*
(the offset-measurement window is the only real delay). Relay operate time is
typically 5–15 ms plus contact bounce, so PWM can start switching while the
relay is still bouncing — boosting through the inrush resistor, or worse,
chattering the bus across bounce events. The state exists precisely to prevent
this and currently does nothing.

Fix: clear `vacRMS.status = 0` on entry to the state (or count a fresh
`PFC_RMS_SQUARE_COUNTMAX` of ISRs explicitly). Better: see §6.2, add an
explicit relay-settle delay.

### 2.2 [HIGH] Runtime toggle of `dutyFFEnable` leaves stale current-PI limits  **[FIXED 2026-08-02]**

> **Resolution.** The limit selection moved into a helper
> (`PFC_CurrentPILimitsUpdate`), called from `PFC_ParamsInit` and again from
> `PFC_CurrentControlLoop` whenever `dutyFFEnable` differs from the new
> `PFC_T.dutyFFEnablePrev` (appended field — SiL-mirror-safe). The toggle is
> also made approximately bumpless: on 1→0 the integrator absorbs last cycle's
> `dutyFF`; on 0→1 it sheds `boostDutyRatio` (estimate; the trim clamp and
> back-calculation absorb the DCM residual within a few cycles).

The header and comments advertise `pfcParam.dutyFFEnable` as run-time writable
"so both can be compared on a single build" (pfc.h:228-231, userparams:258-260).
But the current-PI output limits that depend on it are set **once** in
`PFC_ParamsInit` (pfc.c:588-600):

- Toggling 1 → 0 at run time: `dutyFF` becomes 0 (pfc.c:920) but the PI stays
  clamped to ±`PFC_DUTY_TRIM_MAX` = ±0.25. The loop can no longer reach the
  ~0.15–0.95 duty the boost needs; the bus collapses toward the rectified
  peak, the voltage PI winds to its clamp, and OP_UV eventually trips. The
  advertised A/B experiment takes the converter down.
- Toggling 0 → 1 on a build whose default is 0: the PI keeps limits
  [0, 0.95], so the "trim" cannot go negative and windup protection is lost.

Fix: re-derive the limits inside `PFC_CurrentControlLoop` from the live
`dutyFFEnable` value (two assignments), or provide a setter that updates both.

### 2.3 [MED] `PFC_ControllerPIUpdate` still slams the integrator to the clamp  **[FIXED 2026-08-02]**

> **Resolution.** The PI update now clamps the integral state *alone* to
> [min, max] (integrator clamping) and clamps the output separately — the
> integrator is never assigned the P term's excursion. This is the §6.3
> "correct mechanism inside the PI module"; the outer back-calculation on the
> summed duty in `PFC_CurrentControlLoop` stays, as recommended. Behaviour
> change to expect in SiL regressions: after saturating transients the loops
> recover without the old Ki-rate bleed-off tail.

pfc_pi.c:79-88 assigns `integralOut = maxOutput` (or `minOutput`) whenever the
*sum* P+I saturates. The comment at pfc.c:924-928 identifies exactly this as
wrong — but only the outer duty-sum clamp got back-calculation; the shared PI
update itself still does it, and it runs for **both** loops:

- Current PI (trim mode, limits ±0.25): a transient error of ~7 A makes the P
  term alone hit the clamp, and the integrator — which may have been near zero
  — is *assigned* ±0.25. When the error collapses the stored trim is wrong by
  up to the full trim range and bleeds off only at Ki rate → duty disturbance
  right after every large transient (zero-crossing region in DCM is where
  large relative errors occur).
- Voltage PI (limits 0..1500): on a deep sag/recovery the integrator can be
  slammed to 1500 W by the P term, then take seconds (Ki = 0.0992 at 5.33 kHz)
  to unwind — an overshoot mechanism the loop does not need to have.

Fix: in `PFC_ControllerPIUpdate`, clamp the *output* but leave `integralOut`
untouched when the P term caused the excursion — conditional integration
(`if (U saturated && error drives further into saturation) skip integration`)
or back-calculation with a tracking gain. The outer sum clamp in
`PFC_CurrentControlLoop` already does the right thing and can stay.

### 2.4 [HIGH] Precharge has no timeout — inrush resistor can cook  **[FIXED 2026-08-02]**

> **Resolution.** `PFC_StatePrecharge` counts its dwell
> (`PFC_T.prechargeCount`, appended field) and latches a new
> `PFC_FAULT_PRECHG` (bit 5, never auto-cleared — same latch class as IP_OC)
> after `PFC_PRECHARGE_TIMEOUT_SEC` = 1.0 s (userparams, sized ≥ 5·R·C at low
> line + the 20 ms window gating). The relay is deliberately left open on the
> way out: closing it into a half-charged bus is the surge precharge exists to
> avoid. Plateau-detect completion (the refinement) remains open under §6.2.
> Simulink/X2C fault decoders must bit-test, not compare equal — bit 5 is new.

`PFC_StatePrecharge` (pfc.c:390) waits for `vdcAVG ≥ 0.97 × peak` with no exit
path. With any standing load on the DC bus — and this application's load is a
pulse load that may be present at power-up — the passive R-C divider settles
*below* 97 % of the peak and the state never completes. Inrush resistors are
short-time rated; minutes at even a few watts is a fire-risk review item, and
the converter presents as silently dead.

Fix: add a precharge timeout (a few hundred ms at nominal line) that either
trips a latched fault, or completes on plateau detection (|dVdc/dt| below a
threshold for N windows) with the timeout as backstop. Note the same fraction
threshold also interacts with §3.6 (OP_UV recovery).

---

## 3. Latent bugs and fragile constructs

### 3.1 [CRIT-verify] Center-aligned PWM with a full-period PG4PER — the real switching frequency may be 32 kHz, not 64 kHz

`PG4CONbits.MODSEL = 4` selects Center-Aligned mode (pwm.c:168) and
`PG4PER = PFC_LOOPTIME_TCY` (pwm.c:425), where `PFC_LOOPTIME_TCY` encodes the
**full** 15.625 µs period. On Microchip's high-resolution PWM (dsPIC33CK
family manual, same IP generation), center-aligned mode counts up to PGxPER
and back down: the effective period is **2 × PGxPER**, which is why Microchip's
own center-aligned motor examples set `LOOPTIME_TCY = FCY/FPWM/2`. If that
convention holds on the dsPIC33AK, the converter actually switches at 32 kHz
and the ISR runs at 32 kHz.

The trap: the duty *ratio* is unaffected (ON time and period scale together),
so the converter would still regulate and nothing obviously fails. But every
absolute-time constant would be 2× wrong:

- `PFC_TS_OVER_L` / `PFC_TWO_L_OVER_TS` → DCM detection boundary and DCM duty
  feed-forward both off by 2× (the flagship features of this branch);
- the notch sample rate → the null lands at **50 Hz instead of 100 Hz**, so
  the notch stops protecting the voltage loop *and* attenuates the wrong line;
- voltage-loop rate 2.67 kHz, integral zero and crossover shifted;
- vacRMS/vdcAVG windows become full-line-cycle (benign);
- X2C-Scope sample time 125 µs, not the assumed 62.5 µs.

Verification is one scope probe on PWM4H (or a pin toggle in the ISR). If it
is 32 kHz, either halve `PG4PER`/`PG4DC` scaling or switch MODSEL and re-check
the mid-ON trigger placement (`PG4TRIGA`, CAHALF=1) — the mid-ON sampling
property depends on center-aligned geometry and must be preserved.

### 3.2 [MED] Current-offset measurement runs while current flows

`PFC_OFFSET_MEAS` starts immediately after the relay closes (pfc.c:413), while
the bus is still topping up from ~97 % of peak — charge pulses flow through
the *inductor and shunt* at every line peak. The 1024-sample (16 ms) window is
also not an integer number of half-cycles, so the pulse contribution does not
even average out symmetrically. Today this is moot because
`ENABLE_PFC_CURRENT_OFFSET_CORRECTION` is compiled out (userparams:81), but
the state machine still spends 16 ms measuring an offset nothing consumes, and
the moment offset correction is enabled on hardware (§4.3) the measured offset
is biased.

Fix: gate offset accumulation on non-conduction (e.g. sample only in a window
around the line zero crossing), or measure before the relay closes *and* after
`vdcAVG` has plateaued, and make the window an integer number of half-cycles.

### 3.3 [LOW] NaN can pass the current-reference clamps

`currentReference = powerCommand·vac·KMUL / vacRMS.sqrOutput` (pfc.c:1012).
If `sqrOutput` were 0 with `powerCommand` 0, the result is NaN, and both
boundary checks (`> max`, `< 0`) are false for NaN — it propagates into the
PI, the duty, and the PWM register. The state machine makes this unreachable
today (CTRL_RUN requires a completed RMS window; a dead input trips IP_UV
first), but the guard is one line (`if (!(x >= 0)) x = 0;` style) and removes
an entire class of float-poisoning failure. Same pattern is worth applying to
`boostDutyRatio` (pfc.c:341) which divides by instantaneous `vdc`.

### 3.4 [LOW] ADC macro hygiene — unparenthesized shifts, UB on negatives, dead channel

- adc.h:68-71: the `ADCBUF_*` macros end in `<<4` with **no outer
  parentheses**. `ADCBUF_VDC>>1` (pfc.c:119) only works because `<<` and `>>`
  are left-associative; any arithmetic context (`ADCBUF_PFC_IL * k`) silently
  becomes `... << (4*k)`. Parenthesize the macro bodies.
- Left-shifting a negative `int16_t` is undefined behaviour in ISO C (works on
  XC-DSC, but it is one compiler flag away from being a real bug). Multiply by
  16 instead — same code generated.
- adc.c:144 configures a trigger for AD1CH2, a channel that is never otherwise
  configured or read; several comments name the wrong channels ("AD1CH2 - IL"
  for CH1, "AD2CH3" for AD2CH0). Dead config on an unconfigured channel
  invites a future mis-read.

### 3.5 [LOW] IL2 sample may be one cycle stale (conversion ordering)

VDC (AD1CH0), IL (AD1CH1) and IL2 (AD1CH3) share ADC core 1 and one trigger;
the ISR fires on **CH1** completion, and CH3 converts after it. Depending on
ISR entry latency the `AD1CH3DATA` read (pfc.c:122) races the end of the CH3
conversion: the load-FF input is nondeterministically this-cycle or
last-cycle. Through the ~500 Hz load-FF low-pass this is invisible, but it
should be a documented decision, not an accident — one comment, or move IL2 to
core 2 alongside VAC.

### 3.6 [MED] OP_UV auto-recovery margin is 1 V at nominal line

Recovery requires `vdcAVG ≥ 320 V` (userparams:336, pfc.c:516) while the
passive bus asymptote at 230 Vrms is ~321 V. Anything below ~228 Vrms and the
fault can never auto-clear; slightly above and it recovers with 1 V of margin
against measurement offset. The comment acknowledges the constraint — but the
precharge logic already solved this exact problem with a *fraction of the
measured peak* (userparams:360). Use the same construction:
`recovery = k × sqrt(2·vacRMS.sqrOutput)` with k ≈ 0.95, and the threshold
tracks line voltage automatically.

### 3.7 [LOW] Sticky `status` flags as a design pattern

`vacRMS.status` / `vacAVG.status` are set-once, cleared only in
`PFC_ResetParams`. Every consumer that means "a window has *ever* completed"
is fine; any future consumer that means "a *fresh* window completed" silently
gets the wrong semantics — §2.1 is exactly this failure. Either document the
one-shot semantics at the struct definition, or add a
consume-and-clear accessor.

### 3.8 [LOW] Assorted constants/comment drift

- `PFC_MAX_DUTY = 0.95f` sits under a comment demanding ≤ 0.9 (pwm.h:88-90).
- `PI_I_OUT_MAX` (userparams:394) is no longer referenced by code — only by a
  comment. Delete it or the next reader will assume it is live.
- `#include "libq.h"` in pfc.c:53 appears unused in the float implementation.
- `uint16_t counter` (pfc.c:99) is a file-global with an ISR-modified value
  and a collision-prone name — make it `static` and name it
  `diagDecimCounter`.
- `PFC_INPUT_FREQUENCY_COUNTER` truncates to 1066 (not 1066.67) if
  `PFC_INPUT_FREQUENCY` is ever set to 60 — the vacAVG window then leaks line
  frequency into `offsetVac`. Round, or derive per-frequency constants
  (see §5.1).

---

## 4. Hardware-deployment risks (verify before power-up)

These are not bugs in SiL but become live the day the board is powered.

1. **[CRIT] PWM frequency** — §3.1. One scope measurement settles it; do it
   first, everything else calibrates against it.
2. **[HIGH] ADC scaling vs the actual board** — the firmware's pin map is
   known to target the MCHV Motor Control DIM while the physical board is the
   EV67K87A Digital Power PIM. `PFC_VOLTAGE_BASE = 453 V` (userparams:162) and
   the current base (userparams:175) encode the *DIM's* divider and shunt/amp
   values; the VAC macro additionally implies a ±453 V bipolar span around a
   1.65 V offset. Every loop gain is in engineering units, so a scale error
   multiplies straight into loop gain. Verify all three front-ends (divider
   ratios, opamp gains, offsets) against the DP PIM schematic.
3. **[HIGH] Current-offset correction is compiled out** (userparams:81) —
   correct for SiL (the calibration is meaningless there), wrong for hardware:
   any real sensor offset shifts the software OCP thresholds asymmetrically
   and biases the DCM reconstruction at light load, which is this
   application's dominant operating region. Re-enable for hardware builds
   after fixing §3.2 — consider making it an explicit `#ifdef SIL_BUILD`
   inversion so the safe default is per-target.
4. **[MED] `PFC_LOAD_FF_ENABLE_DEFAULT = 1` contradicts its own deployment
   note** — the comment says "set back to 0 on hardware until the front-end
   scale and sign are measured" (userparams:219-224), but the shipped default
   is 1. A sign error in the load-current sensor turns the feed-forward into
   positive feedback on load steps. Make the safe value the default and enable
   from X2C after verification.
5. **[HIGH] No hardware cycle-by-cycle current limit** — `ENABLE_PWM_FAULT` is
   `#undef`'d (pwm.h:71) and the PCI fault block is compiled out. Protection
   is software-only: a mid-ON sample at 64 kHz compared against 16.97 A, i.e.
   worst-case one full switching period of blind time plus ISR latency before
   the duty even changes. A saturating inductor or shorted bus moves faster
   than that. Wire the shunt/comparator (CMP + PCI current-limit mode, or the
   external fault pin) so the PWM hardware truncates the pulse with no
   software in the loop.
6. **[MED] Watchdog disabled** (`WDTEN no` per the map file / device_config.c)
   — a hung main loop is survivable, a hung 64 kHz ISR is not; the PWM keeps
   switching with a stale duty register at whatever operating point it froze.
   Enable the WDT and kick it from the ISR-driven state machine, not from
   `main()`.
7. **[LOW] X2C sample-time constant** — 62.5 µs assumes the 64 kHz ISR;
   re-derive after item 1.

---

## 5. Control-theory weaknesses and improvements

### 5.1 [MED] The design is hard-wired to 50 Hz in four places

`PFC_INPUT_FREQUENCY` drives the window lengths, but the notch coefficients
are 100 Hz *literals* (userparams:135-139) and the header itself warns they
are wrong on 60 Hz mains. Improvements in ascending effort:

1. Compute the RBJ notch coefficients once in `PFC_ParamsInit` from
   `PFC_INPUT_FREQUENCY` and `VOLTAGE_LOOP_EXE_RATE` (one `sinf`/`cosf` at
   boot — the "deterministic literals" argument protects against a risk far
   smaller than the mis-tune risk it creates).
2. Round, don't truncate, the window counts (§3.8).
3. Longer term: measure the line period from `rectifiedVac` zero crossings
   (or a SOGI-PLL) and adapt windows + notch on the fly; also gives a
   line-loss detect far faster than the 10 ms RMS window.

### 5.2 [MED] The 10 ms staircase in `vacRMS.sqrOutput` feeds the current reference

The multiplier divides by a block-updated Vrms² that jumps every half-line
cycle (pfc.c:1012). In steady state the jumps are nulled by construction; on a
line sag/swell the current reference steps by the full correction once per
10 ms — a duty/current transient synchronized to the window boundary, and a
half-cycle of wrong-amplitude current before each step. Slew-limit or
first-order-filter the *reciprocal* (precompute `1/sqrOutput` once per window
tick and interpolate) — this also deletes a per-ISR division.

### 5.3 [MED] Light-load PF/THD: no EMI-capacitor current compensation

At this application's dominant low-power operating point, the input filter
capacitance draws a leading, voltage-proportional current comparable to the
programmed current. The controller regulates *inductor* current, so the mains
current inherits the capacitor's leading spike at the zero crossings and the
displacement factor cap at light load. Standard fix: subtract the known
reactive component from the reference, `iref_corr = iref − C_filt·dVac/dt`,
with dVac/dt from a two-sample difference of `rectifiedVac` (sign-corrected
around the crossing). Cheap and directly targets the light-load PF that the
load profile makes the primary requirement.

### 5.4 [LOW] Duty feed-forward re-injects raw Vdc switching noise

`boostDutyRatio` is computed from the *instantaneous* `vdc` sample
(pfc.c:334-344) and goes straight into the duty via `dutyFF` at full
bandwidth — the voltage loop is carefully protected by the boxcar/notch, but
this path bypasses all of it. A one-pole ~1–2 kHz filter on the `vdc` used by
`PFC_UpdateBoostDutyRatio` (not on the loop feedback) would cut duty jitter
without touching the FF's intended dynamics. Same argument applies to the
valley estimator's `vo` input (pfc.c:744), where noise directly jitters the
DCM boundary decision.

### 5.5 [LOW] Burst mode: add hysteresis and line-synchronized entry/exit

`PFC_BurstModeUpdate` (pfc.c:350) compares `powerCommand` against a single
1 W threshold — around it, the converter can toggle on/off at the
voltage-loop rate mid-sine, chopping current pulses at arbitrary phase
(audible noise, EMI, bus ripple). Improvements: (a) a hysteresis pair
(e.g. off < 1 W, on > 2.5 W); (b) quantize entry/exit to the line zero
crossing so bursts are whole half-cycles; (c) freeze the voltage-PI
integrator during the off stretch (it currently keeps integrating — bounded,
but it sets the re-entry transient).

### 5.6 [LOW] Current-loop gains are CCM-only by design — acceptable, document it

`Gid = Vout/(sL)` in the .m is the CCM plant; in DCM the plant collapses to a
low-gain first-order response, so closed-loop authority drops precisely where
the duty feed-forward takes over. This division of labour is sound — the PI
is a trim in DCM — but it should be stated in `PFC_CONTROL_THEORY.md` so
nobody "fixes" the DCM tracking by cranking KP_I (which would then ring in
CCM). Optional refinement: scale KP_I by the known DCM plant gain when
`dcmDetected`, but only if measurements show a need.

### 5.7 [LOW] Model the digital delays once, in the .m

The script prints a warning that `margin(Gvcl)` reads optimistically because
the boxcar, notch and ZOH are not modelled (mchp_pfc_foc_dsPIC33A_data.m:203).
Since the true crossover/PM numbers (6.9 Hz / 54°) currently live only in
memory and doc prose, add the exact discrete chain
(`c2d` + boxcar `(1−z⁻ᴺ)/(N(1−z⁻¹))` + notch biquad + computation delay) to
the script so every future retune reproduces them mechanically. The same
model then answers §3.1's "what if fs is actually 32 kHz" in one run.

### 5.8 [LOW] Voltage-loop energy (Vdc²) formulation — optional

Controlling `Vdc²` linearizes the bus-energy plant exactly (½C·dv²/dt = p)
instead of around the 380 V operating point. With a fixed setpoint and ±10 %
excursions the linearization error is small, so this is a refinement, not a
fix — worth it only if large-signal sag recovery ever becomes a tuning
problem. Note the current architecture (power command in watts, multiplier
division by Vrms²) already has the right structure for it; only the PI error
input changes.

---

## 6. Architecture improvements

### 6.1 [MED] Measure and budget the ISR

Everything — measurement conditioning, notch, both loops, DCM detection,
`sqrtf` paths, X2C every 4th tick — runs in one 64 kHz ISR at priority 7,
with no instrumentation. At 200 MHz there are 3125 cycles per tick; the float
chain plus a worst-case `DiagnosticsStepIsr` is plausibly fine, but it is an
assumption. Add a GPIO set/clear (or capture a free-running timer) at ISR
entry/exit and log min/max headroom via X2C. If headroom is tight, the
voltage-loop tick (PI + notch, already decimated by 12) is the natural piece
to move to a lower-priority software interrupt.

### 6.2 [HIGH] Complete the start-up state machine

Combining §2.1, §2.4 and §3.2, the start sequence needs three structural
additions, all cheap:

- `PFC_PRECHARGE`: timeout → latched fault (or plateau-detect completion);
- explicit `PFC_RELAY_SETTLE` state: fixed 20–30 ms (covers operate + bounce)
  between relay command and anything that trusts bus/current measurements;
- `PFC_OFFSET_MEAS` gated on non-conduction (§3.2) — and skipped entirely
  when offset correction is compiled out, rather than burning 16 ms measuring
  an unused value.

### 6.3 [MED] One consistent anti-windup story in the PI module

Today there are three mechanisms: the integrator slam inside
`PFC_ControllerPIUpdate` (§2.3), the back-calculation in
`PFC_CurrentControlLoop`, and the tight trim clamp doubling as windup bound.
Fold the correct one (output clamp + conditional integration or tracking
gain) into `PFC_PI_T`/`PFC_ControllerPIUpdate` itself, parameterized per
instance, and delete the special cases. The PI struct is float-only and
reorderable now (the asm module is retired), so the only constraint is the
SiL bus mirror — append, don't reorder.

### 6.4 [LOW] Fault observability

`PFC_FAULT_IP_OC` latches until reset and then vanishes on power cycle. Add a
first-fault snapshot (fault mask + `iL`, `vdc`, `vacRMS`, state, duty at trip)
in a struct X2C can read, and consider a small EEPROM/flash breadcrumb for
field returns. Cheap now, invaluable the first time a unit comes back "it
just stops sometimes".

### 6.5 [LOW] Naming/config drift

The MPLAB project still produces `PMSM.X.production.map` and the PWM comments
still say "PWM GENERATOR 3" over PG4 registers (pwm.c:274-437 passim) — relics
of the motor-control origin. Harmless individually; collectively they slow
down every schematic-vs-code check (which §4.2 makes safety-relevant).

---

## 7. Optimization opportunities

- **[LOW] Per-ISR division in the reference multiplier** — hoist
  `1/vacRMS.sqrOutput` to the RMS window tick (§5.2 gets you this for free).
- **[LOW] `sqrtf` in `PFC_StatePrecharge`** (pfc.c:407) recomputes the peak
  every ISR from a value that changes once per 10 ms; compute it on the window
  tick. Trivial, but it is the longest single ISR path in that state.
- **[LOW] `PFC_UpdateMeasurements` runs the full chain in every state** —
  deliberate for X2C observability (documented), and fine *if* §6.1 shows
  headroom; otherwise the loadFF and notch updates are skippable outside
  CTRL_RUN.
- **[LOW] Light-load efficiency features** — given the load profile (mostly
  < 100 W, deep DCM), the biggest efficiency lever is not code speed but
  switching-loss reduction: burst refinement (§5.5) first; optionally
  frequency foldback in deep DCM (halve fPWM below a power threshold — note
  this interacts with every Ts-derived constant, so build it on the §5.7
  model, and only after §3.1 is settled).

---

## 8. Suggested priority order

| # | Item | Ref |
|---|------|-----|
| 1 | Scope-verify the real PWM/ISR frequency | §3.1 |
| 2 | ~~Precharge timeout + relay settle + WAIT_1CYCLE fix~~ **done 2026-08-02** (explicit RELAY_SETTLE state and plateau-detect still open) | §2.1, §2.4, §6.2 |
| 3 | Hardware CBC current limit via PCI, WDT on | §4.5, §4.6 |
| 4 | ADC scale verification against the DP PIM | §4.2 |
| 5 | ~~`dutyFFEnable` limit refresh + PI anti-windup cleanup~~ **done 2026-08-02** | §2.2, §2.3, §6.3 |
| 6 | Offset-cal gating + enable for HW builds | §3.2, §4.3 |
| 7 | loadFF default-off for HW, verify sign/scale, re-enable | §4.4 |
| 8 | Notch coefficients from `PFC_INPUT_FREQUENCY` at init | §5.1 |
| 9 | EMI-cap current compensation, burst hysteresis | §5.3, §5.5 |
| 10 | Discrete loop model in the .m; ISR headroom measurement | §5.7, §6.1 |
