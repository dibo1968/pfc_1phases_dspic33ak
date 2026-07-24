# PFC Code Review — `pfc_1phases_dspic33ak` (re-review 2026-07-23)

Re-review of the PFC firmware under `project/pfc/`, cross-checked against the prior
review document (`SinglePhaseBoostPfc/PFC_CODE_REVIEW.md`).

Files covered:
- `project/pfc/pfc.c`
- `project/pfc/pfc.h`
- `project/pfc/pfc_userparams.h`
- `project/pfc/pfc_calc_params.h`
- `project/pfc/pfc_pi.c` / `pfc_pi.h`
- `project/pfc/pfc_measure.c` / `pfc_measure.h`
- `project/pfc/pfc_general.h`
- supporting: `project/hal/pwm.h`, `project/hal/adc.h`

---

## 0. Headline: the prior fixes are NOT in this tree

The attached review was performed against a **different project tree**
(`SinglePhaseBoostPfc/MPLABXProject/pfc/`). This project
(`pfc_1phases_dspic33ak/project/pfc/`) is a separate port to the dsPIC33AK part and
started from the **pristine, pre-fix Microchip reference code**. Spot-checks that
prove it:

| Marker the prior review said was *fixed*                     | This tree                                   |
|--------------------------------------------------------------|---------------------------------------------|
| Fault enum → powers of two (`1,2,4`)                         | still `1,2,3` — `pfc.h:101-104`             |
| `PFC_FaultCheck` uses `\|=`                                   | still `+=` — `pfc.c:605/610/615`           |
| Vdc average = one half-line cycle (`PFC_VDC_AVG_SAMPLES`)    | still `PFC_AVG_SCALER 7` (128 / 2 ms) — `pfc_userparams.h:97` |
| Software OCP (`PFC_FAULT_IP_OC`)                             | absent; `PFC_INPUT_OVER_CURRENT` unused     |
| `>=` voltage-loop gate, rate constant retuned               | still `>` with `VOLTAGE_LOOP_EXE_RATE 10`   |
| Uninitialised `output` seeded to 0                          | still `float output;` — `pfc.c:393`        |
| `#else` copy for `iL` when offset correction off            | still `#ifdef`-only — `pfc.c:202-204`      |

**So essentially every §1–§4 finding from the prior review is OPEN again here.**
The good news: the diagnoses and patches already exist in the sibling project and can
be ported almost verbatim. Below I re-confirm each against the current line numbers,
then add findings that are new to this tree.

---

## 1. Correctness bugs (fix first)

### 1.1 `faultStatus` uses `+=` with non-bitmask values — output OV silently auto-recovers  **[FIXED 2026-07-23]**

> **Resolution.** Fault enum is now powers of two (`pfc.h:102-104`: `1<<0,1<<1,1<<2`),
> so `OP_OV` no longer aliases `IP_UV|IP_OV`. `PFC_FaultCheck` recomputes a fresh mask
> each pass with `|=` and assigns it (`pfc.c:609-633`) — faults can no longer accumulate
> or shadow each other (this also closes §4.5 and §5.3). The `PFC_FAULT` state gained an
> explicit OP_OV recovery clear gated on `vdcAVG.output < PFC_OUTPUT_OVER_VOLTAGE_RECOVERY_LIMIT`
> (`pfc.c:265-273`); the new user param `PFC_OUTPUT_OVER_VOLTAGE_RECOVERY = 395.0f`
> (`pfc_userparams.h:154`, aliased in `pfc_calc_params.h:81/87`) gives 15 V of hysteresis
> below the 410 V trip, so an output OV fault now latches until the bus actually bleeds
> down. **Note:** OP_OV's numeric value changed 3→4 — any Simulink/X2C fault decode must
> bit-test (`faultStatus & PFC_FAULT_OP_OV`) rather than compare `== 3`.
>
> *(Original analysis retained below.)*

`pfc.h:101-104`:
```c
PFC_FAULT_IP_UV = 1,
PFC_FAULT_IP_OV = 2,
PFC_FAULT_OP_OV = 3,     // == IP_UV | IP_OV  (bit aliasing)
```
`pfc.c:603-616` accumulates with `+=`, and the `PFC_FAULT` recovery at
`pfc.c:257-264` only clears IP_UV and IP_OV — there is **no OP_OV clear at all**.

Trace an isolated output over-voltage at 230 Vrms input:
1. `faultStatus = 0 + 3` (OP_OV). → go to `PFC_FAULT`, PWM disabled.
2. In `PFC_FAULT`: `vacRMS² ≈ 52900 ≥ 130²` → clear IP_UV: `3 & ~1 = 2`.
3. `vacRMS² ≈ 52900 < 240²` → clear IP_OV: `2 & ~2 = 0`.
4. `faultStatus == PFC_FAULT_NONE` → **re-enter `PFC_CTRL_RUN`, re-enable PWM while the bus is still over-voltage.**

Because `OP_OV (3)` aliases `IP_UV|IP_OV (1|2)`, the two *input*-voltage recovery
tests clear an *output* fault. Worse, since the OP_OV condition is still true on the
next `PFC_CTRL_RUN` pass, the machine chatters `RUN↔FAULT` — toggling the PWM enable
every couple of ISRs at up to tens of kHz while the DC bus is over-voltage. This is a
safety-relevant defect.

**Fix (port from sibling):** make faults powers of two (`1,2,4,8…`), set with `|=`,
and add an explicit OP_OV recovery clear gated on `vdcAVG.output` falling below a
hysteresis threshold (e.g. 395 V), not on the input-voltage tests.

### 1.2 Uninitialised read in `PFC_CurrentSampleCorrection`  **[FIXED 2026-07-23 — removed]**

> **Resolution.** The whole `PFC_CurrentSampleCorrection` function (the only site of the
> uninitialised read) was deleted as dead code under §4.1, so the latent bug is gone by
> removal rather than by seeding `output = 0`.

`pfc.c:391-411`:
```c
float output;                                   // not initialised
if (pData->boostDutyRatio > 0) { output = ...; }
if (output > 0) { ... }                          // read when first if was false
```
When `boostDutyRatio <= 0` (every line zero-crossing, and startup) `output` is read
uninitialised. Currently unreachable at runtime because `sampleCorrectionEnable` is
hard-wired to 0 (§4.1), but it is a latent bug and an `-Wmaybe-uninitialized` warning
waiting to happen. Seed `float output = 0;`.

### 1.3 `pData->iL` only written when `ENABLE_PFC_CURRENT_OFFSET_CORRECTION` is defined  **[FIXED 2026-07-23]**

> **Resolution.** Added the `#else pfcData->iL = pCurrent->iL;` branch (`pfc.c:202-206`)
> so `pData->iL` is always populated. Pulled in alongside §1.5 because the new software
> OCP consumes `pData->iL`; without the `#else` the OCP would be silently disabled in a
> build with offset correction compiled out.

`pfc.c:202-204`:
```c
#ifdef ENABLE_PFC_CURRENT_OFFSET_CORRECTION
    pfcData->iL = pCurrent->iL - pCurrent->offset;
#endif
```
The macro *is* defined today (`pfc_userparams.h:81`), so the loop runs. But there is
no `#else`, and `pData->iL` is the sole current feedback consumed at `pfc.c:428/430/441`.
Undefine the macro and the current loop silently runs on stale/zero feedback. Add:
```c
#else
    pfcData->iL = pCurrent->iL;
#endif
```
(The sibling project hit exactly this — see the `pfc-current-feedback-gated-by-offset-ifdef` note.)

### 1.4 Off-by-one in `PFC_SquaredRMSCalculate`  **[FIXED 2026-07-23]**

> **Resolution.** Reordered to match `PFC_Average` (`pfc.c:656-672`): accumulate,
> `samples++`, then `if(samples >= sampleLimit)`. Each window now integrates exactly
> `sampleLimit` (640) terms and divides by 640. Removes the +1/640 (+0.156%) high bias on
> `sqrOutput` (~+0.078% on derived Vrms) and restores exact half-line-cycle window
> alignment (was 641 samples, drifting phase against the line). Low-severity — the
> current-ref bias was absorbed by the voltage loop and trip thresholds shifted <0.1% —
> but it makes the two averagers consistent.

`pfc.c:548-562` adds to the sum *before* the count test, so each window integrates
`sampleLimit + 1` products but divides by `sampleLimit`:
```c
pData->sum += input*input;
if (pData->samples < pData->sampleLimit) pData->samples++;
else { pData->sqrOutput = pData->sum / pData->sampleLimit; ...reset... }
```
The companion `PFC_Average` (`pfc.c:575-586`) increments *before* the test and is
correct. `vacRMS.sqrOutput` is biased high by ~1/640. Small, but it feeds the RMS²
feed-forward divisor and every UV/OV trip threshold, and the inconsistency between the
two averagers is a trap. Match `PFC_Average`'s ordering.

### 1.5 Configured over-current limit is dead — no software OCP  **[FIXED 2026-07-23]**

> **Resolution.** New `PFC_INPUT_OVER_CURRENT_PEAK = PFC_INPUT_OVER_CURRENT · √2`
> (`pfc_calc_params.h:92`, ≈16.97 A from the existing 12 Arms user limit). New latching
> fault bit `PFC_FAULT_IP_OC` (`pfc.h:106`). `PFC_FaultCheck` now does a symmetric
> instantaneous check on the offset-corrected `pData->iL` (`pfc.c:641-645`). It is
> **latching** — `PFC_FAULT` provides no auto-clear, because once PWM is off the inductor
> current decays in µs and a threshold clear would self-clear instantly; it holds until
> `PFC_ServiceInit`. The unrelated `currentReference > 14.14` clamp (`pfc.c`) is still a
> magic literal, tracked under §4.2.

`PFC_INPUT_OVER_CURRENT 12.0f` (`pfc_userparams.h:132`) is never referenced. There is
no instantaneous over-current fault anywhere; the only protection is whatever PWM
hardware fault / analog comparator is wired. Meanwhile the current-reference clamp
`currentReference > 14.14` (`pfc.c:506`) is an unrelated magic literal
(14.14 ≈ √2·10) that is inconsistent with the 12 Arms define.

**Fix:** derive `PFC_INPUT_OVER_CURRENT_PEAK = 12·√2 ≈ 16.97 A`, add
`PFC_FAULT_IP_OC`, and check `|pData->iL|` in `PFC_FaultCheck`. Make it **latching**
(no auto-clear — once PWM is off the inductor current decays in µs and any
threshold-based auto-clear self-clears instantly).

---

## 2. Control-performance issues

### 2.1 Vdc average window does not match the 100 Hz ripple  **[FIXED 2026-07-23]**

> **Resolution.** New `PFC_VDC_AVG_SAMPLES = PFC_PWMFREQUENCY_HZ / (2·PFC_INPUT_FREQUENCY)`
> = 640 samples = 10 ms = exactly one 100 Hz ripple period (`pfc_userparams.h:93-102`).
> `PFC_ParamsInit` sets `vdcAVG.sampleLimit` to it directly (`pfc.c:429`); the power-of-two
> `scaler` mechanism is gone (removed the `1<<scaler` derivation and the now-dead
> `PFC_AVG_T.scaler` field, `pfc.h`). The boxcar now structurally nulls the double-line
> ripple and its harmonics, so it no longer leaks into the voltage-loop error / current
> reference. Comment fixed too (closes §4.9).
>
> **Functional change — needs bench check.** The window is 5× longer (2 ms → 10 ms), so the
> voltage-loop feedback is slower and cleaner. The existing KP_V/KI_V were tuned against
> the 2 ms window; verify voltage-loop transient/stability and re-tune if needed. Payoff is
> lower input-current 3rd-harmonic THD. If the 10 ms block-average ZOH delay hurts the loop,
> the next step up is a sliding-window integrator (updates every ISR, same ripple null) —
> more RAM/compute; deferred unless the bench shows it's needed.
`PFC_AVG_SCALER 7` → 128 samples → **2 ms** at 64 kHz (`pfc_userparams.h:97`,
`pfc.c:320-321`). The double-line ripple period is **10 ms**. 2 ms averages one-fifth
of a ripple period, so ~100 Hz survives on the voltage-loop error, gets integrated
into the power command, and re-injects 3rd-harmonic distortion into the input current.
The comment at `pfc.c:151-152` ("remove line frequency ripple") is wrong (§4.9). Fix:
average an integer number of half-line cycles — `PFC_PWMFREQUENCY_HZ / (2·PFC_INPUT_FREQUENCY)`
= 640 samples / 10 ms. The averager already does a float divide, so losing the
power-of-two costs nothing.

### 2.2 Voltage feed-forward (1/V_rms²) lags up to one half-cycle  **[OPEN]**
`vacRMS.sqrOutput` updates only when the half-cycle window wraps (`pfc.c:557`); the
divisor at `pfc.c:502-503` is stale for up to 10 ms during line transients. Consider a
faster rolling estimate or a slew-limited update.

### 2.3 Voltage-loop gate is off-by-one vs the constant and the comment  **[FIXED 2026-07-23]**

> **Resolution.** Gate changed to `if (++pData->voltLoopExeRate >= VOLTAGE_LOOP_EXE_RATE)`
> with the `else{ counter++ }` removed (`pfc.c:586-600`), so the divisor is now exactly the
> constant. Constant set to **12** (`pfc_userparams.h:193`), which **preserves the current
> 5.33 kHz rate exactly** (old logic: threshold 10 → divisor 12) — so this is behaviour-
> neutral and the existing KP_V/KI_V stay valid; §2.1 remains the only dynamics change to
> re-tune against. Comment corrected to state 64 kHz/12 ≈ 5.33 kHz and to document that
> setting the constant to 16 gives the originally-intended 4 kHz (at the cost of re-tuning).
> Verified: startup phase and run cadence are identical to the old code (first run on the
> 12th RUN-ISR either way).
`pfc.c:478`: `if (voltLoopExeRate > VOLTAGE_LOOP_EXE_RATE)` with post-increment in the
`else`. With `VOLTAGE_LOOP_EXE_RATE = 10`, the PI actually runs once every **12** ISRs
(period = threshold + 2), i.e. 64 kHz/12 ≈ **5.33 kHz** — while the comment claims
"64 kHz/16 = 4 kHz" (`pfc_userparams.h:167-172`). Documentation, constant, and behaviour
all disagree. *(The prior review said "11 ISRs / 5.8 kHz"; that was itself off by one —
the true period is 12.)* Use `if (++voltLoopExeRate >= VOLTAGE_LOOP_EXE_RATE)` so the
divisor is exactly the constant, then pick the intended rate and re-tune KP_V/KI_V.

### 2.4 Discontinuous gain step on voltage error (no hysteresis)  **[FIXED 2026-07-23]**

> **Resolution.** Replaced the single 10 V threshold with a hysteresis band
> (`pfc.c:590-604`): `|error| > PFC_VOLTAGE_ERR_GAIN_HI` (12 V) → `Ki = KI_V/2`;
> `|error| < PFC_VOLTAGE_ERR_GAIN_LO` (8 V) → `Ki = KI_V`; inside [8, 12] the previous
> `Ki` is held, so the gain can no longer dither tick-to-tick at the switch point. No new
> state — `piVoltage.ki` persists across voltage-loop ticks and encodes the last decision.
> Thresholds are named user params (`pfc_userparams.h:222-228`), ±2 V around the original
> 10 V step. **Behaviour change** (switch points 10→12/8), so verify on the bench with the
> §2.1/§2.3 voltage-loop pass.
`pfc.c:482-489`: `ki` hard-switches between `KI_V` and `KI_V/2` at exactly `|error| = 10 V`.
At that boundary the gain dithers every voltage tick → potential limit cycle. Use a
hysteresis band (e.g. slow down above 12 V, restore below 8 V) with the thresholds as
named params.

### 2.5 Clamp-style anti-windup corrupts integrator state  **[OPEN]**
Both PIs use `integralOut = limit` on saturation (`pfc_pi.c:77-86`). The honest
integrator value is overwritten by the limit, so recovery from saturation is jerky.
Back-calculation (`integralOut += (clamped − U)·Kt`) preserves linearity. *(The PI
clamp is symmetric — the prior "asymmetric anti-windup" claim was already corrected to
this; see §4.8 for the redundant secondary clamp.)*

### 2.6 Soft-start expressed in raw counts, not V/s  **[OPEN]**
`RAMP_COUNT = PFC_VOLTAGE_BASE/32768 ≈ 0.0138 V` per step, every `RAMP_RATE+1 = 21`
ISRs → ≈ 42 V/s, but you have to reverse-engineer that through the ISR period
(`pfc_userparams.h:159-160`, `pfc.c:211-226`). Re-express as `SOFT_START_VPS` and derive
the counters.

### 2.7 Burst-mode threshold has no hysteresis  **[OPEN]**
`pfc.c:242-246` zeroes duty when `piVoltage.output < PFC_MIN_POWER` and re-engages at
the same threshold → chatter at minimum load. Add an on/off band. (Especially relevant
here — the load profile is pulse-type and mostly light / DCM.)

### 2.8 Voltage-PI bump on FAULT exit  **[OPEN]**
`pfc.c:265-271`: on recovery, `reference = vdcAVG.output` and both integrators are
zeroed while the load may already be drawing → current spike at re-engagement.

### 2.9 No zero-crossing / DCM phase compensation  **[OPEN]**
Pure shape-follow PFC distorts near the line zero crossings; worth revisiting if input
THD is a spec. (See the `pfc-current-sampling-valley-and-dcm-reconstruction` note.)

---

## 3. Real-time / numerical

### 3.1 Float divide in ISR with no lower bound  **[OPEN]**
`pfc.c:502-503` divides by `vacRMS.sqrOutput`; `pfc.c:230` divides by `vdc` (guarded
only by `> 0`). Structurally `PFC_FaultCheck` runs first and trips IP_UV on AC loss, so
normal operation is safe — but floor the divisor
(`max(sqrOutput, PFC_INPUT_UNDER_VOLTAGE_LIMIT_LO)`) as cheap defense-in-depth against
reordering / mistuned thresholds.

### 3.2 Entire control runs in the ADC ISR  **[OPEN]**
`pfc.c:96-131`: averaging, RMS, both PIs, fault logic and diagnostics all run in
`PFC_ADCInterrupt` at 64 kHz. It fits with the FPU, but there is no headroom
instrumentation. Add an ISR worst-case duration counter (read a free-running timer
before/after, keep the max) and expose it to X2C.

### 3.3 / 3.4 Bare `ADCBUF_VDC >> 1` and one `ADC_VOLTAGE_SCALE` for two front-ends  **[OPEN — re-characterised]**
`pfc.c:99`: `outputVoltage = ADCBUF_VDC >> 1`. With this tree's ADC macros
(`adc.h:68-71`) each raw reading is already `<< 4`, so:
- Vdc: `AD1CH0DATA<<4` (unipolar, 0…65520) `>>1` → 0…32760, ×(453/32768).
- Vac: `(AD2CH0DATA−2048)<<4` (bipolar, ±32768), ×(453/32768).

The `>>1` is **not** a 2× error — it compensates for Vdc using the full ADC span
(0…3.3 V ⇒ 0…453 V) while Vac is bipolar (0…3.3 V ⇒ −453…+453 V). For the *same*
physical voltage the two paths compute the same volts **only if** the analog dividers
have the intended 2:1 gain relationship. That assumption is undocumented and baked into
a bare shift + a shared scale. Add a comment tying it to the schematic, and verify the
Vac divider against it — if the hardware gains are not 2:1, one channel is off by 2×.

### 3.5 `outputVoltage` / `acVoltage` are `int16_t`  **[OPEN]**
`pfc_measure.h:81-82`. `outputVoltage` holds a unidirectional value that reaches 32760
— one bit under the `int16_t` max. Use `uint16_t` for the raw Vdc reading to remove the
arithmetic-shift / near-overflow hazard.

### 3.6 Diagnostics decimation is a magic `counter < 3`  **[FIXED 2026-07-23 — via §4.2]**
Now `PFC_DIAGNOSTICS_DECIMATION = 4` (`pfc_userparams.h`), used as
`counter < (PFC_DIAGNOSTICS_DECIMATION - 1)` in the ISR — the 16 kHz scope rate is visible
and tunable. Behaviour unchanged.

---

## 4. Readability / maintainability

### 4.1 Dead / vestigial code  **[PARTIALLY FIXED 2026-07-23]**

> **Resolution (partial).** Removed the entire sample-correction feature: the
> `PFC_CurrentSampleCorrection` function + forward declaration, the
> `if (sampleCorrectionEnable == 1)` branch in `PFC_CurrentControlLoop` (now an
> unconditional `averageCurrent = iL`), and the `sampleCorrectionEnable` field + its
> initialiser. Also fixed §1.2 by deletion.
>
> **Kept deliberately** (do **not** delete): `PFC_PI_T.propOut/input/kpScale/kiScale` —
> `pfc_pi.s` hardcodes `PFC_PI_T` byte offsets (`.equ _kpScale,16` …), so changing that
> struct's layout would silently corrupt the assembly PI if it is ever built.
> `PFC_RMS_SQUARE_T.peak/peakcheck`, `PFC_T.samplePoint`, and `volatile boostDutyRatio`
> are likely X2C-scope observables — §5 recommends *populating* them, not removing them.
>
> **`PFC_POWER_CONTROL` removed (2026-07-23, per go-ahead).** The `#ifdef`/`#endif` were
> stripped so the current-reference calc is now unconditional (`pfc.c:567-571`,
> behaviour-neutral — the macro was always defined). The `#define` is gone from
> `pfc_userparams.h`, and the obsolete "ensure PFC_POWER_CONTROL is defined" setup step was
> removed from `README.md`. (The `images/pfc_power_control.png` asset is now unreferenced —
> left in place, not deleted.)
- `sampleCorrectionEnable` set to 0 once (`pfc.c:341`), never changed → the whole DCM
  branch (`PFC_CurrentSampleCorrection`, `pfc.c:391-411`; call site `pfc.c:435-438`) is
  unreachable.
- `PFC_RMS_SQUARE_T.peak` / `peakcheck` (`pfc.h:85-86`) never written (peak is reset in
  `PFC_ResetParams` but never used).
- `PFC_PI_T.propOut`, `input`, `kpScale`, `kiScale` (`pfc_pi.h:74,76,83-84`) unused;
  reinforced by the commented-out lines `pfc.c:328-329`.
- `PFC_POWER_CONTROL` (`pfc.c:498-504`) has no `#else` — effectively mandatory but
  dressed up as configurable.
- `PFC_T.samplePoint` (`pfc.h:110`) never referenced.
- `inline static` on `PFC_CurrentRefGenerate` / `PFC_CurrentControlLoop` (`pfc.c:66-67`)
  is a no-op for same-TU statics.

*Keep* `boostDutyRatio` (`volatile` → X2C observability) even though its only consumer is
the dead DCM branch.

### 4.2 Magic literals  **[FIXED 2026-07-23]**

> **Resolution.** All named in `pfc_userparams.h`, behaviour-preserving:
> - `14.14` (current-ref clamp, 2×) → `PFC_IREF_PEAK_MAX` (documented as √2·~10 Arms, below
>   the 12 Arms OCP trip).
> - `0.0001` (iL floor) → `PFC_IL_MIN`.
> - `counter < 3` (diag decimation) → `counter < (PFC_DIAGNOSTICS_DECIMATION - 1)` with
>   `PFC_DIAGNOSTICS_DECIMATION = 4` (closes §3.6).
> - `> 10 / < -10` (voltage-error gain switch) → `PFC_VOLTAGE_ERR_GAIN_THRESHOLD` (the
>   named home §2.4 will split into a HI/LO hysteresis band).
>
> Values unchanged; `14.14`/`0.0001` now live only in the `#define`s. The `int`/`double`
> literals became `float` (`14.14f`, `0.0001f`, `10.0f`) — same stored values, comparisons
> now consistently in float.
`14.14` (`pfc.c:506`), `0.0001` (`pfc.c:430`), `counter < 3` (`pfc.c:119`),
`> 10 / < -10` gain switch (`pfc.c:482`), `piVoltage.maxOutput = 1500`
(`pfc.c:336`/`PI_V_OUT_MAX`). Name them and tie to physical limits.

### 4.3 Wrong type on local  **[FIXED 2026-07-23 — via §6]**
`pfc.c:146`: `uint16_t pfcState = pfcData->state;` — `state` is `PFC_CTRL_STATE_T`. Use
the enum so the compiler can warn on unhandled cases.

### 4.4 File-scope globals  **[OPEN]**
`pfcParam` and `counter` (`pfc.c:78-79`). `counter` should be `static`; `pfcParam`
makes the module non-reentrant / hard to unit-test.

### 4.5 `PFC_FaultCheck` clauses stack in one call  **[FIXED 2026-07-23 — via §1.1]**
`pfc.c:600-617`: three independent `if`s each `+=`. Combined with §1.1's aliasing,
`IP_UV+OP_OV = 4` is an unrepresentable, un-clearable value. A bitmask (`|=`) plus
"detect vs. recover" separation fixes this.

### 4.6 Doxygen / copy-paste rot  **[OPEN]**
`pfc_pi.c:65` still references `MC_ControllerPIUpdate` (motor-control name). Many
`@example` blocks just restate the prototype.

### 4.7 Header coupling  **[FIXED 2026-07-23]**

> **Resolution.** Moved `#include "pfc_calc_params.h"` out of `pfc.h` and into `pfc.c`, so
> the user-config chain (`pfc_calc_params.h` → `pfc_userparams.h`) no longer leaks into the
> module's public header. Verified safe: `pfc.h` is included only by `pfc.c` and `main.c`,
> `main.c` uses just `PFC_ServiceInit()`, and `pfc.h`'s type definitions need only
> `pfc_pi.h`/`pfc_measure.h`. `pfc.c` still gets everything transitively (calc_params →
> userparams → pwm.h).
`pfc.h` → `pfc_calc_params.h` → `pfc_userparams.h` (`pfc_calc_params.h:62`): user config
leaks into the module's public surface. Move the user-param include into the `.c`.

### 4.8 Mixed units in the current path  **[OPEN]**
PI output is a duty *fraction* (`maxOutput = PFC_MAX_DUTY = 0.95`) but the secondary
clamp (`pfc.c:449-461`) works in PWM *counts* (`PFC_MAX_DUTY_COUNTS`). The high branch
re-pins `integralOut = PI_I_OUT_MAX` after the PI already clamped it (redundant); the
low branch (`duty < PFC_MIN_DUTY_COUNTS = 0`) can never fire. Pick one unit for the
controller output and drop the redundant clamp (keep a defensive symmetric one if
desired, but don't re-touch the integrator).

### 4.9 Misleading Vdc-average comment  **[FIXED 2026-07-23 — via §6 + §2.1]**
The old "removes line frequency ripple" claim was dropped in the §6 refactor and replaced
with an accurate ripple-nulling comment once §2.1 made it true (`pfc.c:184-185`).

### 4.10 Upside-down recovery-threshold names  **[OPEN]**
`pfc.c:257-263`: the *HI* input-UV limit clears the UV fault and the *LO* input-OV limit
clears the OV fault. Rename to `PFC_UV_RECOVERY` / `PFC_OV_RECOVERY`.

---

## 5. New findings (specific to this tree)

### 5.1 `IL2` (load current) → load-power feed-forward  **[IMPLEMENTED 2026-07-23]**

> **Resolution.** Per the design intent, IL2 is the **load/output current** (after the
> DC-link cap). Replaced the unused `pfcCurrent2` with `PFC_LOAD_FF_T loadFF` (`pfc.h`),
> and implemented a **load-power feed-forward**: filter the load current
> (`loadFF.currentFilt`, IIR), form `powerFF = gain·Vdc·I_load` (`pfc.c` in
> `PFC_UpdateMeasurements`), and add it to the voltage-loop power command in
> `PFC_CurrentRefGenerate` (`i_ref = (P_pi + P_ff)·v_rect/V_rms²`). This cancels the load
> term in the bus power balance (`C·Vdc·dVdc/dt = P_pi`), so the slow voltage loop only
> trims losses and load steps are answered immediately.
>
> **Off by default** (`loadFF.enable = 0`) and computed-but-not-injected, so the
> measurement can be watched in X2C first. Tunables in `pfc_userparams.h`:
> `PFC_LOAD_CURRENT_SCALE` (**VERIFY vs the load sensor** — placeholder = inductor scale,
> which is also what the SiL uses to inject `Iout`, so the default is correct in SiL),
> `PFC_LOAD_FF_FILT_COEFF` (0.05), `PFC_LOAD_FF_GAIN` (0.8), `PFC_LOAD_FF_ENABLE_DEFAULT`.
> The `PFC_IREF_PEAK_MAX` clamp still bounds the total reference.
>
> **Bring-up:** SiL first (feed `Iout_in` steps, watch `loadFF.current` track it, enable,
> confirm the bus dip on a load step shrinks); then on HW verify sensor **scale + sign**
> before enabling. Interactions to watch: burst-mode threshold still keys off
> `piVoltage.output` only; feed-forward is active during soft-start.
`pfc.c:102/107` read and scale `pfcCurrent2` from `ADCBUF_PFC_IL2` (`adc.h:71`), and
`pfc.c:148` declares `PFC_MEASURE_CURRENT_T *pCurrent2` — but **nothing consumes it**:
no offset, no fault check, no control term, no diagnostics. `pCurrent2` is a
set-but-unused local (compiler warning), and two float ops per ISR are wasted. The ADC
channel for IL2 has been actively churned in recent commits, so this is clearly WIP —
but as it stands the second phase/sensor is dead weight. Decide: (a) it's a real second
interleaved phase → wire it into current sharing + its own OCP; (b) it's a redundant
sensor → use it for cross-check / OCP; or (c) remove it until needed.

### 5.2 No output *under*-voltage / boost-failure detection  **[FIXED 2026-07-23]**

> **Resolution.** New fault bit `PFC_FAULT_OP_UV` (`pfc.h:105`) and
> `PFC_OUTPUT_UNDER_VOLTAGE_LIMIT` alias (`pfc_calc_params.h`). `PFC_FaultCheck`
> trips it when `vdcAVG.output < 310 V`, but **only once soft-start has reached the
> nominal reference** (`pData->piVoltage.reference >= PFC_OUPUT_VOLTAGE_REFERENCE`,
> `pfc.c:657-663`) — during the ramp, and during any post-fault re-ramp, the bus is
> legitimately below nominal so the check is masked.
>
> **Auto-recovering with hysteresis** (per user request): `PFC_FAULT` clears OP_UV once
> `vdcAVG.output >= PFC_OUTPUT_UNDER_VOLTAGE_RECOVERY` (320 V, `pfc.c:282-285`). Because
> PWM is off during the fault, the bus is passively capped at the rectified input peak,
> so the recovery limit sits below that peak (320 V vs ≈325 V at 230 Vrms → a 10 V band,
> not the OV loop's 15 V). Net behaviour is a **slow hiccup**: on a persistent overload
> the converter retries roughly every soft-start period (OP_UV re-arms only when the
> ramp reaches nominal). At low line the bus can't passively reach 320 V, so it holds
> off until the line/bus recovers. Only `IP_OC` remains latching.
`PFC_OUTPUT_UNDER_VOLTAGE 310.0f` (`pfc_userparams.h:152`) and its `_NORMALIZED` alias
(`pfc_calc_params.h:81`) are defined but never checked. A collapsing bus (boost failure,
overload, lost feedback) is not caught — only OP_OV, IP_UV, IP_OV are. Add an OP_UV
fault (latched, with a startup mask so soft-start doesn't trip it).

### 5.3 `faultStatus` is never re-zeroed on entry to `PFC_CTRL_RUN`  **[FIXED 2026-07-23 — via §1.1]**
It is set once in `PFC_ParamsInit` and only ever mutated by `+=` / `&=`. Given §1.1's
aliasing this is how a stray `4` can persist. A clean "recompute from scratch each pass"
fault evaluation (bitmask, no running accumulation) removes the whole class.

### 5.4 `PFC_ResetParams` doesn't reset everything it implies  **[NEW / minor]**
`pfc.c:356-377` resets the averagers and integrators but not `faultStatus`,
`voltLoopExeRate`, `rampRate`, or `state`. Harmless today (INIT runs once) but a latent
trap if the machine is ever made re-enterable.

---

## 6. Architecture / readability refactor (the part you asked about)

> **§6.1–§6.3 + parts of §6.6 IMPLEMENTED 2026-07-23.** `PFC_StateMachine` is now a
> thin dispatcher over a `PFC_CTRL_STATE_T`-typed state (fixes §4.3), calling one handler
> per state — `PFC_StateInit/OffsetMeas/Wait1Cycle/CtrlRun/Fault`, each returning the next
> state. The always-run block is `PFC_UpdateMeasurements`; `PFC_CTRL_RUN` is decomposed
> into `PFC_UpdateCurrentFeedback`, `PFC_SoftStartUpdate`, `PFC_CurrentRefGenerate`,
> `PFC_UpdateBoostDutyRatio`, `PFC_CurrentControlLoop`, `PFC_BurstModeUpdate`. The unused
> `pCurrent2` local is gone (the IL2 ADC read in the ISR stays, so §5.1 is otherwise
> unchanged). Behaviour-neutral: side-effect order and state transitions are identical
> (verified by trace + structural checks; not yet compiled/bench-run). Still open from
> §6: the bitmask fault module (§6.4 — largely done via §1.1/§1.5), single controller
> unit (§6.5), header-include hygiene and dead-code removal (§6.6).

The functionality is hard to follow because `PFC_StateMachine` mixes three concerns in
one function: (a) per-ISR signal conditioning that runs regardless of state, (b) the
state dispatch, and (c) the full body of each state inline — with the biggest state
(`PFC_CTRL_RUN`, ~50 lines) carrying soft-start, ref-gen, duty math, current loop and
burst logic all at one indentation level. Here is a concrete, low-risk restructuring.

### 6.1 Split "always-run measurement" from the state machine
The block at `pfc.c:150-163` (Vdc/Vac average, rectify, RMS) runs every ISR in every
state. Pull it out:

```c
static void PFC_UpdateMeasurements(PFC_T *p)
{
    PFC_Average(&p->vdcAVG, p->pfcVoltage.vdc);
    p->outputVdc = p->vdcAVG.output;
    PFC_Average(&p->vacAVG, p->pfcVoltage.vac);
    p->pfcVoltage.offsetVac = p->vacAVG.output;
    p->rectifiedVac = PFC_SignalRectification(&p->pfcVoltage);
    PFC_SquaredRMSCalculate(&p->vacRMS, p->rectifiedVac);
}
```

### 6.2 One function per state, each returning the next state
This is exactly the "move each state's content into its own function" idea. Each handler
takes the context and returns the next `PFC_CTRL_STATE_T`, so transitions are explicit
and greppable:

```c
static PFC_CTRL_STATE_T PFC_StateInit(PFC_T *p);
static PFC_CTRL_STATE_T PFC_StateOffsetMeas(PFC_T *p);
static PFC_CTRL_STATE_T PFC_StateWait1Cycle(PFC_T *p);
static PFC_CTRL_STATE_T PFC_StateCtrlRun(PFC_T *p);
static PFC_CTRL_STATE_T PFC_StateFault(PFC_T *p);

void PFC_StateMachine(PFC_T *p)
{
    PFC_UpdateMeasurements(p);

    switch (p->state)
    {
        case PFC_INIT:        p->state = PFC_StateInit(p);        break;
        case PFC_OFFSET_MEAS: p->state = PFC_StateOffsetMeas(p);  break;
        case PFC_WAIT_1CYCLE: p->state = PFC_StateWait1Cycle(p);  break;
        case PFC_CTRL_RUN:    p->state = PFC_StateCtrlRun(p);     break;
        case PFC_FAULT:       p->state = PFC_StateFault(p);       break;
        default:              p->state = PFC_INIT;                break;
    }
}
```

`PFC_StateMachine` now reads as a one-page table of contents; the detail lives in named
functions you can read in isolation. (A function-pointer table indexed by state is the
next step up, but for five states the explicit switch is clearer and the compiler still
inlines the statics.)

### 6.3 Break `PFC_CTRL_RUN` into named sub-steps
The run state is where the control actually lives; give each phase a name:

```c
static PFC_CTRL_STATE_T PFC_StateCtrlRun(PFC_T *p)
{
    PFC_UpdateCurrentFeedback(p);          // §1.3 iL copy, offset-correct
    PFC_FaultCheck(p);
    if (p->faultStatus != PFC_FAULT_NONE)
        return PFC_FAULT;

    PFC_SoftStartUpdate(p);                // the ramp block, pfc.c:211-226
    PFC_CurrentRefGenerate(p);
    PFC_UpdateBoostDutyRatio(p);           // pfc.c:230-238
    PFC_CurrentControlLoop(p);
    PFC_BurstModeUpdate(p);                // pfc.c:242-246, + hysteresis (§2.7)
    return PFC_CTRL_RUN;
}
```

Now the control sequence is legible at a glance, and each concern (soft-start,
ref-gen, burst) is independently testable and independently tunable.

### 6.4 Make faults a proper bitmask module
Bundle detection and recovery so §1.1/§4.5/§5.2/§5.3 stop being possible:

```c
typedef enum {
    PFC_FAULT_NONE  = 0,
    PFC_FAULT_IP_UV = 1u << 0,
    PFC_FAULT_IP_OV = 1u << 1,
    PFC_FAULT_OP_OV = 1u << 2,
    PFC_FAULT_OP_UV = 1u << 3,
    PFC_FAULT_IP_OC = 1u << 4,
} PFC_FAULT_TYPE_T;

// recompute from scratch each pass; never accumulate
static uint16_t PFC_EvaluateFaults(const PFC_T *p)
{
    uint16_t f = 0;
    if (p->vdcAVG.output   >= PFC_OUTPUT_OVER_VOLTAGE_LIMIT) f |= PFC_FAULT_OP_OV;
    if (p->vacRMS.sqrOutput <  PFC_INPUT_UNDER_VOLTAGE_LIMIT_LO) f |= PFC_FAULT_IP_UV;
    if (p->vacRMS.sqrOutput >= PFC_INPUT_OVER_VOLTAGE_LIMIT_HI)  f |= PFC_FAULT_IP_OV;
    if (fabsf(p->iL) >= PFC_INPUT_OVER_CURRENT_PEAK)            f |= PFC_FAULT_IP_OC;
    return f;
}
```
`PFC_StateFault` then clears each bit against its own recovery threshold (with the OP_OV
clear gated on the bus actually falling), and latches OP_OV/IP_OC.

### 6.5 Pick one unit for the controller output
Let the current PI output be a duty fraction end-to-end and convert to counts exactly
once at the write to `PFC_PWM_PDC`, or keep everything in counts. Either kills the §4.8
double-representation and the redundant secondary clamp.

### 6.6 Smaller hygiene wins that aid readability
- Type the state local as `PFC_CTRL_STATE_T` (§4.3); `static`-qualify `counter` (§4.4).
- Move `pfc_userparams.h` out of the public header chain (§4.7).
- Replace the magic literals with named, physically-derived constants (§4.2).
- Delete the dead members/branches (§4.1) — every deleted field is one less thing to
  reason about; keep only the `volatile` X2C observables.
- Optionally strip the `<editor-fold>` markers (MPLAB-only UI noise).

None of §6 changes control behaviour on its own — it is pure structure — so it can land
before the functional fixes and make those fixes smaller and safer to review.

---

## 7. Suggested order of attack

1. **§1.1 + §4.5 + §5.3** — fault bitmask + separate detect/recover + OP_OV hysteresis
   clear. Safety-critical; do first. (§6.4 is the vehicle.)
2. **§1.3, §1.4** — iL `#else`, RMS off-by-one. One-liners, correctness.
3. **§1.5 + §5.2** — real software OCP and output-UV, both latching.
4. **§2.1** — Vdc average = one half-line cycle (kills 100 Hz distortion path).
5. **§2.3** — fix the voltage-loop divisor and reconcile the comment/constant/rate.
6. **§6.1–§6.3** — state-handler extraction for readability (safe, behaviour-neutral).
7. **§5.1** — decide IL2's fate (wire in or remove).
8. **§2.4–§2.8, §2.5 back-calc, §3.x instrumentation** — tuning-pass bundle, on the bench.

All of §1's diagnoses and most patches already exist in the sibling
`SinglePhaseBoostPfc` tree and can be ported with the line numbers above.
