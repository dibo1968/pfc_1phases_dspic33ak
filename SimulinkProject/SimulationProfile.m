% SiL scenario catalogue. Select with SimIndex below, run via
% run_sil_scenario.m (headless) or Play in Simulink.
%
% Columns:
%   1  SimTime      simulation stop time [s]
%   2  SimLoadTime  load-resistance breakpoints [s]   (row vector)
%   3  SimLoadVal   load resistance at breakpoints    (row vector, ohm)
%   4  SimCoutInit  initial DC-bus capacitor voltage [V] (0 = full precharge)
%   5  SimName      short scenario tag (goes into the run log / .mat name)
%   6  SimVacRms    nominal input voltage [Vrms]; also the default value of
%                   the time-varying SimVacTime/SimVacVal profile below
%   7  overrides    struct() of per-profile overrides. Any field name must
%                   match one of the Sim* variables defined in the defaults
%                   block at the bottom (SimVacTime/SimVacVal, SimCfg,
%                   SimLtol/SimCtol, SimILGain/SimILBias, SimVinGain/...,
%                   SimCfgTime/SimCfgVal). struct() = all defaults.
%
% Conventions:
% - Load / Vac steps are encoded for the linearly-interpolating From
%   Workspace blocks as breakpoint pairs 1e-4 s apart (t-1e-4 -> old, t -> new).
% - R = Vout^2/Po = 96.27 ohm = 1500 W at 380 V, so R*k means 1500/k watts.
%   R*1e5 is the "no load" convention.
% - SimCfg = [dutyFFEnable; sampleCorrectionEnable; vdcNotch.enable;
%   loadFF.enable], -1 = leave firmware default, 0 = force off, 1 = force on.
%   The model wiring is a CONSTANT block, so cfg is fixed per run. Profiles
%   carrying a SimCfgTime/SimCfgVal override need that Constant swapped for a
%   From Workspace reading [SimCfgTime' SimCfgVal] with Interpolate OFF
%   (zero-order hold - linear interpolation through -1/0/1 makes no sense);
%   until then the extraction below applies the first row and warns.
% - Wall time is roughly 13 min per simulated second (R2026a, this PC).
% - Reference numbers quoted below assume the 2026-08-02 firmware (bug-fix
%   commit 8bd97de): profile-1 baseline in PFC_CONTROL_STRATEGY_REVIEW.md §2.

% Output-impedance probe segments (profile 21): +/-20 % load modulation about
% 375 W at 5 / 10 / 20 Hz, 0.2 s per tone, bracketing the 6.9 Hz voltage-loop
% crossover. Built here so the cell array below can reference them.
[pt1, pv1] = localSineLoad(0.75, 0.95, 380^2/1500*4, 0.2,  5, 5000);
[pt2, pv2] = localSineLoad(0.95, 1.15, 380^2/1500*4, 0.2, 10, 5000);
[pt3, pv3] = localSineLoad(1.15, 1.35, 380^2/1500*4, 0.2, 20, 5000);
SimProbeTime = [0, 0.75-1e-4, pt1, pt2, pt3, 1.35, 2];
SimProbeVal  = [380^2/1500*[1e5, 1e5], pv1, pv2, pv3, 380^2/1500*[1e5, 1e5]];

SimulationsProfiles = {
    % --- 1: regression baseline -----------------------------------------
    % Precharge from 0 V, soft start, 1500 W step at 0.8 s. Expected:
    % precharge done ~0.25 s, WAIT_1CYCLE 10 ms, no faults, step dip ~10 V,
    % THD ~0.35 % at full load.
    1.2 [0 0.8-1e-4 0.8 2] R*[1e5 1e5 1 1] 0 ...
        'step-1500W' 230 struct()

    % --- 2: legacy pulse load, no precharge -----------------------------
    0.5 [0 0.1-1e-5 0.1 0.3-1e-5 0.3 2] R*[1e5 1e5 1 1 1e5 1e5] Vout ...
        'pulse-no-precharge' 230 struct()

    % --- 3: precharge-timeout fault (validates PFC_FAULT_PRECHG) --------
    % Full load hangs on the bus from t=0, so the passive charge settles
    % below 0.97x peak. Expected: PFC_FAULT_PRECHG (faultStatus bit 5 = 32)
    % latches at ~1.02 s (1 s timeout + 20 ms window gating), relay stays
    % open, duty stays 0 to the end.
    1.2 [0 2] R*[1 1] 0 ...
        'precharge-timeout-fault' 230 struct()

    % --- 4: power staircase across conduction modes ---------------------
    % 60 / 200 / 375 / 750 / 1500 W, 150 ms each, from 0.75 s. Validates
    % DCM reconstruction + duty feed-forward through pure-DCM (<88 W),
    % mixed (375 W ~30 % DCM) and pure-CCM (>608 W). Judge: THD and
    % reference tracking per level over the last 5 line cycles of each
    % dwell; dcmDetected fraction should fall monotonically with power.
    1.5 [0 0.75-1e-4 0.75 0.9-1e-4 0.9 1.05-1e-4 1.05 1.2-1e-4 1.2 1.35-1e-4 1.35 2] ...
        R*[1e5 1e5 25 25 7.5 7.5 4 4 2 2 1 1] 0 ...
        'power-staircase' 230 struct()

    % --- 5: small-signal load step (validates the tuning model) ---------
    % 375 W baseline, +75 W step at 0.95 s, back at 1.15 s. The dip and
    % settling should match the voltage-loop design model (6.9 Hz
    % crossover, PM 54 deg, loadFF gain 0.9 feeding ~90 % of the step
    % forward). Compare against profile 17 (same step, loadFF off) to
    % separate the loop response from the feed-forward.
    1.3 [0 0.75-1e-4 0.75 0.95-1e-4 0.95 1.15-1e-4 1.15 2] ...
        R*[1e5 1e5 4 4 10/3 10/3 4 4] 0 ...
        'small-signal-step-375-450' 230 struct()

    % --- 6: application pulse train (the real load profile) -------------
    % 37.5 W baseline with three 80 ms pulses to 750 W. Validates bus hold
    % at light load, loadFF pulse response, and DCM<->CCM transitions each
    % pulse edge. Judge: per-pulse dip repeatability, no faults, clean
    % re-entry into DCM between pulses.
    1.35 [0 0.75-1e-4 0.75 0.85-1e-4 0.85 0.93-1e-4 0.93 1.0-1e-4 1.0 ...
          1.08-1e-4 1.08 1.15-1e-4 1.15 1.23-1e-4 1.23 2] ...
         R*[1e5 1e5 40 40 2 2 40 40 2 2 40 40 2 2 40 40] 0 ...
        'app-pulse-train' 230 struct()

    % --- 7: overload collapse + OP_UV hiccup recovery -------------------
    % 375 W baseline, 6 kW overload at 0.9 s (power clamp 1500 W cannot
    % carry it -> bus collapses), overload removed at 1.1 s. Expected:
    % OP_UV trips below 310 V, converter parks in FAULT (recovery needs
    % vdcAVG >= 320 V, unreachable under the overload), then auto-recovers
    % and re-soft-starts after 1.1 s. NOTE: recovery margin is only ~1 V
    % above the passive asymptote (review §3.6) - if this profile fails to
    % recover, that fragility is what you are seeing.
    1.4 [0 0.75-1e-4 0.75 0.9-1e-4 0.9 1.1-1e-4 1.1 2] ...
        R*[1e5 1e5 4 4 0.25 0.25 1e5 1e5] 0 ...
        'overload-opuv-hiccup' 230 struct()

    % --- 8: deep DCM + burst mode ---------------------------------------
    % 7.5 W (deep DCM, no burst) from 0.75 s, then 0.15 W from 1.0 s -
    % below PFC_MIN_POWER, so burst gating engages. Judge: regulation and
    % reconstruction at 7.5 W; burst on/off cadence and bus creep at
    % 0.15 W (review §5.5 flags the missing hysteresis - expect threshold
    % chatter, this profile is the evidence run for that improvement).
    1.3 [0 0.75-1e-4 0.75 1.0-1e-4 1.0 2] ...
        R*[1e5 1e5 200 200 1e4 1e4] 0 ...
        'deep-dcm-burst' 230 struct()

    % --- 9: low line 110 Vrms -------------------------------------------
    % Precharge to ~151 V, LONG soft-start ramp (147 V/s -> ~1.6 s), then
    % 375 W. Validates precharge fraction at low line, input-current
    % headroom (Iref clamp), and that IP_UV does not false-trip at the
    % 110 V boundary.
    2.1 [0 1.9-1e-4 1.9 3] R*[1e5 1e5 4 4] 0 ...
        'low-line-110V' 110 struct()

    % --- 10: high line 250 Vrms ------------------------------------------
    % Near the 255 V IP_OV trip without crossing it; small boost ratio.
    % Validates D_ideal ~ 0.07 operation, no IP_OV false trip, notch
    % behaviour with the larger relative ripple.
    1.0 [0 0.6-1e-4 0.6 2] R*[1e5 1e5 4 4] 0 ...
        'high-line-250V' 250 struct()

    % --- 11: line sag ride-through --------------------------------------
    % 375 W; 230 -> 160 Vrms sag at 0.9 s for 100 ms. 160 V stays above
    % the 110 V UV trip, so NO fault may occur. Judge: the Iref staircase
    % as vacRMS updates in 10 ms blocks (review §5.2 - expect one
    % half-cycle of wrong-amplitude current per update), current clamp
    % untouched (Iref pk ~3.3 A at 160 V), bus dip < ~5 V.
    1.3 [0 0.75-1e-4 0.75 2] R*[1e5 1e5 4 4] 0 ...
        'line-sag-160V' 230 ...
        struct('SimVacTime',[0 0.9-1e-4 0.9 1.0-1e-4 1.0 2], ...
               'SimVacVal',[230 230 160 160 230 230])

    % --- 12: line dropout + IP_UV fault/recovery ------------------------
    % 375 W; line to 0 V at 0.9 s for 80 ms. Expected: IP_UV trips within
    % 10-20 ms (one full RMS window below 110^2), PWM off in FAULT, bus
    % sags to ~325 V under the load (deliberately just above the 310 V
    % OP_UV trip), IP_UV clears 10-20 ms after the line returns
    % (>130 Vrms), reference reseeds at the sagged bus and soft-start
    % ramps back to 380 V by ~1.45 s. Also exercises the protection
    % ordering that keeps the Iref division safe as vacRMS collapses.
    1.5 [0 0.75-1e-4 0.75 3] R*[1e5 1e5 4 4] 0 ...
        'line-dropout-ipuv' 230 ...
        struct('SimVacTime',[0 0.9-1e-4 0.9 0.98-1e-4 0.98 3], ...
               'SimVacVal',[230 230 0 0 230 230])

    % --- 13: duty feed-forward OFF (legacy regression guard) ------------
    % 375 W steady with dutyFFEnable forced 0 for the whole run. The
    % current-loop velocity error must come back: THD ~30 %, PF ~0.93
    % (the 2026-07 numbers that motivated the feed-forward). If THD stays
    % low here, the A/B has stopped measuring what it claims to.
    1.2 [0 0.75-1e-4 0.75 2] R*[1e5 1e5 4 4] 0 ...
        'dutyff-off-legacy' 230 struct('SimCfg',[0;-1;-1;-1])

    % --- 14: duty-FF mid-run toggle (validates bug-fix 2.2) -------------
    % NEEDS the time-varying cfg wiring (see header). 375 W steady;
    % dutyFF 1->0 at 0.95 s, 0->1 at 1.1 s. Judge: the limits refresh +
    % integrator handover make both toggles near-bumpless - duty blip of
    % a few percent for a few ms, bus disturbance < ~2 V, no fault. With
    % the pre-fix firmware the 1->0 direction collapsed the bus.
    1.3 [0 0.75-1e-4 0.75 2] R*[1e5 1e5 4 4] 0 ...
        'dutyff-midrun-toggle' 230 ...
        struct('SimCfgTime',[0 0.95-1e-4 0.95 1.1-1e-4 1.1 2], ...
               'SimCfgVal',[1 -1 -1 -1; 1 -1 -1 -1; 0 -1 -1 -1; ...
                            0 -1 -1 -1; 1 -1 -1 -1; 1 -1 -1 -1])

    % --- 15: DCM reconstruction method A/B ------------------------------
    % 375 W (mixed mode, ~30 % DCM) with the legacy RATIO method forced.
    % Compare THD and dcmDetected chatter at the CCM/DCM boundary against
    % the profile-1/4 baseline (valley estimation). Edit SimCfg(2) to 0
    % for the no-reconstruction baseline (current settles below reference
    % in the DCM segments).
    1.2 [0 0.75-1e-4 0.75 2] R*[1e5 1e5 4 4] 0 ...
        'dcm-method-ratio' 230 struct('SimCfg',[-1;1;-1;-1])

    % --- 16: Vdc notch OFF (phase-margin A/B) ---------------------------
    % Same small-signal step as profile 5 with the notch disabled: the
    % voltage PI closes on the 10 ms boxcar alone, PM drops 54 -> 34 deg.
    % Judge against profile 5: visibly more ringing / slower settling on
    % the step recovery, and 100 Hz content in piVoltage.output. This is
    % the direct evidence run for the notch's phase claim in the theory
    % doc.
    1.3 [0 0.75-1e-4 0.75 0.95-1e-4 0.95 1.15-1e-4 1.15 2] ...
        R*[1e5 1e5 4 4 10/3 10/3 4 4] 0 ...
        'notch-off-step' 230 struct('SimCfg',[-1;-1;0;-1])

    % --- 17: load feed-forward OFF (loop-only step response) ------------
    % Same small-signal step as profile 5 with loadFF disabled: the slow
    % voltage loop carries the whole step. Judge: dip grows from
    % sub-volt (FF on) to the loop-only prediction ~3-4 V
    % (dP/(2*pi*fc*C*Vo) with the sensitivity peak), settling ~1/fc.
    % This is the cleanest tuning-model validation - no FF pollution.
    1.3 [0 0.75-1e-4 0.75 0.95-1e-4 0.95 1.15-1e-4 1.15 2] ...
        R*[1e5 1e5 4 4 10/3 10/3 4 4] 0 ...
        'loadff-off-step' 230 struct('SimCfg',[-1;-1;-1;0])

    % --- 18: swinging core, plant L = 0.7 x firmware L ------------------
    % Power staircase with the PLANT inductor at 476 uH while the firmware
    % keeps PFC_INDUCTANCE = 680 uH (the point: a 30 % model error, as a
    % powder core does at high current). Judge vs profile 4: the DCM
    % boundary shifts, the valley estimator biases toward late DCM
    % detection, DCM duty-FF over-delivers ~sqrt(1/0.7); THD should
    % degrade gracefully, no faults, no boundary limit-cycling.
    1.5 [0 0.75-1e-4 0.75 0.9-1e-4 0.9 1.05-1e-4 1.05 1.2-1e-4 1.2 1.35-1e-4 1.35 2] ...
        R*[1e5 1e5 25 25 7.5 7.5 4 4 2 2 1 1] 0 ...
        'swinging-core-L70' 230 struct('SimLtol',0.7)

    % --- 19: sensor tolerance worst case --------------------------------
    % Application pulse train with IL gain -5 %, IL offset +0.1 A and Vout
    % gain +2 %. With offset correction compiled out the IL offset goes
    % straight into the loop (evidence for review §4.3): expect a light-
    % load current bias and DCM-factor error; the Vout gain error makes
    % the true bus regulate at ~372.5 V while reporting 380. Judge: no
    % false OCP, no faults, quantify the light-load distortion penalty.
    1.35 [0 0.75-1e-4 0.75 0.85-1e-4 0.85 0.93-1e-4 0.93 1.0-1e-4 1.0 ...
          1.08-1e-4 1.08 1.15-1e-4 1.15 1.23-1e-4 1.23 2] ...
         R*[1e5 1e5 40 40 2 2 40 40 2 2 40 40 2 2 40 40] 0 ...
        'sensor-tol-worstcase' 230 ...
        struct('SimILGain',0.95,'SimILBias',0.1,'SimVoutGain',1.02)

    % --- 20: aged bus capacitors, C = 0.7 x nominal ---------------------
    % Full-load step with the bus capacitance down 30 % (end-of-life
    % electrolytics). Plant gain rises 1/0.7 -> crossover ~9.9 Hz, PM
    % shrinks; ripple grows to ~12.7 Vpp at 1.5 kW. Judge: step dip ~14 V
    % (still >> above OP_UV), recovery stays damped (no sustained
    % ringing), notch still nulls the 100 Hz (it is line-derived, not
    % C-derived). This is the aging-margin evidence run.
    1.2 [0 0.8-1e-4 0.8 2] R*[1e5 1e5 1 1] 0 ...
        'cap-aged-C70' 230 struct('SimCtol',0.7)

    % --- 21: closed-loop output-impedance probe -------------------------
    % +/-20 % sinusoidal load modulation about 375 W at 5, 10, 20 Hz
    % (0.2 s per tone), loadFF OFF so the loop alone answers. Judge: the
    % Vdc component at each tone gives |Zout_cl|; compare against the
    % design model 1/(s*C*Vo) shaped by the sensitivity function - the
    % peak should sit near the 6.9 Hz crossover with height set by
    % PM 54 deg. THE quantitative check that the tuning model matches the
    % implemented loop.
    1.45 SimProbeTime SimProbeVal 0 ...
        'output-impedance-probe' 230 struct('SimCfg',[-1;-1;-1;0])
};

SimIndex = 12;

SimTime     = SimulationsProfiles{SimIndex, 1};
SimLoadTime = SimulationsProfiles{SimIndex, 2};
SimLoadVal  = SimulationsProfiles{SimIndex, 3};
SimCoutInit = SimulationsProfiles{SimIndex, 4};
SimName     = SimulationsProfiles{SimIndex, 5};
SimVacRms   = SimulationsProfiles{SimIndex, 6};

% ---- defaults for the wired-in extras (per-profile overrides in column 7) --
% Time-varying input voltage [Vrms] (From Workspace2 in the plant).
SimVacTime = SimLoadTime;
SimVacVal  = SimVacRms*ones(size(SimVacTime));

% SimCfg: [pfcParam.dutyFFEnable; pfcParam.sampleCorrectionEnable;
%          pfcParam.vdcNotch.enable; pfcParam.loadFF.enable]
% -1: firmware default; 0: force off; 1: force on. Static Constant wiring.
SimCfg = [-1; -1; -1; -1];
% Time-varying cfg (only once the Constant is swapped for a From Workspace
% with Interpolate OFF - see the header). Empty = static SimCfg applies.
SimCfgTime = [];
SimCfgVal  = [];

% Plant-tolerance factors: inductor L -> L*SimLtol, bus cap C -> C*SimCtol.
% The firmware's compiled-in PFC_INDUCTANCE does NOT follow - that is the
% point (model-error robustness).
SimLtol = 1;
SimCtol = 1;

% Sensor gain and bias perturbations (Signal_Aquisition blocks).
SimILGain = 1;
SimILBias = 0;
SimVinGain = 1;
SimVinBias = 0;
SimVoutGain = 1;
SimVoutBias = 0;
SimIoutGain = 1;
SimIoutBias = 0;

% ---- apply the per-profile overrides ---------------------------------------
ov = SimulationsProfiles{SimIndex, 7};
ovNames = fieldnames(ov);
for kOv = 1:numel(ovNames)
    % Only variables already defined above may be overridden - a typo in a
    % profile fails loudly here instead of silently doing nothing.
    assert(exist(ovNames{kOv}, 'var') == 1, ...
        'SimulationProfile: unknown override "%s" in profile %d (%s)', ...
        ovNames{kOv}, SimIndex, SimName);
    eval([ovNames{kOv} ' = ov.(ovNames{kOv});']);
end

if ~isempty(SimCfgTime)
    % Static-Constant fallback: apply the initial cfg row and say so. Swap
    % the Constant for a From Workspace [SimCfgTime' SimCfgVal] (Interpolate
    % OFF) to make the mid-run changes real.
    SimCfg = SimCfgVal(1,:).';
    warning(['SimulationProfile: profile ''%s'' contains a mid-run cfg ' ...
             'change; with the static Constant wiring only the initial ' ...
             'value %s applies.'], SimName, mat2str(SimCfg.'));
end

% Local helper for the output-impedance probe: R(t) = R0/(1 + depth*sin) so
% the drawn POWER (V^2/R) modulates as +depth*sin at constant bus voltage.
% Segments are half-open ([t0, t1)) so consecutive tones can share t1/t0
% without duplicating breakpoints.
function [tt, vv] = localSineLoad(t0, t1, R0, depth, f, fs)
    tt = t0 : 1/fs : t1 - 1/fs;
    vv = R0 ./ (1 + depth*sin(2*pi*f*(tt - t0)));
end
