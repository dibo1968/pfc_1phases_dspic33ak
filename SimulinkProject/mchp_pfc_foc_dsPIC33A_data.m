%% ************************************************************************
% Model         :   Single-phase boost Power Factor Correction
% Description   :   Plant parameters and control-loop gain derivation for the
%                   single-stage boost PFC. Sets up the workspace for
%                   pfc_simulation.slx and documents where every gain in
%                   project/pfc/pfc_userparams.h comes from.
%
%                   The PMSM / FOC motor and inverter parameters that this
%                   file used to carry have been removed - this project is
%                   PFC only.
%
% File name     :   mchp_pfc_foc_dsPIC33A_data.m
% Copyright 2026 Microchip Technology Inc.
%
% NOTE: this script emits no firmware header. The constants in
% pfc_userparams.h are maintained by hand; the cross-check at the bottom is
% what keeps the two from drifting apart silently.
%% ************************************************************************

%% Simulation set-up
clc
clear all
close all
s = tf('s');

%% Switching frequency and sample times
fsw             = 64e3;             % Hz  // converter switching = control freq
T_pwm           = 1/fsw;            % s   // PWM period
Ts              = T_pwm;            % s   // controller simulation time step

% Voltage/current loop execution ratios. Isr/Osr are in ISR ticks and must
% track VOLTAGE_LOOP_EXE_RATE in pfc_userparams.h.
% Osr was 10, but the firmware executes the voltage PI every 12 ISRs
% (VOLTAGE_LOOP_EXE_RATE = 12). Corrected 2026-08-02 so Tsv describes what
% actually runs; see the fz_v note below.
Isr             = 1;
Osr             = 12;
Tsi             = Isr / fsw;        % s   // current-loop execution period
Tsv             = Osr / fsw;        % s   // voltage-loop execution period

%% Set data type for controller & code-gen
dataType        = 'single';

%% Circuit parameters
Po          = 1500;                 % W   // rated output power
Pout_max    = Po;                   % W   // kept as an alias, cannot drift
Vout        = 380;                  % V   // regulated bus voltage
Vin_rms     = 230;                  % V   // nominal input [110 - 250 V range]
Vin_pk      = Vin_rms*sqrt(2);      % V   // input peak
Vrms_min    = 200;                  % V   // minimum input rms used for Kmul
Vpk_min     = Vrms_min*sqrt(2);     % V   // minimum input peak used for Kmul
Iin_max_rms = 10;                   % A   // maximum input current rms
Iin_max_pk  = Iin_max_rms*sqrt(2);  % A   // maximum input current peak
Max_Duty    = 0.95;                 % --  // duty clamp, = PFC_MAX_DUTY

Vac_Frequency   = 50;               % Hz  // line frequency, = PFC_INPUT_FREQUENCY
Vac_TimePeriod  = 1/Vac_Frequency;  % s   // line period
Vac_Samples     = Vac_TimePeriod/T_pwm;  % ADC samples per line period
                                         % = PFC_INPUT_FREQUENCY_COUNTER (1280)

L = 680e-6;                         % H   // boost inductor, = PFC_INDUCTANCE
C = 3*470e-6;                       % F   // bus capacitance (3 x 470 uF)
R = (Vout^2)/Po;                    % ohm // equivalent full-load resistance

%% Protection thresholds
% The input tests are pre-squared because the firmware compares against
% vacRMS.sqrOutput and never takes a square root. Trip/recovery pairs give
% the hysteresis. CORRECTED 2026-08-02: Output_OV_TH was 400 V and
% Input_OV_TH_HIGH was 250 V, neither of which matched the firmware (410 V
% and 255 V). The firmware is the source of truth - it is what ships.
Input_UV_TH_LOW   = 110*110;        % = PFC_INPUT_UNDER_VOLTAGE_LO  (trip)
Input_UV_TH_HIGH  = 130*130;        % = PFC_INPUT_UNDER_VOLTAGE_HI  (recovery)

Input_OV_TH_LOW   = 240*240;        % = PFC_INPUT_OVER_VOLTAGE_LO   (recovery)
Input_OV_TH_HIGH  = 255*255;        % = PFC_INPUT_OVER_VOLTAGE_HI   (trip)

Output_OV_TH      = 410;            % V // = PFC_OUTPUT_OVER_VOLTAGE  (trip)
Output_UV_TH      = 310;            % V // = PFC_OUTPUT_UNDER_VOLTAGE (trip)

%% Base values for gains
% All unity: the firmware works in engineering units (volts, amps, watts),
% so no per-unit normalisation is applied anywhere in the loop maths.
Ibase = 1;
Vbase = 1;

KiL  = 1/Ibase;                     % current feedback gain
Kvo  = 1/Vbase;                     % bus voltage feedback gain
Kvin = 1/Vbase;                     % input voltage feedback gain
Kmul = 1;                           % multiplier scaling, = KMUL

%% Control system targets
fc_i = fsw/10;                      % Hz // current-loop crossover target
% fc_v was 12 Hz, but that figure was produced by a voltage plant carrying a
% spurious factor of 2 (see below). With the plant corrected, the gain that
% ships as KP_V = 20.199 corresponds to a nominal crossover of 6 Hz.
% Corrected 2026-08-02. NOTE this is the crossover of the PROPORTIONAL term
% against the plant; the PI zero adds magnitude near crossover, so the true
% loop crossing is a little higher - 6.90 Hz once the Vdc filter is included.
fc_v = 6.0;                         % Hz // voltage-loop crossover target

%% Soft start: ramping the voltage-loop reference
% Simulation only. The firmware ramps with RAMP_COUNT / RAMP_RATE instead.
Vref      = Vout;
Vout_init = Vin_pk;                 % bus starts at the rectified input peak
T_rise    = 0.20;                   % s
Slope     = (Vref-Vout_init)/T_rise;

%% Current loop: inductor current transfer function [ IL / d ]
Gid = Vout/(s*L);

% Current loop compensator
fz_i = fc_i/10;                     % Hz // PI zero, a decade below crossover

Gci  = (2*pi*fc_i*L)/(KiL*Vout);

Kp_i = Gci;
Ki_i = Kp_i*2*pi*fz_i;

Gic_comp = Kp_i+Ki_i/s;
Gicl     = Gid*Gic_comp*KiL;        % open-loop current gain; try margin(Gicl)

%% Voltage loop: power command to bus voltage [ Vo / P ]
% CORRECTED 2026-08-02. This was Gvc = (2*Kmul)/(Kvin*KiL*C*Vout*s), which is
% 2x the physical plant. Linearising the bus energy balance:
%       (1/2)*C*d(Vo^2)/dt = Pin - Pload   ->   C*Vo*dvo/dt = p
%       vo(s)/p(s) = 1/(s*C*Vo)
% The factor 2 belongs to the SQUARED-voltage form, Vo^2(s)/P(s) = 2/(s*C),
% but converting that back to Vo requires dividing by 2*Vo, not Vo - so the 2
% was left behind in a plant that should not carry it. That the loop variable
% really is watts is confirmed by PI_V_OUT_MAX = 1500 = Po and by the
% multiplier Iref = P*vac/Vrms^2, which makes mean(vac*iL) = P identically.
% Validated against the measured bus ripple: 2.23 Vpp at 375 W.
Gvc = Kmul/(Kvin*KiL*C*Vout*s);

% Voltage loop compensator
% fz_v was 5 Hz, chosen when Osr was 10. The firmware runs the PI every 12
% ISRs but its per-tick integral gain KI_V was never rescaled, so the zero the
% hardware actually realises is 5*(10/12) = 4.167 Hz. That is DELIBERATE and
% must not be "corrected" by raising KI_V: the lower zero is worth about 4 deg
% of phase margin. Expressed against Osr so KI_V below stays invariant.
fz_v = 5*(10/Osr);                  % Hz // effective PI zero

% The matching factor 2 is removed from the compensator, since it existed only
% to cancel the one in Gvc above. Kp_v is unchanged at 20.199.
Gcv  = (C*Vout*2*pi*fc_v*Kvin*KiL)/(Kvo*Kmul);

Kp_v = Gcv;
Ki_v = Kp_v*2*pi*fz_v;

Gvc_comp = Kp_v+Ki_v/s;
Gvcl     = (Gvc*Gvc_comp*Kvo);      % open-loop voltage gain; try margin(Gvcl)

%% ------------------------------------------------------------------------
%  Cross-check against the firmware
%
%  This script emits nothing, so the only thing stopping it drifting away
%  from project/pfc/pfc_userparams.h is this table. The right-hand column is
%  transcribed from the firmware headers; if a row fails, one of the two
%  files was changed without the other.
%
%  The PI gains are compared in their DISCRETE form: the firmware integrates
%  as integralOut += ki*error once per execution, so ki_firmware = Ki*T_exec.
%  The current-loop gains ship at HALF the derived values - the continuous
%  design ignores ~1.5 sample periods of digital delay, and halving moves the
%  crossover 6.4 kHz -> 3.2 kHz to buy the phase margin back - which is why
%  Kp_i/2 and Ki_i*Tsi/2 are what get compared.
% -------------------------------------------------------------------------
check = { ...
    'KP_I',                        Kp_i/2,                  0.036
    'KI_I',                        Ki_i*Tsi/2,              0.0022607
    'KP_V',                        Kp_v,                    20.199
    'KI_V',                        Ki_v*Tsv,                0.0992
    'PFC_PWMFREQUENCY_HZ',         fsw,                     64e3
    'VOLTAGE_LOOP_EXE_RATE',       Osr,                     12
    'PFC_INDUCTANCE',              L,                       680e-6
    'PFC_INPUT_FREQUENCY',         Vac_Frequency,           50
    'PFC_OUPUT_VOLTAGE_NOMINAL',   Vout,                    380
    'PFC_MAX_DUTY',                Max_Duty,                0.95
    'PFC_INPUT_FREQUENCY_COUNTER', Vac_Samples,             1280
    'PFC_INPUT_UNDER_VOLTAGE_LO',  sqrt(Input_UV_TH_LOW),   110
    'PFC_INPUT_UNDER_VOLTAGE_HI',  sqrt(Input_UV_TH_HIGH),  130
    'PFC_INPUT_OVER_VOLTAGE_LO',   sqrt(Input_OV_TH_LOW),   240
    'PFC_INPUT_OVER_VOLTAGE_HI',   sqrt(Input_OV_TH_HIGH),  255
    'PFC_OUTPUT_OVER_VOLTAGE',     Output_OV_TH,            410
    'PFC_OUTPUT_UNDER_VOLTAGE',    Output_UV_TH,            310 };

fprintf('\n  PFC cross-check: this script vs the firmware headers\n');
fprintf('  %-28s %13s %13s %9s\n', 'constant', 'script', 'firmware', 'delta');
nbad = 0;
for k = 1:size(check,1)
    name = check{k,1};  got = check{k,2};  want = check{k,3};
    d = 100*(got/want - 1);
    fprintf('  %-28s %13.7g %13.7g %8.2f %%\n', name, got, want, d);
    if abs(d) > 0.5
        nbad = nbad + 1;
        warning('%s differs from the firmware by %.2f %% - reconcile before use.', ...
                name, d);
    end
end
if nbad == 0
    fprintf('  all consistent (sub-0.1%% deltas are rounding in the header literals)\n');
end
fprintf(['\n  Voltage loop: nominal fc %.2f Hz, PI zero %.3f Hz.\n' ...
         '  True crossing is 6.90 Hz and PM 54 deg WITH the 100 Hz notch on\n' ...
         '  the Vdc feedback; without it PM is only 34 deg. Neither the notch\n' ...
         '  nor the 10 ms bus average is modelled above, so margin(Gvcl) will\n' ...
         '  read optimistically - see PFC_CONTROL_THEORY.md sections 4.5-4.6.\n\n'], ...
        fc_v, fz_v);
