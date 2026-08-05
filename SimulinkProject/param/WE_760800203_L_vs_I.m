% WE-TORPFC 760800203 (T50) - typical differential inductance vs DC bias
% Digitized from datasheet rev 001.001 (2023-12-11), p.2, 'Typical Inductance vs. Current'.
% Curve fit:  L(I) = Lmin + (L0-Lmin)/(1+(I/a)^b)   [uH],  valid 0..85 A, typical part, ~25 degC.
% Fit residual vs digitized curve: rms 0.7 uH, max 2.2 uH (< plotted line width).
% Part tolerance is +/-20 % on L0 -- scale accordingly for worst-case studies.

L0   = 600.0;    % uH, zero-bias inductance (plot start; datasheet nominal 584 uH +/-20%)
Lmin = 9.979;   % uH, deep-saturation asymptote
a    = 30.168;  % A
b    = 1.582;   % -

Ltab = @(i) 1e-6*(Lmin + (L0-Lmin)./(1+(abs(i)/a).^b));   % H, differential inductance

% Lookup-table form (1 A grid) for Simulink 1-D Lookup Table blocks:
I_A    = 0:1:85;
L_diff = Ltab(I_A);              % H
% Flux linkage lambda(i) = int_0^i L_diff di'  -- use this for a physically consistent
% nonlinear-inductor model (v = dlambda/dt).  Odd-symmetric extension for i<0.
lam    = cumtrapz(I_A, L_diff);  % Wb (V.s)
L_sec  = [L0*1e-6, lam(2:end)./I_A(2:end)];   % H, secant (energy-equivalent) inductance
