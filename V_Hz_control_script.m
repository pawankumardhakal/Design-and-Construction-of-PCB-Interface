% Induction machine parameters
Pn  = 1.5e3;     % W,  rated power
Vn  = 400;       % V,  rms phase-to-phase, rated voltage
fn  = 50;        % Hz, rated frequency
Rs  = 12.56;   % stator resistance
Lls = 0.0489;   % stator leakage inductance
Rr  = 11.97;   % rotor resistance, referred to the stator side
Llr = 0.0466;   %rotor leakage inductance, referred to the stator side
Lo  = 1.028;    % magnetizing inductance
Lr = Llr+Lo;    % rotor inductance
Ls = Lls+Lo;    % stator inductance
p = 2;       % pole pairs

%% Control parameters
V_to_Hz=6.5;    %constant
Ts = 1e-6;     % fundamental sample time
fsw = 10e3;    % switching frequency 
Tsc = 1/(fsw*20); % control sample time
