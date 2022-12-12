clc;
clear;
%given parameters
VLL = 400; %%supply voltage in volts
Is = 3.3; %%rated current in amps
Pr = 1.5e3;  %%rated power in watts;
Wr = 1435; %%rated speed in RPM
T_rated = Pr/(Wr*2*pi/60); %%rated torque in Nm
Eff_100 = 0.842; %%efficiency at 100% loading
Eff_75 = 0.841; %%efficiency at 75% loading
Eff_50 = 0.819; %%efficiency at 100% loading
pf = 0.76; %%power factor full load 100%
I_start = 7*Is; %%starting current in amps
T_start = 3.1*T_rated; %%starting torque in Nm
T_max = 3.5*T_rated;  %%pullout torque in Nm
p = 2; %%no of pole pairs 
f = 50; %%frequency in Hz
weight = 17; %%weight of motor in Kg
%Delta connected
Vphase=400;
Iphase= Is/sqrt(3);

% parameters calculation
IR = Iphase*pf; %%rotor current
Io = Iphase*sin(acos(pf)); %%magnetizing current
Ws = 60*f/(p); %%synchronous speed (rev/min)
S = (Ws-Wr)/Ws; %%slip
We = 2*pi*f; %%electrical speed (rad/sec)
Lo = Vphase/(We*Io); %%magnetizing inductance
RR = Vphase*S/IR; %%rotor resistance

%to calculate Rs we need efficiency at 100% loading and 50% loading
%current at 50% loading
I_50 = sqrt((Iphase*sin(acos(pf)))^2+((Iphase*pf)/2)^2);

%%%@ 100% load, 3*Is^2*(Rs+Rr)+C = ((1-eff@100%)/eff@100%)*Pr @100%
%%%@ 50% load, 3*I_50^2*(Rs+Rr)+C = ((1-eff@50%)/eff@50%)*Pr @50%
Sum1 = ((1-Eff_100)/Eff_100)*Pr;
Sum2 = ((1-Eff_50)/Eff_50)*Pr*0.5;
Diff1 = Sum1-Sum2;
Diff2 = 3*Iphase^2-3*I_50^2;
Sumofresistances = Diff1/Diff2;

%Stator Resistance
RS = Sumofresistances - RR;

%calculation of leakage inductances%%
%%% @start, Vs = jWe*(ls+lr)*I_start
%%  ls/lr = Rs/Rr
var1 = Vphase/(We*I_start/sqrt(3)); %%ls+lr
ratio = RS/RR; %%ls/lr
llR = var1/(1+ratio); %%rotor lekage inductance
llS = var1-llR;  %%stator leakage inductance
k=1/ratio;

%%Rotor and stator Self Inductances%%

LS = Lo+llS ; %%stator self inductance
LR = Lo+llR ; %%rotor self inductance

%%leakage factors %%

sigma_s = llS/Lo; %% stator leakage factor
sigma_r = llR/Lo; %%rotor leakage factor
sigma = 1-(1/(1+sigma_s + sigma_r)); %%total leakage factor

%% Time Constants %%
tau_s = sigma*LS/RS; %%stator time constant%%
tau_r = LR/RR;  %% rotor time constant %%

%% Using RMS convention %%


Isd = Io; %%d-axis current at steady state%%
Isq = IR; %%q-axis current at steady state%%
fi_R = Lo*Isd; %%rotor flux
T_calculated = 3*(2*p/2)*(Lo/LR)*Isq*fi_R;

%%

P = 4; %no. of poles
tau_s = sigma*LS/RS;
tau_r = LR/RS;

Current_Wn = 200; %Current BW in Hz
Ts=1e-5;


%Q-axis current controller Using pole zero cancellation
%Controller of the form (Kp+Ki/s) for the plant of (1/Rs)/(1/RS)+s*tau_s))%%
Ki=2*pi*Current_Wn*RS;
Kp= 2*pi*Current_Wn*LS*sigma;

s = tf('s');
PIq = Kp+Ki/s;

%%speed controller
J=0.1;
B=0;
Speed_Wn= Current_Wn/20;
Ki_speed=2*pi*Speed_Wn*B;
Kp_speed= 2*pi*Speed_Wn*J;
%%%butter worth filter design
fc=4000; %cutoff frequency
fs=10000; %sampling frequency
[b,a] = butter(2,fc/(fs));
Filter=tf(b,a);








