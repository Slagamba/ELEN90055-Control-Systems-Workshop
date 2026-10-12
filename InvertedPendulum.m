%% ELEN90074 Control Systems
% Workshop 4: Inverted Pendulum
clc
clear
close all

sympref('FloatingPointOutput', true);
%% Experimentally determined Plant parameters
wd = 1.93*2*pi; % Damped natural freq in rad/s
zeta = 0.33; % Damping ratio
wn = wd*sqrt(1 - (zeta^2)); % Natural Freq

K = sqrt(wn); % Estimated Plant Value
c = 0.7*K; % Estimated Plant Value
b = 2*zeta*wn; % Estimated Plant Value


%% Lead compensator parameters

% Set both tau to zero is not using
tau1 = 0.0694; % Zero location 
tau2 = 0.1*tau1; % Pole location

C_lead_num = [tau1 1];
C_lead_den = [tau2 1];

C_lead = tf(C_lead_num, C_lead_den);

tau3 = 0.1*tau2; 

C_lag_num = [tau2 1];
C_lag_den = [tau3 1];

C_lag = tf(C_lag_num, C_lag_den);

% Controller is lead compensator if tau1 > tau2

%% PID Controller parameters
Kp = 150; % Proportional Gain 
Ki = 0.5; % Integral Gain
Kd = 0.1; % Derivative Gain

% Good candidate PI controller for Kp = 100, Ki = 0.1

%% Simulink stuff
Ts = 0.05; % Set Sample Period
To = 2*Ts;

%% Defining disturbances
ImpulseMag = 30; % Angle in degrees of impulse disturbance
w_car = pi/3; % Angular Velocity in rad/s of car steering

%% Plant Transfer Function

% Denote G as the inverted pendulum TF
G0_num = [0 0 c]; % Numerator Coefficients
G0_den = [1 b -K]; % Denominator Coefficients
G0 = tf(G0_num, G0_den);
G0_num_poly = poly2sym(G0_num);
G0_den_poly = poly2sym(G0_den);

%% Controller Transfer Function

% Denote C as the controller TF
C_PI_num = [0 Kp Ki]; % Numerator Coefficients
C__PI_den = [0 1 0]; % Denominator Coefficients

C_PI = tf(C_PI_num, C__PI_den);

% Compensator in series with PI controller
C = C_PI*C_lead*C_lag_den;

[C_num, C_den] = tfdata(C); % Full Controller Numerator/Denominator
C_num_poly = poly2sym(cell2mat(C_num));
C_den_poly = poly2sym(cell2mat(C_den));


%% Characteristic Equation

CE = G0_den_poly*C_den_poly + G0_num_poly*C_num_poly;
CE_roots = roots(sym2poly(CE));

%% Sensitivity Functions
% Define the Complementary Sensitivity TF
T0 = (G0*C)/(1 + (G0*C));
disp('Complementary Sensitivity T0:')
disp(zpk(T0))
disp(newline)

% Define the Output Sensitivity S0
S0 = 1/(1 + (G0*C));
disp('Output Sensitivity S0:')
disp(zpk(S0))
disp(newline)

% Define the Input Disturbance Sensitivity Si0
Si0 = G0/(1 + (G0*C));
disp('Input Disturbance Sensitivity Si0:')
disp(zpk(Si0))
disp(newline)

% Define the Control Sensitivity Su0
Su0 = C/(1 + (G0*C));
disp('Control Sensitivity Su0:')
disp(zpk(Su0))

%% Specifications

% Find the Bandwidth of the Complementary Sensitivity TF
bw_T0 = bandwidth(T0);

% Find the Gain/Phase Margins, and their corresponding frequencies
[Gm, Pm, Wcg, Wcp] = margin(C*G0);

%% Plotting

figure
bode(T0)
title('Bode Plot of Complementary Sensitivity T0')

figure
bode(S0)
title('Bode Plot of Output Sensitivity S0')

figure
bode(Si0)
title('Bode Plot of Input Disturbance Sensitivity Si0')

figure
bode(Su0)
title('Bode Plot of Control Sensitivity Su0')

figure
margin(G0*C)
title('Gain and Phase Margins')

figure
%}





