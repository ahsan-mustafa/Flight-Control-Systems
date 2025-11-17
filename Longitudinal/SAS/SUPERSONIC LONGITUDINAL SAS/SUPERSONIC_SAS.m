% SUPERSONIC - FLIGHT CONDITION
clear
clc
% FlIGHT CONDITIONS
M = 1.8;     % Approach mach number
u0 = 1742;   % True airspeed
Q = 434.5;   % Dynamic pressure
g = 32.2;    % Gravitational acceleration
 
% AIRCRAFT PARAMETERS
S = 530;     % Wing surface area
cbar = 16;   % Mean aerodynamic chord
b = 38.7;    % Wing Span
m = 1212.16; % Mass of aircraft in slugs
Iy = 122200; % Moment of inertia Y-Axis

% NON-DIMENSIONAL DERIVATIVES
CD0 = 0.0480; CDalpha = 0.400; 
CL0 = 0.1700; CLalpha = 2.800; CLalphaDot = 0.170; CLq = 1.30; CLdelta_e = 0.2500;
CM0 = -0.025; CMalpha = -0.78; CMalphaDot = -0.25; CMq = -2.0; CMdelta_e = -0.380; 

CDu = -0.054;
CMu = 0.0540;
CLu = ((M^2)/(1-M^2)) * CL0;

CX0 = 0; CZ0 = 0; CTu = 0;
CLm = 0; CDm = 0; CMm = 0;

CXu = -(CDu + (2*CD0)) + CTu;
CZu = -(CLu + (2*CL0));

CXalpha = CL0 - CDalpha; 
CZalpha = -(CLalpha + CD0);

CXdelta_e = 0;
CZdelta_e = -CLdelta_e;  

% DIMENSIONAL DERIVATIVES
Xu = ((Q*S)/(m*u0)) * ((2*CX0) + CXu)               % Xu
Zu = ((Q*S)/(m*u0)) * ((2*CZ0) + CZu);               % Zu

Xw = ((Q*S)/(m*u0)) * CXalpha;                       % Xw
Zw = ((Q*S)/(m*u0)) * CZalpha;                       % Zw

Mu = ((Q*S*cbar)/(Iy*u0)) * CMu;                     % Mu
Mw = ((Q*S*cbar)/(Iy*u0)) * CMalpha;                 % Mw
Mq = ((Q*S*(cbar^2))/(2*Iy*u0)) * CMq;               % Mq

Malpha = u0*Mw;                                      % Malpha
Zalpha = u0*Zw;                                      % Zalpha

MwDot = ((Q*S*(cbar^2))/(2*Iy*(u0^2))) * CMalphaDot; % MwDot
MalphaDot = u0*MwDot;                                % MalphaDot

Xdelta_e = 0;                                        % Xdelta_e  
Zdelta_e = (CZdelta_e*S*Q)/m;                        % Zdelta_e
Mdelta_e = (Q*S*cbar*CMdelta_e)/Iy;                  % Mdelta_e
  
% UNAUGMENTED LONGITUDINAL DYNAMICS
disp('-----------------------------UNAUGMENTED LONGITUDINAL DYNAMICS------------------------------------')
A = [Xu Xw 0 -g; Zu Zw u0 0; (Mu+(MwDot*Zu)) (Mw+(MwDot*Zw)) (Mq+(MwDot*u0)) 0; 0 0 1 0]; % Matrix A 
B = [Xdelta_e 0; Zdelta_e 0; (Mdelta_e+(MwDot*Zdelta_e)) 0; 0 0];                         % Matrix B
C=[1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];                                                   % Matrix C
D=[0 0; 0 0; 0 0; 0 0];                                                                   % Matrix D
SYSTEM = ss(A,B,C,D);                                        % State Space
G = tf(SYSTEM);                                              % Transfer Functions
PitchRate_TF = G(3,1);                                       % Pitch Rate Transfer Function
figure
step(PitchRate_TF)                                           % Open Loop Step Response for Pitch Rate Transfer Function 
PitchRate_TF_StepInfo = stepinfo(PitchRate_TF)               % Step Response Info for Pitch Rate Transfer Function
title 'Open Loop Step Response for Pitch Rate Transfer Function'
Characteristic_Equation = poly(A)                            % Characteristic Equation Coefficients
damp(A)                                                      % Unaugmented Mode Characteristics
Sys1 = [PitchRate_TF];                                       % System defined for Simulink Model

% PITCH RATE FEEDBACK -- 'q'
disp('-----------------------------PITCH RATE FEEDBACK------------------------------------')
controlSystemDesigner('CSD_SUPERSONIC.mat')                  % Root Locus to Find Approprite Gain
K = 0.815;                                                   % Gain Obtained
CL_PitchRate_TF = feedback(PitchRate_TF,K,+1);               % Closed Loop Pitch Rate Transfer Function
[num den] = tfdata(CL_PitchRate_TF)                          % Taking Denominator for Characteristic Equation
damp(CL_PitchRate_TF)                                        % Mode Characteristics After Pitch Rate Feedback
figure
step(CL_PitchRate_TF)                                        % Closed Loop Step Response for Pitch Rate Transfer Function After Pitch Rate Feedback
CL_PitchRate_TF_StepInfo = stepinfo(CL_PitchRate_TF)         % Closed Loop Step Response Info for Pitch Rate Transfer Function After Pitch Rate Feedback
title 'Closed Loop - Pitch Rate TF After ''q'' Feedback Only'
Characteristic_Equation = den                                % Characteristic Equation Coefficients
