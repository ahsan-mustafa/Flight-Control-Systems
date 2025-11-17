% CODE FOR SIMULINK MODEL - APPROACH (FLIGHT CONDITION)
clear
clc
% FlIGHT CONDITIONS
M = 0.206;   % Approach mach number
u0 = 230;    % True airspeed
Q = 62.9;    % Dynamic pressure
g = 32.2;    % Gravitational acceleration

% AIRCRAFT PARAMETERS
S = 530;     % Wing surface area
cbar = 16;   % Mean aerodynamic chord
b = 38.7;    % Wing Span
W = 33200;   % Weight of Aircraft
m = W/g;     % Mass of aircraft
Iy = 117500; % Moment of inertia Y-Axis

% NON-DIMENSIONAL DERIVATIVES
CD0 = 0.2000; CDalpha = 0.5550; 
CL0 = 1.0000; CLalpha = 2.8000; CLalphaDot = 0.630; CLq = 1.33; CLdelta_e = 0.2400;
CM0 = -0.020; CMalpha = -0.098; CMalphaDot = -0.95; CMq = -2.0; CMdelta_e = -0.322; 
CDu = 0;
CMu = 0;
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
Xu = ((Q*S)/(m*u0)) * ((2*CX0) + CXu);               % Xu
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
AOA_TF = G(2,1)*(1/u0);                                      % AOA Transfer Function
PitchRate_TF = G(3,1);                                       % Pitch Rate Transfer Function
PitchAttitude_TF = G(4,1);                                   % Pitch Attitude Transfer Function
Characteristic_Equation = poly(A)                            % Characteristic Equation Coefficients
damp(A)                                                      % Unaugmented Mode Characteristics
Sys1 = [PitchRate_TF ; AOA_TF ]                              % System defined for Simulink Model

controlSystemDesigner('CSD_Approach_PitchHold.mat')          % Root Locus of Final Pitch Hold Autopilot showing all the handling qualities requirements