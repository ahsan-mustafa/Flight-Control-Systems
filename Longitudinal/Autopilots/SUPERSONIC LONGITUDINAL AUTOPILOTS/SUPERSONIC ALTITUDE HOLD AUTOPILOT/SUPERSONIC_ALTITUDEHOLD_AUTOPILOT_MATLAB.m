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
CD0 = 0.0480; CDu = -0.054; CDalpha = 0.400; 
CL0 = 0.1700; CLu = -0.180; CLalpha = 2.800; CLalphaDot = 0.170; CLq = 1.30; CLdelta_e = 0.2500;
CM0 = -0.025; CMu = 0.0540; CMalpha = -0.78; CMalphaDot = -0.25; CMq = -2.0; CMdelta_e = -0.380; 

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
system = ss(A,B,C,D);                                    % State Space
G = tf(system);                                          % Transfer Functions
AOA_TF = (1/u0)*G(2,1);                                  % AOA Transfer Function 
PitchRate_TF = G(3,1);                                   % Pitch Rate Transfer Function
PitchAttitude_TF = G(4,1);                               % Pitch Attituded Transfer Function
Characteristic_Equation = poly(A)                        % Characteristic Equation Coefficients
damp(A)                                                  % Unaugmented Mode Characteristics
SYSTEM = [AOA_TF; PitchAttitude_TF; PitchRate_TF];       % System defined for Simulink Model

% FROM SIMULINK MODEL
disp('-----------------------------SIMULINK STEP & IMPULSE RESPONSES------------------------------------')
Integrator = tf(1,[1 0]);                                           % Integrator Transfer Function
Servo_TF = tf(10,[1 10]);                                           % Servo Transfer Function
Altimeter_TF = tf(1,[1 1]);                                         % Altimeter Transfer Function
MAIN_TF = Servo_TF * SYSTEM;                                        % System Transfer Functions Multiplied with Servo Transfer Function    
Kq = 0.852412043994715;                                             % Gain obtained from Simulink model and CSD
Ktheta = 1;                                                         % Gain obtained from Simulink model and CSD
Kh = -0.000106442901254868;                                         % Gain obtained from Simulink model and CSD
Inner_Loop = feedback(MAIN_TF,[0, Ktheta, Kq],+1);                  % Inner Loop of Simulink Model
Altitude_TF = (Inner_Loop(2) - Inner_Loop(1)) * u0 * Integrator;    % Altitude Transfer Function
Path_1 = Kh * Altitude_TF;                                          % Straight Path of Simulink Model
Outer_Loop = feedback(Path_1,Altimeter_TF,-1);                      % Full Closed Loop of Entire System
step(Outer_Loop)                                                    % Step Response of Full Closed Loop of Entire System
StepInfo = stepinfo(Outer_Loop,'SettlingTimeThreshold',0.05)        % Step Info
title('Step Response Without PD controller')
figure
impulse(Outer_Loop)                                                 % Impulse Response of Full Closed Loop of Entire System with PD Controller
title('Impulse Response Without PD controller')
figure
Kp = 0.702446637657272;                                             % Gain obtained from Simulink model and CSD
Ki = 0;                                                             % Gain obtained from Simulink model and CSD
Kd = 2.92686099023863;                                              % Gain obtained from Simulink model and CSD
C = pid(Kp,Ki,Kd);                                                  % PD Controller
Path_2 = C * Kh * Altitude_TF;                                      % Straight Path of Simulink Model
Outer_Loop_PD = feedback(Path_2,Altimeter_TF,-1);                   % Full Closed Loop of Entire System with PD Controller
step(Outer_Loop_PD,150)                                             % Step Response of Full Closed Loop of Entire System with PD Controller
StepInfo_PD = stepinfo(Outer_Loop_PD,'SettlingTimeThreshold',0.05)  % Step Info
title('Step Response With PD controller')                           
figure
impulse(Outer_Loop_PD)                                              % Impulse Response of Full Closed Loop of Entire System with PD Controller
title('Impulse Response With PD controller')
damp(Outer_Loop_PD)