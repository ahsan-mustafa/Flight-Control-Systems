% APPROACH - FLIGHT CONDITION
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
CLu = 0;
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
SYSTEM = ss(A,B,C,D);                                                                     % State Space
G = tf(SYSTEM);                                                                           % Transfer Functions
Elev_Dyn = tf(10,[1 10]);                                                                 %Elevator Dynamics
AOA_TF = Elev_Dyn * (G(2,1)*(1/u0));                                                      % AOA Transfer Function with Elevator Dynamics
PitchRate_TF = Elev_Dyn * (G(3,1));                                                       % Pitch Rate Transfer Function with Elevator Dynamics
PitchAttitude_TF = Elev_Dyn * (G(4,1));                                                   % Pitch Attitude Transfer Function with Elevator Dynamics
Characteristic_Equation = poly(A)                                                         % Characteristic Equation Coefficients
damp(PitchAttitude_TF)                                                                    % Unaugmented Mode Characteristics

% PITCH RATE FEEDBACK -- 'q'
disp('-----------------------------PITCH RATE FEEDBACK------------------------------------')
% controlSystemDesigner('CSD_PITCHRATE.mat')                   % Root Locus to Find Approprite Gain
K = 1.898;                                                     % Gain Obtained
CL_PitchRate_TF = feedback(PitchRate_TF,K,+1);                 % Closed Loop Pitch Rate Transfer Function
damp(CL_PitchRate_TF)                                          % Mode Characteristics After Pitch Rate Feedback
[num0 den0] = tfdata(CL_PitchRate_TF);                         % Taking Numerator of Pitch Rate Transfer Function (Stays Same Even After Feedback)

% AOA FEEDBACK -- 'alpha'
disp('-----------------------------PITCH RATE & AOA FEEDBACK------------------------------------')
[num1 den1] = tfdata(AOA_TF);                                  % Taking Numerator of AOA Transfer Function (Stays Same Even After Feedback)
[num2 den2] = tfdata(CL_PitchRate_TF);                         % Taking Denominator of Closed Loop New System Caused by Pitch Rate Feedback
AOA_TF = (tf(num1,den2));                                      % AOA Transfer Function due to New System Dynamics Caused by Pitch Rate Feedback
% controlSystemDesigner('CSD_AOA.mat')                         % Root Locus to Find Approprite Gain
K = 2;                                                         % Gain Obtained
CL_AOA_TF = feedback(AOA_TF,K,+1);                             % Closed Loop AOA Transfer Function After Pitch Rate & AOA Feedback
damp(CL_AOA_TF)                                                % Mode Characteristics After Pitch Rate Feedback & AOA Feedback
[num3 den3] = tfdata(CL_AOA_TF);                               % Taking Denominator of Closed Loop New System Caused by Pitch Rate & AOA Feedback
CL_PITCHRATE_TF = tf(num0, den3);                              % Pitch Rate Transfer Function due to New System Dynamics caused by Pitch Rate & AOA Feedback

% PITCH ATTITUDE FEEDBACK -- 'theta'
[num4 den4] = tfdata(PitchAttitude_TF);                        % Taking Numerator of Pitch Attitude Transfer Function (Stays Same Even After Feedback)
PitchAttitude_TF = tf(num4,den3);                              % Pitch Attitude Transfer Function due to New System Dynamics caused by Pitch Rate & AOA Feedback
% controlSystemDesigner('CSD_PITCHATTITUDE.mat')               % Root Locus to Find Approprite Gain
K = -0.7;                                                      % Gain Obtained
CL_PitchAttitude_TF = feedback(K*PitchAttitude_TF,1)           % Closed Loop Pitch Attitude Transfer Function After All 3 Feedbacks

% The following shows the final closed loop step responses of Pitch Rate, AOA & Pitch Attitude along with the step response info and mode characteristics

% ALL 3 FEEDBACKS - FINAL STEP RESPONSES OF THE SYSTEM
disp('-----------------------------PITCH RATE, AOA & PITCH ATTITUDE FEEDBACK------------------------------------')
[num5 den6] = tfdata(CL_PitchAttitude_TF);                    % Taking denominator of Closed Loop New System Caused by All 3 Feedbacks
step(CL_PitchAttitude_TF)                                     % Closed Loop Step Response for Pitch Attitude Transfer Function with All 3 Feedbacks
CL_PitchAttitude_TF_StepInfo = stepinfo(CL_PitchAttitude_TF)  % Closed Loop Step Response Info for Pitch Attitude Transfer Function with All 3 Feedbacks
title 'Pitch Hold Autopilot with Pitch rate & AoA SAS'

% FINAL SYSTEM MODE CHARACTERISTICS
%{ 
As mode characteristics depend on the characteristic equation, we can take any one of 
the above closed loop transfer fucntion as they have the same characteristic equation
%}
Characteristic_Equation = den6;                               % Characteristic Equation Coefficients
damp(CL_PitchAttitude_TF)

%{ 
The final dynamics of the system has alot of steady state error and poor
transient characteristics. To have better response characteristics we apply
a PI Controller feedback to the final pitch transfer function obtained
%}
%PI CONTROLLER
Kp = 0.5;                                                     % Proportional Gain
Ki = 0.7;                                                     % Integral Gain
Kd = 0;                                                       % Derivative Gain (Zero because we are implementing a PI controller)
PI = pid(Kp,Ki,Kd)                                            % Transfer Function of PI Controller
Comp_Sys = PI * CL_PitchAttitude_TF;
CL_Sys = feedback(Comp_Sys,1)                                 % Applying PI controller feedback to the final pitch transfer function
figure
step(CL_Sys)                                                  % Closed Loop Step Response for Pitch Attitude Transfer Function with All the Feedbacks
title 'Pitch Hold Autopilot - Step Response'
figure
impulse(CL_Sys)                                               % Closed Loop Impulse Response for Pitch Attitude Transfer Function with All the Feedbacks
title 'Pitch Hold Autopilot - Impulse Response'
damp(CL_Sys)                                                  % Mode Characteristics After All 3 Feedbacks and a PI controller feedback to the final system
controlSystemDesigner('CSD_Approach_PitchHold.mat')