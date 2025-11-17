% SUPERSONIC - FLIGHT CONDITION

% FlIGHT CONDITIONS
M = 1.8;     % Approach mach number
u0 = 1742;   % True airspeed
Q = 434.5;   % Dynamic pressure
g = 32.2;    % Gravitational acceleration
 
% AIRCRAFT PARAMETERS
S = 530;     % Wing surface area
cbar = 16;   % Mean aerodynamic chord
b = 38.7;    % Wing Span
W = 39000;   % Mass of aircraft in pounds
m = W/g;     % Mass of aircraft in slugs
Ix = 25000;  % Moment of inertia X-Axis
Iy = 122200; % Moment of inertia Y-Axis
Iz = 139800; % Moment of inertia Z-Axis

% Non-Dimensional Stability Derivatives
Cl_B = -0.025; Cl_p = -0.2; Cl_r = 0.0400;
Cy_B = -0.700; Cy_p = 0.00; Cy_r = 0.0000;
Cn_B = 0.0900; Cn_p = 0.00; Cn_r = -0.260; Cn_TB = 0;

Cl_delta_a = 0.01500; Cl_delta_r = 0.0030;
Cy_delta_a = -0.0100; Cy_delta_r = 0.0500;
Cn_delta_a = -0.0009; Cn_delta_r = -0.025;

% Dimensional Stability Derivatives
Y_v = (Cy_B*Q*S) / (m*u0);             % Yv
L_v = (Cl_B*Q*S*b) / (Ix*u0);          % Lv
N_v = (Cn_B*Q*S*b) / (Iz*u0);          % Nv

Y_B = Y_v * u0;                        % Y Beta
L_B = L_v * u0;                        % L Beta
N_B = N_v * u0;                        % N Beta

Y_p = (Q*S*b*Cy_p) / (2*m*u0);         % Yp
L_p = (Cl_p*Q*S*(b^2)) / (2*Ix*u0);    % Lp
N_p = (Cn_p*Q*S*(b^2)) / (2*Iz*u0);    % Np

Y_r = (Q*S*b*Cy_r) / (2*m*u0);         % Yr
L_r = (Cl_r*Q*S*(b^2)) / (2*Ix*u0);    % Lr
N_r = (Cn_r*Q*S*(b^2)) / (2*Iz*u0);    % Nr

Y_delta_a = (Q*S*Cy_delta_a) / m;      % Y delta a
L_delta_a = (Q*S*b*Cl_delta_a) / Ix;   % L delta a
N_delta_a = (Q*S*b*Cn_delta_a) / Iz;   % N delta a

Y_delta_r = (Q*S*Cy_delta_r) / m;      % Y delta r
L_delta_r = (Q*S*b*Cl_delta_r) / Ix;   % L delta r
N_delta_r = (Q*S*b*Cn_delta_r) / Iz;   % N delta r

% State Space
A = [Y_B/u0 Y_p/u0 -(1-(Y_r/u0)) g/u0; L_B L_p L_r 0; N_B N_p N_r 0; 0 1 0 0];
B = [0 Y_delta_r/u0; L_delta_a L_delta_r; N_delta_a N_delta_r; 0 0];
C = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
D = [0 0; 0 0; 0 0; 0 0];

states = {'Beta' 'p' 'r' 'Phi'};
inputs = {'Aileron' 'Rudder'};
outputs = {'Sideslip' 'Roll Rate' 'Yaw Rate' 'Bank Angle'};
sys = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs)

% MIMO 
sys11 = sys('Sideslip','Aileron');
sys12 = sys('Roll Rate','Aileron');
sys13 = sys('Yaw Rate','Aileron');
sys14 = sys('Bank Angle','Aileron');

sys21 = sys('Sideslip','Rudder');
sys22 = sys('Roll Rate','Rudder');
sys23 = sys('Yaw Rate','Rudder');
sys24 = sys('Bank Angle','Rudder');

% Open Loop Analysis
disp('-----------OPEN LOOP ANALYSIS--------------')
Characteristic_Equation = poly(A)
damp(sys)
figure
step(sys)
openloop_step = stepinfo(sys23)

% Closed Loop SAS Analysis
disp('-----------CLOSED LOOP SAS ANALYSIS--------------')
damp(clsys)
figure
step(sys)
hold on
step(clsys)
figure
impulse(sys)
hold on
impulse(clsys)
clsys23 = clsys('Yaw Rate - r', 'Rudder');
closeloop_step = stepinfo(clsys23)

% Coordinated Turn Hold Autopilot
disp('-----------Coordinated Turn Hold Autopilot--------------')
figure
step(BetaToAileron1)
hold on
step(PhiToAileron1)
legend('beta','phi')
CoTurnHold = stepinfo(PhiToAileron1)
figure
impulse(BetaToAileron1)
hold on
impulse(PhiToAileron2)
legend('beta','phi')

% Heading Autopilot
disp('-----------Heading Autopilot--------------')
figure
step(HeadingAngle)
HeadingHold = stepinfo(HeadingAngle)
figure
step(BetaToAileron3)
hold on
step(PhiToAileron3)
legend('beta','phi')
figure
step(RollRate1)
hold on
step(YawRate1)
legend('Roll Rate (p-deg/s)', 'Yaw Rate (r-deg/s)')