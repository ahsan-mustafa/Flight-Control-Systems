% SUBSONIC - FLIGHT CONDITION

% FlIGHT CONDITIONS
M = 0.90;    % Approach mach number
u0 = 876;    % True airspeed
Q = 283.2;   % Dynamic pressure
g = 32.2;    % Gravitational acceleration
 
% AIRCRAFT PARAMETERS
S = 530;     % Wing surface area
cbar = 16;   % Mean aerodynamic chord
b = 38.7;    % Wing Span
m = 39000/g; % Mass of aircraft
Iy = 122200; % Moment of inertia Y-Axis

% NON-DIMENSIONAL DERIVATIVES
CD0 = 0.0300; CDalpha = 0.30; 
CL0 = 0.2600; CLalpha = 3.75; CLalphaDot = 0.86; CLq = 1.80; CLdelta_e = 0.4000;
CM0 = 0.0250; CMalpha = -0.4; CMalphaDot = -1.3; CMq = -2.7; CMdelta_e = -0.580; 

CDu = 0.0270;
CMu = -0.117;
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
  
% DYNAMIC MATRIX A
disp('-----------------------DYNAMIC MATRIX A-----------------------')
A = [Xu Xw 0 -g; Zu Zw u0 0; (Mu+(MwDot*Zu)) (Mw+(MwDot*Zw)) (Mq+(MwDot*u0)) 0; 0 0 1 0]

% CONTROL INPUT MATRIX B
disp('--------------------CONTROL INPUT MATRIX B--------------------')
B = [Xdelta_e 0; Zdelta_e 0; (Mdelta_e+(MwDot*Zdelta_e)) 0; 0 0]

% LONGITUDINAL CHARACTERISTICS
disp('--------------------LONGITUDINAL CHARACTERISTICS--------------------')
Characteristic_Equation = poly(A)   % Characteristic equation coefficients
damp(A)                             % Mode characteristics