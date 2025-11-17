% APPROACH - FLIGHT CONDITION

% FlIGHT CONDITIONS
M = 0.206;   % Approach mach number
u0 = 230;    % True airspeed
Q = 62.9;    % Dynamic pressure
g = 32.2;    % Gravitational acceleration
 
% AIRCRAFT PARAMETERS
S = 530;     % Wing surface area
cbar = 16;   % Mean aerodynamic chord
b = 38.7;    % Wing Span
m = 1031.89; % Mass of aircraft in slugs
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
