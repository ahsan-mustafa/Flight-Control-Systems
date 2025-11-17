clear 
clc
%Aircraft Data
M = 0.206; a_o = 1117; u_o = 230; rho = 0.002377; g = 32.2;
W = 33200; m = W/g; S = 530; b = 38.7;
Ix = 23700; Iz = 133700; Q = 0.5*rho*(u_o)^2;

%Non-Dimensional Stability Derivatives
Cy_b = -0.655; Cl_b = -0.156; Cn_b = 0.199;
Cl_p = -0.272; Cn_p = 0.013; Cy_p = 0;
Cl_r = 0.205; Cn_r = -0.320; Cy_r = 0;
Cl_del_a = 0.0570; Cn_del_a = 0.0041;
Cy_del_r = 0.124; Cl_del_r = 0.0009; Cn_del_r = -0.072;

%Dimensional Stability Derivatives
Y_v = ((Q*S)/(m*u_o)) * Cy_b;
L_v = ((Q*S*b)/(Ix*u_o)) * Cl_b;
N_v = ((Q*S*b)/(Iz*u_o)) * Cn_b;
Y_p = ((Q*S*b)/(2*m*u_o)) * Cy_p;
L_p = ((Q*S*(b)^2)/(2*Ix*u_o)) * Cl_p;
N_p = ((Q*S*(b)^2)/(2*Iz*u_o)) * Cn_p;
Y_r = ((Q*S*b)/(2*m*u_o)) * Cy_r;
L_r = ((Q*S*(b)^2)/(2*Ix*u_o)) * Cl_r;
N_r = ((Q*S*(b)^2)/(2*Iz*u_o)) * Cn_r;
L_del_a = ((Q*S*b)/(Ix)) * Cl_del_a;
N_del_a = ((Q*S*b)/(Iz)) * Cn_del_a;
Y_del_r = ((Q*S)/(m)) * Cy_del_r;
L_del_r = ((Q*S*b)/(Ix)) * Cl_del_r;
N_del_r = ((Q*S*b)/(Iz)) * Cn_del_r;
Y_b = Y_v*u_o;
L_b = L_v*u_o;
N_b = N_v*u_o;

%State Space and Characteristics of Lateral Motion (MIMO System)
A = [(Y_b/u_o) (Y_p/u_o) -(1 - (Y_r/u_o)) (g/u_o) ; L_b L_p L_r 0 ; N_b N_p N_r 0 ; 0 1 0 0];
B = [0 Y_del_r/u_o ; L_del_a L_del_r ; N_del_a N_del_r ; 0 0];
C = [1 0 0 0 ; 0 1 0 0 ; 0 0 1 0 ; 0 0 0 1];
D = [0 0 ; 0 0 ; 0 0 ; 0 0];
states = {'beta' 'roll rate' 'yaw rate' 'phi'};
inputs = {'aileron' 'rudder'};
outputs = {'beta' 'roll rate' 'yaw rate' 'phi'};
Sys = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs)
damp(Sys)
step(Sys)
figure();

%All the 8 systems extracted out of the MIMO System
Sys11 = Sys('beta','aileron')
Sys12 = Sys('roll rate','aileron');
Sys13 = Sys('yaw rate','aileron');
Sys14 = Sys('phi','aileron');

Sys21 = Sys('beta','rudder')
Sys22 = Sys('roll rate','rudder');
Sys23 = Sys('yaw rate','rudder');
Sys24 = Sys('phi','rudder');

%Designing a Yaw Damper i.e., Modifying Sys23
[A1,B1,C1,D1] = ssdata(Sys23);
[num,den] = ss2tf(A1,B1,C1,D1);
G_23 = tf(num,den)
Filter = tf([1 0.03],[1 0.7]);                  %Washout Filter for Spiral Mode
Sys_Filter = Sys23*Filter;
rlocus(-Sys_Filter)                             %Root Locus with Filter
figure();
Kr = -0.732;                                    %Selected Gain value
WOF = Filter*Kr;
CL23 = feedback(Sys23,WOF);                     %Closed Loop System for Yaw Damper
damp(CL23)
step(Sys23)
Open_Loop_Stepinfo = stepinfo(Sys23)
hold on
step(CL23)
Closed_Loop_Stepinfo = stepinfo(CL23)
figure();

impulse(Sys23)
hold on
impulse(CL23)
figure();

%Applying Yaw Damper on MIMO System
CL_Sys = feedback(Sys,WOF,2,3);
step(CL_Sys)