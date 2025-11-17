clear 
clc

%Aircraft Data
M = 0.206; a_o = 1117; u_o = 230; rho = 0.002377; g = 32.2;
W = 33200; m = W/g; S = 530; b = 38.7;
Ix = 23700; Iz = 133700; Q = 0.5*rho*(u_o)^2;

%Non-Dimensional Stability Derivatives
Cy_b = -0.655;
Cl_b = -0.156;
Cn_b = 0.199;
Cl_p = -0.272;
Cn_p = 0.013;
Cy_p = 0;
Cl_r = 0.205;
Cn_r = -0.320;
Cy_r = 0;
Cl_del_a = 0.0570;
Cn_del_a = 0.0041;
Cy_del_r = 0.124;
Cl_del_r = 0.0009;
Cn_del_r = -0.072;

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

%State Space and Characteristics of Lateral Motion
disp('-----------------------DYNAMIC MATRIX A-----------------------')
A = [Y_v Y_p -(u_o - Y_r) g ; L_v L_p L_r 0 ; N_v N_p N_r 0 ; 0 1 0 0]
disp('--------------------CONTROL INPUT MATRIX B--------------------')
B = [0 Y_del_r ; L_del_a L_del_r ; N_del_a N_del_r ; 0 0]

Characteristic_Equation = poly(A)
damp(A)