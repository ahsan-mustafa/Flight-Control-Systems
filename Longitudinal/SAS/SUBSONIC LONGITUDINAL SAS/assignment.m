%Study of Longitudianl Motion
m=39000*0.0311; %mass
g=32.2; % acceleration due to gravity
W=m*g; % Weight
Ix=25000; % Moment of Inertia x-axis 
Iy=122200; %Moment of Inertia y-axis 
Iz=139800;  %Moment of Inertia z·axis
S=530; %Wing Swface
b=38.7; % Wing Span  
AR=(b^2)/S;  % Aspect Ratio
c=16;  %MAC
V=876; % True airspeed
Q=283.2;   QS=Q*S; 
M=0.9;
a=QS/(m*V); % dummy variable
% Steady state
CL=0.26;
CD=0.030;
Cm=0;
CTx=0.030;
CmT=0;
% Stability derivatives
CTu=0; % because this is a jet aircraft
CD_0=0.0205;
CDu=0.027;
CDalpha=0.30;
CL_0=0.10;
CLu=0.270;
CLalpha=3.75;
CLalphadot=0.86;
Cmu=-0.117;
Cma=-0.40; 
Cmadot=-1.30;
Cmq=-2.70;
% Control derivatives % just an assumption: ignore throttle settings
CXdelta_e=-0.10;
CZdelta_e=0.40;
Cmdelta_e=-0.580;


%The u derivatives
%Xu
Xu=(a)*(-(CDu+(2*CD)))+CTu;
%Zu
Zu=-(a)*(2*CL+CLu);
%Mu
Mu=((QS*c)/(Iy*V))*Cmu;

%The w derivatives
%Xw
s=(2*CL)/(pi*e*AR); % dummy variable
Xw=a*(CL-(s*CLalpha));
%Zw
Zw=(a)*(-CLalpha-CD);
Za=V*Zw;
%Mw
Mw=((QS*c)/(Iy*V))*Cma;
Ma=V*Mw;
%Mwdot
Mwdot=((QS*c^2)/(2*(Iy)*(V^2)))*Cmadot;
Madot=V*Mwdot;

%The q derivatives
%Mq
 Mq=((QS*c^2)/(2*Iy*V))*Cmq;

%The delta e derivatives
%Xdelta_e
Xdelta_e=(QS/m)*CXdelta_e;

%Zdelta_e
Zdelta_e=(QS/m)*CZdelta_e;

%Mdelta_e
Mdelta_e=((QS*c^2)/Iy)*Cmdelta_e;

A=[Xu Xw 0 -g; Zu Zw V 0; Mu+(Mwdot*Zu) Mw+(Mwdot*Zw) Mq+(Mwdot*V) 0; 0 0 1 0];
B=[Xdelta_e Zdelta_e Mdelta_e+(Mwdot*Zdelta_e) 0]';
C=[0 0 0 1];
D=[0]';
System=ss(A,B,C,D);
damp(A);

% SAS DESIGN
% First we will use pitch attitude feedback for phugoid mode
%figure()
step(System)
s1=stepinfo(System)
%figure()
%rlocus(-System)
%pause
%[K_theta]=rlocfind(System)% This command is used for  
%gain selection from the root locus plot 

K_theta=0.044;
sys1=feedback(System,K_theta,+1);
[A2,B2,C2,D2]=ssdata(sys1);
C3=[0 0 1 0];
sys2=ss(A2,B2,C3,D2);

controlSystemDesigner('CSD_SUBSONIC.mat') % It has been used to 
%draw handling quality graph

%figure()
%rlocus(-sys2)
%pause
%[K_q]=rlocfind(sys2) % This is for gain selection and analysis

K_q=0.0155;
sys3=feedback(sys2,K_q,+1);
damp(sys3)
figure()
step(sys3)
s2=stepinfo(sys3)




