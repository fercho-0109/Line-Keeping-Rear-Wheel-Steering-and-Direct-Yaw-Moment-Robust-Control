clear;
clc;
close all;
format short;

m=1562;             % [kg] mass
Iz=2630;            % [kg*m^2] Inertia 
af=1.104;           % [m] Distance from the center of gravity to front axle
ar=1.421;           % [m] Distance from the center of gravity to rear axle
Cf=42000;           % [N/rad] Front cornering stiffness 
Cr=64000;           % [N/rad] Rear cornering stiffness 
v=80/3.6;          % [m/s] Longitudinal velocity
Fz=3830;

parameters=[m,Iz,af,ar,Cf,Cr,v,Fz];

x0=[0; 0];      % Initial conditions beta/gamma

% parameters of the desire model
t_beta=0.05;        % [s]
k_beta=0;
t_gamma=0.0375;     % [s]
k_gamma=3.03; 

% State Space model
a11=-(Cf+Cr)/(m*v);
a12=-((af*Cf-ar*Cr)/(m*v^2))-1;
a21=-(af*Cf-ar*Cr)/(Iz);
a22=-(af^2*Cf+ar^2*Cr)/(Iz*v);

b11=Cr/(m*v);
b12=0;
b21=-ar*Cr/Iz;
b22=1/Iz;

e1=Cf/(m*v);
e2=af*Cf/Iz;

A=[a11 a12;
   a21 a22];
B=[b11 b12;
   b21 b22];
C=[1 0;
   0 1];
E=[e1;
   e2];
D=[0 0;0 0];
model_M=[A,B,C,D,E];
% desired model xdp= Ad*xd + Ed*deltaf
Ad=[-1/t_beta 0;
    0       -1/t_gamma];
Ed=[k_beta/t_beta;
    k_gamma/t_gamma];
 %% controller gains in diferents velocities 
 [kff_50, kfb_50] = calc_Kvar(50);
 [kff_60, kfb_60] = calc_Kvar(60);
 [kff_70, kfb_70] = calc_Kvar(70);
 [kff_80, kfb_80] = calc_Kvar(80);
 [kff_90, kfb_90] = calc_Kvar(90);
 [kff_100, kfb_100] = calc_Kvar(100);

 load('BusSignals2.mat')