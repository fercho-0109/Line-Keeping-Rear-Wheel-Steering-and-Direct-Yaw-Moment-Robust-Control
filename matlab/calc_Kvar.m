function [kff, kfb] = calc_Kvar(vM)
%%
vM=vM/3.6;          % [v] longitudinal velocity

m=1562;             % [kg] mass
Iz=2630;            % [kg*m^2] Inertia 
af=1.104;           % [m] Distance from the center of gravity to front axle
ar=1.421;           % [m] Distance from the center of gravity to rear axle
Cf=42000;           % [N/rad] Front cornering stiffness 
Cr=64000;           % [N/rad] Rear cornering stiffness ;               %

% % desired model xdp= Ad*xd + Ed*deltaf
% t_beta=0.05;        % [s]
% k_beta=0;
% t_gamma=0.0375;     % [s]
% k_gamma=3.03; 
% 
% Ad=[-1/t_beta 0;
%     0       -1/t_gamma];
% Ed=[k_beta/t_beta;
%     k_gamma/t_gamma];

% systen matrices at max velocity

% A

a11=-(Cf+Cr)/(m*vM);
a12=-1-(af*Cf-ar*Cr)/(m*vM^2);
a21=-(af*Cf-ar*Cr)/(Iz);
a22=-(af^2*Cf+ar^2*Cr)/(Iz*vM);
% B
b11=Cr/(m*vM);
b12=0;
b21=-ar*Cr/Iz;
b22=1/Iz;
% E
e1=Cf/(m*vM);
e2=af*Cf/Iz;

AM=[a11 a12;
   a21 a22];

BM=[b11 b12;
   b21 b22];
C=[1 0;
   0 1];
D=[0 0;0 0];
EM=[e1;
   e2];

%B2M=[(EM-Ed) (AM-Ad)];
B2M=EM;
%%
     Fw=zeros(2,1);
     Aex=[zeros(2) -C;zeros(2) AM];
     B1=BM;
     Bex=[-D;B1];
     B2=EM;
     Eex=[(-Fw) eye(2);B2 zeros(2)];
     C1=[1 0;0 1];
     rho=5e-15;
     D12=sqrt(rho)*[zeros(4,2);1 0;0 1];
     Cz=[eye(2) zeros(2);zeros(4,4)];
     Dzu=D12;
     Dzw=zeros(6,3);
     lambda=2080.5101053;%6.3101053 8.5 980.5101053
    % uscita di prestazioni z=[0 Cz;1 0][xe x]' + Dzu*u + [Fz 0][wp xd]'
    % put in zero the errors

    %% DESCOMENT TO CHOOSE THE CONTROL
     %  Hinf
     K_Hinf = H_inf(Aex,Bex,Eex,Cz,Dzu,Dzw);  
     autH8in=eig(Aex+Bex*K_Hinf);
     kff_hinf=[K_Hinf(1,1),K_Hinf(1,2);K_Hinf(2,1),K_Hinf(2,2)];
     kfb_hinf=[K_Hinf(1,3),K_Hinf(1,4);K_Hinf(2,3),K_Hinf(2,4)];

     %para el reshape
     kff=[K_Hinf(1,1),K_Hinf(2,1),K_Hinf(1,2),K_Hinf(2,2)];
     kfb=[K_Hinf(1,3),K_Hinf(2,3),K_Hinf(1,4),K_Hinf(2,4)];

     % %  H2
     % K_H2=H_2_gain(Aex,Bex,Eex,Cz,Dzu)  
     % autH2in=eig(Aex+Bex*K_H2);
     % kff_h2=[K_H2(1,1),K_H2(1,2);K_H2(2,1),K_H2(2,2)];
     % kfb_h2=[K_H2(1,3),K_H2(1,4);K_H2(2,3),K_H2(2,4)];
     % 
     %  L1
     % K_L1=L_1(Aex,Bex,Eex,Cz,Dzu,Dzw,lambda);  
     % autL1in=eig(Aex+Bex*K_L1);
     % kff_l1=[K_L1(1,1),K_L1(1,2);K_L1(2,1),K_L1(2,2)];
     % kfb_l1=[K_L1(1,3),K_L1(1,4);K_L1(2,3),K_L1(2,4)];
     % 
     % % para el reshape
     % 
     % kff=[K_L1(1,1),K_L1(2,1),K_L1(1,2),K_L1(2,2)];
     % kfb=[K_L1(1,3),K_L1(2,3),K_L1(1,4),K_L1(2,4)];

end
