function kff = calc_ff(v)
load('computo_controller.mat')
%load('gains_L1.mat')
%load('gains_hinf.mat')
if v>50 && v<=60
    rho=(v-50)/(60-50);
    kff=(1-rho)*kff_50+rho*kff_60;
end
 if v>60 && v<=70
    rho=(v-60)/(70-60);
    kff=(1-rho)*kff_60+rho*kff_70;
 end
 if v>70 && v<=80
    rho=(v-70)/(80-70);
    kff=(1-rho)*kff_70+rho*kff_80
 end
 if v>80 && v<=90
    rho=(v-80)/(90-80);
    kff=(1-rho)*kff_80+rho*kff_90
 end
 if v>90 && v<=100
    rho=(v-90)/(100-90);
    kff=(1-rho)*kff_90+rho*kff_100
end
end
