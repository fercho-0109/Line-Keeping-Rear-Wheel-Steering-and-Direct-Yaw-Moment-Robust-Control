function kfb = calc_fb(v)
load('computo_controller.mat')
%load('gains_L1.mat')
%load('gains_hinf.mat')
if v>50 && v<=60
    rho=(v-50)/(60-50);
    kfb=(1-rho)*kfb_50+rho*kfb_60
end
 if v>60 && v<=70
    rho=(v-60)/(70-60);
    kfb=(1-rho)*kfb_60+rho*kfb_70
 end
 if v>70 && v<=80
    rho=(v-70)/(80-70);
    kfb=(1-rho)*kfb_70+rho*kfb_80
 end
 if v>80 && v<=90
    rho=(v-80)/(90-80);
    kfb=(1-rho)*kfb_80+rho*kfb_90
 end
 if v>90 && v<=100
    rho=(v-90)/(100-90);
    kfb=(1-rho)*kfb_90+rho*kfb_100
end
end
