clc;clear all; close all;

%% System seting 
fs = 16000    ;
T  = 60        ;
t  = 0:1/fs:T ;
N  = length(t);
%% Primary Path and secondary path 
p_low  = 80  ;
p_high = 5000 ;
Pri = fir1(255,[2*p_low/fs 2*p_high/fs]);
s_low = 80 ;
s_high = 5000;
S = fir1(127,[2*s_low/fs 2*s_high/fs]);
%% noise generation
noise = 2*sin(2*pi*t*400);
low = 100;
high = 2000;
fil = fir1(64,[2*low/fs 2*high/fs]);
Ref = filter(fil,1,noise);
Dis = filter(Pri,1,Ref);
Fx = filter(S,1,Ref);

%% Pre-train stage
muP = 0.01;
plen = 256;
EstPriPath = Primary_Path_Estimation(Pri,muP,plen); % obtain estimated primary path

%% MOVFXLMS with Primary path estimation

muw = 0.000002;
len = 256;
rho = 1;
a = MOVFXLMS_PriPathEst(muw,muw,S',len,EstPriPath);
[WT,E,yc,yc1,Gs,lambda,a] = a.MOVFXLMS_PriPathEst_controller_v2(Dis,Ref,rho);

figure;
plot(Dis);hold on;plot(E);


%% FXLMS
[y_fxlms,Cy_fxlms,Cy1_fxlms,e_fxlms,~,W_fxlms]=FxLMS(Ref',muw,muw,128,256,Dis',S,'type','OutputSaturation');
figure;plot(Dis);hold on;plot(e_fxlms);


%save('case4_1_data',"WT","lambda","Gs","yc","yc1","E","Cy1_fxlms","Cy_fxlms","y_fxlms","W_fxlms","e_fxlms","Dis","Ref","Fx");