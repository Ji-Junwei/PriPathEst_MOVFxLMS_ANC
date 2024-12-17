clc;clear all; close all;
% compare with MOVFXLMS,MOVMFXLMS,ImprovedMOVFXLMS

%% System seting 
fs = 16000    ;
T  = 240        ;
t  = 0:1/fs:T ;
N  = length(t);

%% Primary Path and secondary path 
P   = [1.82 1.11] ;
S   = [0.03,0.87] ;
Pri = conv(P,S)   ;

%% noise generation
noise = 1*randn(N,1);
low = 400;
high = 2000;
fil = fir1(64,[2*low/fs 2*high/fs]);
Ref = filter(fil,1,noise);
Dis = filter(Pri,1,Ref);

low1 = 200;
high1 = 300;
fil1 = fir1(64,[2*low1/fs 2*high1/fs]);
Inter = filter(fil1,1,noise);

% Dis1 = Dis + 1*sin(2*pi*t*300)';
Dis1 = awgn(Dis,10);
Fx = filter(S,1,Ref);

%% parameter
wlen = 2;
muw = 0.00006;

rho = 1;

%% FXLMS algorithm 
FXLMS = dsp.FilteredXLMSFilter(wlen,"SecondaryPathCoefficients",S,"SecondaryPathEstimate",S,"StepSize",muw);
[fxlms_y,fxlms_e] = FXLMS(Ref,Dis1);

%% MOVFXLMS

% Optimal solution under the constrain 
% no interference at error
Gs_movfxlms =  Gs_Calculation(S,Dis,64,0.1);
SimD_movfxlms = var(Dis);
eta_movfxlms  = SimD_movfxlms/(Gs_movfxlms*rho);
lambda_movfxlms = max(Gs_movfxlms*(sqrt(eta_movfxlms)-1),0);
muw2_movfxlms = 1.000*lambda_movfxlms * muw;

MOVFXLMS = LeakPower_FxLM(muw,wlen,S',muw2_movfxlms);
[WT_MOVFXLMS,E_MOVFXLMS,Y_MOVFXLMS,W_n2_MOVFXLMS,MOVFXLMS]= MOVFXLMS.LeakPower_controller(Dis,Ref);

% interference at error
Gs_movfxlms1 =  Gs_Calculation(S,Dis1,64,0.01);
SimD_movfxlms1 = var(Dis1);
eta_movfxlms1  = SimD_movfxlms1/(Gs_movfxlms1*rho);
lambda_movfxlms1 = max(Gs_movfxlms1*(sqrt(eta_movfxlms1)-1),0);
muw2_movfxlms1 = 1.000*lambda_movfxlms1 * muw;

MOVFXLMS1 = LeakPower_FxLM(muw,wlen,S',muw2_movfxlms1);
[WT_MOVFXLMS1,E_MOVFXLMS1,Y_MOVFXLMS1,W_n2_MOVFXLMS1,MOVFXLMS1]= MOVFXLMS1.LeakPower_controller(Dis1,Ref);

%% MOVMFXLMS

% no interference at error
movmfxlms = MOVMFXLMS(muw,muw,S',wlen);
[WT_movmfxlms,E_movmfxlms,yc_movmfxlms,Gs_movmfxlms,lambda_movmfxlms,movmfxlms] = movmfxlms.MOVMFXLMS_controller_v1(Dis,Ref,rho);

% interference at error
movmfxlms1 = MOVMFXLMS(muw,muw,S',wlen);
[WT_movmfxlms1,E_movmfxlms1,yc_movmfxlms1,Gs_movmfxlms1,lambda_movmfxlms1,movmfxlms1] = movmfxlms1.MOVMFXLMS_controller_v1(Dis1,Ref,rho);
%% improved MOVFXLMS

% Pre-train stage
muP = 0.1;
plen = 3;
EstPriPath = Primary_Path_Estimation(Pri,muP,plen); % obtain estimated primary path

% no interference at error
Improved_MOVFXLMS = MOVFXLMS_PriPathEst(muw,muw,S',wlen,EstPriPath);
[WT_imovfxlms,E_imovfxlms,yc_imovfxlms,Gs_imovfxlms,lambda_imovfxlms,Improved_MOVFXLMS] = Improved_MOVFXLMS.MOVFXLMS_PriPathEst_controller(Dis,Ref,rho);

% interference at error
Improved_MOVFXLMS1 = MOVFXLMS_PriPathEst(muw,muw,S',wlen,EstPriPath);
[WT_imovfxlms1,E_imovfxlms1,yc_imovfxlms1,Gs_imovfxlms1,lambda_imovfxlms1,Improved_MOVFXLMS1] = Improved_MOVFXLMS1.MOVFXLMS_PriPathEst_controller(Dis1,Ref,rho);

%% plot
figure;
plot(smooth(WT_MOVFXLMS(1,:),4096));hold on;
plot(smooth(WT_MOVFXLMS1(1,:),4096));hold on;
plot(smooth(WT_movmfxlms(1,:),4096));hold on;
plot(smooth(WT_movmfxlms1(1,:),4096));hold on;
plot(smooth(WT_imovfxlms(1,:),4096));hold on;
plot(smooth(WT_imovfxlms1(1,:),4096));
legend('MOVFXLMS no interference','MOVFXLMS has interference','MOVMFXLMS no interference','MOVMFXLMS has interference','Im_MOVFXLMS no interference','Im_MOVFXLMS has interference');

figure;
plot(Dis);hold on;plot(E_imovfxlms1);hold on;plot(fxlms_e);

% save('Case3_data_nointerference.mat','Dis','Ref','E_MOVFXLMS','Y_MOVFXLMS','E_movmfxlms','yc_movmfxlms','Gs_movfxlms','Gs_movmfxlms','lambda_movfxlms','lambda_movmfxlms','E_imovfxlms','yc_imovfxlms','Gs_imovfxlms','lambda_imovfxlms','WT_MOVFXLMS','WT_movmfxlms','WT_imovfxlms');
% save('Case3_data_hasinterference.mat','Dis1','Fx','E_MOVFXLMS1','Y_MOVFXLMS1','E_movmfxlms1','yc_movmfxlms1','Gs_movfxlms1','Gs_movmfxlms1','lambda_movfxlms1','lambda_movmfxlms1','E_imovfxlms1','yc_imovfxlms1','Gs_imovfxlms1','lambda_imovfxlms1','WT_MOVFXLMS1','WT_movmfxlms1','WT_imovfxlms1');
% 
% 
% 



