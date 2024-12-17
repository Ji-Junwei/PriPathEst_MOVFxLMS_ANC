clc;clear all; close all;
% compare with MOVFXLMS,MOVMFXLMS,FXLMS,ImprovedMOVFXLMS using real path
% and real noise (Aircraft)


%% System seting 
fs = 16000    ;
T  = 60        ;
t  = 0:1/fs:T ;
t1 = 0:1/fs:(2*T+1);
N  = length(t);
%% Primary Path and secondary path 
load("Path.mat");
Pri = P;
%% noise generation
data = load("Noise_data\Aircraft.mat");
data1 = load("Noise_data\Engine.mat");
data2 = load("Noise_data\Motor.mat");
noise = [5*(data1.Engine_noise(1:20*fs));2.5*(data.aircraft_noise);12*(data2.Motor_noise)];
Ref = noise;
Dis = awgn(filter(Pri,1,Ref),20);
Fx = filter(S,1,Ref);


%% parameter
wlen = 256;
muw = 0.0002;

rho = 0.8;                  % power constraint
%% FXLMS algorithm 
FXLMS = dsp.FilteredXLMSFilter(wlen,"SecondaryPathCoefficients",S,"SecondaryPathEstimate",S,"StepSize",muw);
[fxlms_y,fxlms_e] = FXLMS(Ref,Dis);

%% MOVFXLMS

% Optimal solution under the constrain 
Gs_movfxlms =  Gs_Calculation(S,Dis,800,0.01);
SimD_movfxlms = var(Dis);
eta_movfxlms  = SimD_movfxlms/(Gs_movfxlms*rho);
lambda_movfxlms = max(Gs_movfxlms*(sqrt(eta_movfxlms)-1),0);    % penalty factor
muw2_movfxlms = 1.000*lambda_movfxlms * muw;

MOVFXLMS = LeakPower_FxLM(muw,wlen,S,muw2_movfxlms);
[WT_MOVFXLMS,E_MOVFXLMS,Y_MOVFXLMS,W_n2_MOVFXLMS,MOVFXLMS]= MOVFXLMS.LeakPower_controller(Dis,Ref);

%% MOVMFXLMS

movmfxlms = MOVMFXLMS(muw,muw,S,wlen);
[WT_movmfxlms,E_movmfxlms,yc_movmfxlms,Gs_movmfxlms,lambda_movmfxlms,movmfxlms] = movmfxlms.MOVMFXLMS_controller_v1(Dis,Ref,rho);

%% improved MOVFXLMS

% Pre-train stage
muP = 0.1;
plen = 800;
EstPriPath = Primary_Path_Estimation(Pri,muP,plen); % obtain estimated primary path

Improved_MOVFXLMS = MOVFXLMS_PriPathEst(muw,muw,S,wlen,EstPriPath);
[WT_imovfxlms,E_imovfxlms,yc_imovfxlms,Gs_imovfxlms,lambda_imovfxlms,Improved_MOVFXLMS] = Improved_MOVFXLMS.MOVFXLMS_PriPathEst_controller(Dis,Ref,rho);

%% plot
set(groot,'defaultAxesTickLabelInterpreter','latex');

plot(Dis);hold on; 
plot(E_movmfxlms);hold on; 
plot(E_imovfxlms);hold on;
plot(E_MOVFXLMS);hold on;
plot(fxlms_e);
legend('Disturbance','MOVMFXLMS','Improved MOVFXLMS','MOVFXLMS','FXLMS','Interpreter','latex');
title('Error Signal','Interpreter','latex');
xlabel('Time (Sec)','Interpreter','latex');
grid on;

%save('Case2_data_inter.mat','Dis','Ref','fxlms_y','fxlms_e','E_MOVFXLMS','Y_MOVFXLMS','E_movmfxlms','yc_movmfxlms','Gs_movfxlms','Gs_movmfxlms','lambda_movfxlms','lambda_movmfxlms','E_imovfxlms','yc_imovfxlms','Gs_imovfxlms','lambda_imovfxlms');

