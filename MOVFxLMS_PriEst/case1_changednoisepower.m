clc;clear all; close all;
% compare with MOVFXLMS,MOVMFXLMS,FXLMS,ImprovedMOVFXLMS


%% System seting 
fs = 16000    ;
T  = 60        ;
t  = 0:1/fs:T ;
t1 = 0:1/fs:(3*T+1);
N  = length(t);
%% Primary Path and secondary path 
p_low  = 80  ;
p_high = 5000 ;
Pri = fir1(255,[2*p_low/fs 2*p_high/fs]);        % primary path
s_low = 80 ;
s_high = 5000;
S = fir1(127,[2*s_low/fs 2*s_high/fs]);          % secondary path
%% noise generation
noise3 = 1.5*randn(N,1);
noise1 = 2.5*randn(N,1);
noise2 = 3.5*randn(N,1);
noise = [noise3;noise1;noise2];
low = 100;
high = 2000;
fil = fir1(64,[2*low/fs 2*high/fs]);
Ref = filter(fil,1,noise);                    % reference
Dis = filter(Pri,1,Ref);                      % disturbance
Fx = filter(S,1,Ref);                         % filtered reference


%% parameter
wlen = 256;                         % control filter length
muw = 0.0002;                       % step size

rho = 1;                            % power constraint
%% FXLMS algorithm 
FXLMS = dsp.FilteredXLMSFilter(wlen,"SecondaryPathCoefficients",S,"SecondaryPathEstimate",S,"StepSize",muw);
[fxlms_y,fxlms_e] = FXLMS(Ref,Dis);

%% MOVFXLMS

% Optimal solution under the constrain 
Gs_movfxlms =  Gs_Calculation(S,Dis,128,0.01);
SimD_movfxlms = var(Dis);
eta_movfxlms  = SimD_movfxlms/(Gs_movfxlms*rho);
lambda_movfxlms = max(Gs_movfxlms*(sqrt(eta_movfxlms)-1),0);   % penalty factor
muw2_movfxlms = 1.000*lambda_movfxlms * muw;

MOVFXLMS = LeakPower_FxLM(muw,wlen,S',muw2_movfxlms);
[WT_MOVFXLMS,E_MOVFXLMS,Y_MOVFXLMS,W_n2_MOVFXLMS,MOVFXLMS]= MOVFXLMS.LeakPower_controller(Dis,Ref);

%% MOVMFXLMS

movmfxlms = MOVMFXLMS(muw,muw,S',wlen);
[WT_movmfxlms,E_movmfxlms,yc_movmfxlms,Gs_movmfxlms,lambda_movmfxlms,movmfxlms] = movmfxlms.MOVMFXLMS_controller_v1(Dis,Ref,rho);

%% improved MOVFXLMS

% Pre-train stage
muP = 0.1;
plen = 256;
EstPriPath = Primary_Path_Estimation(Pri,muP,plen); % obtain estimated primary path

Improved_MOVFXLMS = MOVFXLMS_PriPathEst(muw,muw,S',wlen,EstPriPath);
[WT_imovfxlms,E_imovfxlms,yc_imovfxlms,Gs_imovfxlms,lambda_imovfxlms,Improved_MOVFXLMS] = Improved_MOVFXLMS.MOVFXLMS_PriPathEst_controller(Dis,Ref,rho);

%% plot
set(groot,'defaultAxesTickLabelInterpreter','latex');

plot(t1(1:(3*N-3)),Dis(1:(3*N-3)));hold on; 
plot(t1(1:(3*N-3)),E_movmfxlms(1:(3*N-3)));hold on; 
plot(t1(1:(3*N-3)),E_imovfxlms(1:(3*N-3)));hold on;
plot(t1(1:(3*N-3)),E_MOVFXLMS(1:(3*N-3)));hold on;
plot(t1(1:(3*N-3)),fxlms_e(1:(3*N-3)));
legend('Disturbance','MOVMFXLMS','Improved MOVFXLMS','MOVFXLMS','FXLMS','Interpreter','latex');
title('Error Signal','Interpreter','latex');
xlabel('Time (Sec)','Interpreter','latex');
grid on;


%save('Case1_Data.mat','Dis','Ref','fxlms_y','fxlms_e','E_MOVFXLMS','Y_MOVFXLMS','E_movmfxlms','yc_movmfxlms','Gs_movfxlms','Gs_movmfxlms','lambda_movfxlms','lambda_movmfxlms','E_imovfxlms','yc_imovfxlms','Gs_imovfxlms','lambda_imovfxlms');

