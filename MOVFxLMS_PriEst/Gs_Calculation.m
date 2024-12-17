% %% clean memory 
% close all ;
% clear     ;
% clc       ;
% %% Secondary path
% S   = [0.03,0.7];
function Gs = Gs_Calculation(Sec,Disturbance,len,muS)
S = Sec ;
%% System seting 
fs = 16000 ;
T  = 400   ; %10 200
t  = 0:1/fs:T;
N  = length(t);
%% Noise generation 
Re  = 1*randn(N,1);
Xin = filter(S,1,Re);
%% -----------------------------------------------------------------
%len  = 64 ;
Dely = zeros(len,1);
Dely(len/2) =  1   ;
Dir         = filter(Dely,1,Re);
%%
%muS = 0.1;
secondaryPathEstimator = dsp.LMSFilter('Method','Normalized LMS','StepSize', muS, ...
    'Length', len);
[yS,eS,SecondaryPathCoeffsEst] = secondaryPathEstimator(Xin,Dir);
% figure ;
% plot(eS);
% grid on ;
%-----------------------------------------------------
Xin_esti       = filter(SecondaryPathCoeffsEst,1,Disturbance) ; 
%-----------------------------------------------------
Gs = var(Disturbance)/var(Xin_esti);
end