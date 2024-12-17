%% Estimated primary path obtained
function Pri_Est = Primary_Path_Estimation(Pri_path, muP,plen)

    
    %% System seting 
    fs = 16000 ;
    T  = 400   ; %10 200
    t  = 0:1/fs:T;
    N  = length(t);

    %% Noise generation 
    v  = randn(N,1);
    v_prime = filter(Pri_path,1,v);

    %%Primary path estimate
    PrimaryPathEstimator = dsp.LMSFilter('Method','Normalized LMS','StepSize', muP, ...
    'Length', plen);
    [yP,eP,PrimaryPathCoeffsEst] = PrimaryPathEstimator(v,v_prime);
    Pri_Est = PrimaryPathCoeffsEst;

end