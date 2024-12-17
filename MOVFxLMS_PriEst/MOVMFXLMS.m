classdef MOVMFXLMS
    properties
        muw1           % step size
        muw2           % step size * penalty factor
        len            % control filter length
        SecPath        % secondary path
        slen           % secondary path length

    end
    methods
        function obj = MOVMFXLMS(muW1,muW2,Sec,len)
            obj.muw1 = muW1;
            obj.muw2 = muW2;
            obj.SecPath = Sec;
            obj.len = len;
            obj.slen = length(Sec);

        end

        function [W,E,yc,Gs,lambda,obj] = MOVMFXLMS_controller_v1(obj,Dis,X,rho)
            N = length(X);
            xc = zeros(obj.len,1);
            W = zeros(obj.len,N);
            ys = zeros(obj.slen,1);
            Y = zeros(N,1);
            yc = zeros(N,1);
            E = zeros(N,1);
            yf = zeros(obj.slen,1);
            fx = filter(obj.SecPath,1,X);  % filtered reference by secondary path
            xf = zeros(obj.len,1);
            e = zeros(N,1);
            EstDis = zeros(N,1);

            K = 1024;
            Pxf = zeros(K,1);        % Power of filtered reference
            Px = zeros(K,1);         % Power of reference
            Pdis = zeros(K,1);       % Power of disturbance
            Gs = zeros(N,1);
            lambda = zeros(N,1);     % penalty factor

            for ii = 1:N
                xc = [X(ii);xc(1:end-1)];
                yc(ii) = W(:,ii)'*xc;          % control signal
                ys = [yc(ii);ys(1:end-1)];
                Y(ii) = obj.SecPath'*ys;   % anti-noise
                E(ii) = Dis(ii) - Y(ii);   % residual error
%                 E(ii) = awgn(E(ii),10);

                yf = [yc(ii);yf(1:end-1)];
                Yf = obj.SecPath'*ys;      % estimated anti-noise
                EstDis(ii) = E(ii) + Yf;   % estimated disturbance

                xf = [fx(ii);xf(1:end-1)];  % filtered reference vector
                yc1 = W(:,ii)'*xf;          
                e(ii) = EstDis(ii) - yc1;   % estimated error

                Pxf = [fx(ii)^2;Pxf(1:end-1)];
                Px = [X(ii)^2;Px(1:end-1)];
                Gs(ii) = max(sum(Pxf),0.3)/max(sum(Px),0.3);      % Gs calculation
                Pdis = [EstDis(ii)^2;Pdis(1:end-1)];
                eta = sum(Pdis)/(K*Gs(ii)*rho);
                lambda(ii) = max(Gs(ii)*(sqrt(eta)-1),0);

                obj.muw2 = obj.muw1*lambda(ii);
                W(:,ii+1) = W(:,ii) + obj.muw1*e(ii)*xf - obj.muw2*yc(ii)*xc;  % control filter updating


            end

        end


    end



end