classdef MOVFXLMS_PriPathEst
    properties
        muw1        % step size
        muw2        % step size * penalty factor
        len         % control filter length
        SecPath     % secondary path
        slen        % secondary path length
        EstPriPath  % primary path

    end

    methods
        function obj = MOVFXLMS_PriPathEst(muW1,muW2,Sec,len,EstPri)
            obj.muw1 = muW1;
            obj.muw2 = muW2;
            obj.SecPath = Sec;
            obj.len = len;
            obj.slen = length(Sec);
            obj.EstPriPath = EstPri;

        end

        function [W,E,yc,Gs,lambda,obj] = MOVFXLMS_PriPathEst_controller(obj,Dis,X,rho)
            N = length(X);
            xc = zeros(obj.len,1);
            W = zeros(obj.len,N);
            ys = zeros(obj.slen,1);
            Y = zeros(N,1);
            yc = zeros(N,1);
            E = zeros(N,1);
            fx = filter(obj.SecPath,1,X);          % filtered reference by secondary path
            xf = zeros(obj.len,1);
            EstDis = filter(obj.EstPriPath,1,X);   % estimated disturbance

            K = 1024;
            Pxf = zeros(K,1);               % Power of filtered reference
            Px = zeros(K,1);                % Power of reference
            Pdis = zeros(K,1);              % Power of disturbance
            Gs = zeros(N,1);
            lambda = zeros(N,1);            % penalty factor

            for ii = 1:N
                xc = [X(ii);xc(1:end-1)];
                yc(ii) = W(:,ii)'*xc;          % control signal
                ys = [yc(ii);ys(1:end-1)];
                Y(ii) = obj.SecPath'*ys;       % anti-noise
                E(ii) = Dis(ii) - Y(ii);       % residual error
%                 E(ii) = awgn(E(ii),10);
                xf = [fx(ii);xf(1:end-1)];     % filtered reference vector

                Pxf = [fx(ii)^2;Pxf(1:end-1)];
                Px = [X(ii)^2;Px(1:end-1)];
                Gs(ii) = max(sum(Pxf),0.3)/max(sum(Px),0.3);      % Gs calculation
                Pdis = [EstDis(ii)^2;Pdis(1:end-1)];
                eta = sum(Pdis)/(K*Gs(ii)*rho);
                lambda(ii) = max(Gs(ii)*(sqrt(eta)-1),0);

                obj.muw2 = obj.muw1*lambda(ii);
                W(:,ii+1) = W(:,ii) + obj.muw1*E(ii)*xf - obj.muw2*yc(ii)*xc;  % control filter updating


            end

        end
        
        function [W,E,yc,yc1,Gs,lambda,obj] = MOVFXLMS_PriPathEst_controller_v2(obj,Dis,X,rho)
            N = length(X);
            xc = zeros(obj.len,1);
            W = zeros(obj.len,N);
            ys = zeros(obj.slen,1);
            Y = zeros(N,1);
            yc = zeros(N,1);
            yc1 = zeros(N,1);
            E = zeros(N,1);
            Vthr = sqrt(2);
            fx = filter(obj.SecPath,1,X);  % filtered reference by secondary path
            xf = zeros(obj.len,1);
            EstDis = filter(obj.EstPriPath,1,X);  % estimated disturbance

            K = 1024;
            Pxf = zeros(K,1);               % Power of filtered reference
            Px = zeros(K,1);                % Power of reference
            Pdis = zeros(K,1);              % Power of disturbance
            Gs = zeros(N,1);
            lambda = zeros(N,1);            % penalty factor

            for ii = 1:N
                xc = [X(ii);xc(1:end-1)];
                yc(ii) = W(:,ii)'*xc;                % control signal
                yc1(ii) = Amplifier_model(Vthr,yc(ii)); % through amplifier model
                ys = [yc1(ii);ys(1:end-1)];
                Y(ii) = obj.SecPath'*ys;             % anti-noise
                E(ii) = Dis(ii) - Y(ii);             % residual error
%                 E(ii) = awgn(E(ii),10);
                xf = [fx(ii);xf(1:end-1)];           % filtered reference vector

                Pxf = [fx(ii)^2;Pxf(1:end-1)];
                Px = [X(ii)^2;Px(1:end-1)];
                Gs(ii) = max(sum(Pxf),0.3)/max(sum(Px),0.3);      % Gs calculation
                Pdis = [EstDis(ii)^2;Pdis(1:end-1)];
                eta = sum(Pdis)/(K*Gs(ii)*rho);
                lambda(ii) = max(Gs(ii)*(sqrt(eta)-1),0);

                obj.muw2 = obj.muw1*lambda(ii);
                W(:,ii+1) = W(:,ii) + obj.muw1*E(ii)*xf - obj.muw2*yc(ii)*xc;  % control filter updating


            end

        end


    end
end