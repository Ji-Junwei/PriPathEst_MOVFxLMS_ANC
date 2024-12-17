classdef LeakPower_FxLM
    properties 
        muW1
        muW2
        len
        Xd 
        Fx
        Sec
        Xf
        Yd
    end
    methods 
        function obj = LeakPower_FxLM(numW1,len,Sec,numW2)
            obj.muW1 = numW1 ;
            obj.muW2 = numW2 ;
            obj.len  = len   ;
            obj.Sec   = Sec   ;
            obj.Xd    = zeros(len,1);
            obj.Fx    = zeros(len,1);
            obj.Xf    = zeros(length(Sec),1);
            obj.Yd    = zeros(length(Sec),1);
        end
        function [W,E,Y,W_n2,obj] = LeakPower_controller(obj,Dir,X)
            Xref = filter(obj.Sec,1,X);
            N    = length(X)          ;
            E    = zeros(N,1)         ;
            W    = zeros(obj.len,N)   ;
            W_n2 = zeros(obj.len,1)   ;
            Y    = zeros(N,1)         ;
            for ii = 1:N
                obj.Xd = [X(ii);obj.Xd(1:end-1)]   ;
                obj.Fx = [Xref(ii);obj.Fx(1:end-1)];
                %------------------------------------
                % Filtering the signal
                %------------------------------------
                Y(ii)  = W(:,ii)'*obj.Xd           ;
                %------------------------------------
                % Generating the anti-noise wave
                %------------------------------------
                obj.Yd = [Y(ii);obj.Yd(1:end-1)];
                yt     = obj.Sec'* obj.Yd       ;
                %------------------------------------
                % Noise cancellation 
                %------------------------------------
                E(ii)  = Dir(ii) - yt           ;
                %------------------------------------
                % Control filter updating 
                %------------------------------------
                W(:,ii+1) = W(:,ii) + obj.muW1*E(ii)*obj.Fx - obj.muW2*Y(ii)*obj.Xd ;
                %------------------------------------
                W_n2(ii) = norm(W(:,ii+1));
            end
        end
    end
    
end