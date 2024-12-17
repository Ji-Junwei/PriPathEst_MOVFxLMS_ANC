%FxLMS off-line path estimation feedforward system
function [y,Cy,Cy1,e,Sew,Cw]=FxLMS(x,mus,muf,Ms,Mf,Dis,Sw,varargin)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   input
%        x : reference signal (noise)
%        mus: step size for secondary path
%        muf: step size for fxlms
%        M : length of filter coefficients
%        Pw: Actual primary path (from source to the error sensor)
%        Sw: Actual Secondary path (from control source to the error sensor)
%      optional:
%        type: type of FxLMS , default: "original"--FxLMS;
%        "normalised"--FxNLMS; "leaky"--leaky FxLMS
%        beta: normalised factor  default 0
%        alpha: leaky fator    default 0
%   
%   output
%       y: filtered output
%       e: error signal
%       Sew: estimated secondary path coefficients
%       Cw； control weights
%
%  Structure of FxLMS
%
%
%              +-----------+                       +   
% x(k) ---+--->|   P(z)    |--xp(k)----------------> sum --+---> e(k)
%         |    +-----------+                          ^-   |
%         |                                           |    |
%         |        \                                y(k)   |     
%         |    +-----------+          +-----------+   |    |
%         +--->|   C(z)    |--yc(k)-->|   S(z)    |---+    |
%         |    +-----------+          +-----------+        |
%         |            \                                   |
%         |             \----------------\                 |
%         |                               \                |
%         |    +-----------+          +-----------+        |
%         +--->|   Se(z)   |--xs(k)-->|   FxLMS   |<-------+
%              +-----------+          +-----------+        
% 
%       x(k):  reference siganl/noise source
%       xp(k): response of Primary path/desired signal of x(k)
%       xs(k): estimated response of Secondary path of x(k)
%       yc(k): control signal
%       y(k):  response of Secondary path of yc(k)
%       e(k):  error signal
%       P(Z):  transfer function of Primary signal
%       C(Z):  control weights
%       Se(Z): estimated transfer function of Secondary path
%       S(Z):  transfer function of Secondary path (in reality)
%      
%      original code from matlab uploaded by Agustinus Oey <oeyaugust@gmail.com>
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%% Initialization

T=length(x);   %duration
Mus=mus;       %step size of secondary path
Muf=muf;       %step size of FxLMS
Nums=Ms;       %number of filter coefficients secondary
Numf=Mf;       %number of filter coefficients fxlms
% Pz=Pw;         %Primary path
% Sz=Sw;         %Secondary path

p=inputParser;
p.addParameter('type','original');
p.addParameter('beta',0.001);
p.addParameter('alpha',0);
parse(p,varargin{:});
type=p.Results.type;                %choose type of fxlms, default fxlms
beta=p.Results.beta;                %beta default 0.001 
alpha=p.Results.alpha;              %leaky factor alpha

%% first, estimate secondary path.
% x_iden=randn(1,T); %generate white noise
% 
% %send x_iden to actuator, and mearsure it by sensor (error sensor)
% y_iden=filter(Sz,1,x_iden); % error sensor measured
% 
% %set some buffer
% Sx_idne=zeros(1,Nums); %the state of Sew
% Sew=zeros(1,Nums); %the weight of Sew
% e_iden=zeros(1,T); %data buffer for the identification error
% 
% %estimate secondary path by LMS algorithm
% 
% for k=1:T
%     Sx_idne=[x_iden(k) Sx_idne(1:(Nums-1))]; %update state
%     Shy=sum(Sx_idne.*Sew);              %calculate output of sez
%     e_iden(k)=y_iden(k)-Shy;        %calculate error
%     Sew=Sew+Mus*e_iden(k)*Sx_idne;       %adjust the weight
% end

Sew = Sw;

%% do the ANC use FxLMS
X=x;  %reference signal

Xp=Dis; %primary response sensed by error transducer

%set the system
Cx=zeros(1,Numf);     %the state of Cz
Cw=zeros(T,Numf);     %the weight of Cz
Sy=zeros(size(Sw));  %the state of Sz
e=zeros(1,T);        %data buffer for error
y=zeros(1,T);        %response of secondary path
Xf=zeros(1,Numf);     %estimate secondary path xsk
Cx1=zeros(1,Nums);     %the state of Sez
Cy = zeros(1,T);
Cy1 = zeros(1,T);
Vthr = sqrt(2);

switch type
    case 'original'
    %start FxLMS
        for k=1:T
            Cx=[X(k) Cx(1:(Numf-1))];             %update controller state
            Cy(k)=sum(Cx.*Cw(k,:));                       %controller output yc
            Sy=[Cy(k) Sy(1:(length(Sy)-1))];         %update state state of Sz
            y(k)=sum(Sy.*Sw);                     %filtered output 
            e(k)=Xp(k)-sum(Sy.*Sw);               %sense the error
            Cx1=[X(k) Cx1(1:(Nums-1))];           %update state of sez
            Xf=[sum(Cx1.*Sew) Xf(1:(Numf-1))];    %estimate secondary path xsk
            Cw((k+1),:)=Cw(k,:)+Muf*e(k)*Xf;                    %adjust the controller weight
        end

     case 'OutputSaturation'
    %start FxLMS
        for k=1:T
            Cx=[X(k) Cx(1:(Numf-1))];             %update controller state
            Cy(k)=sum(Cx.*Cw(k,:));                       %controller output yc
            Cy1(k) = Amplifier_model(Vthr,Cy(k));
            Sy=[Cy1(k) Sy(1:(length(Sy)-1))];         %update state state of Sz
            y(k)=sum(Sy.*Sw);                     %filtered output 
            e(k)=Xp(k)-sum(Sy.*Sw);               %sense the error
            Cx1=[X(k) Cx1(1:(Nums-1))];           %update state of sez
            Xf=[sum(Cx1.*Sew) Xf(1:(Numf-1))];    %estimate secondary path xsk
            Cw((k+1),:)=Cw(k,:)+Muf*e(k)*Xf;                    %adjust the controller weight
        end
    case 'normalised'
    %start FxNLMS
        for k=1:T
            Cx=[X(k) Cx(1:(Numf-1))];             %update controller state
            Cy=sum(Cx.*Cw);                       %controller output yc
            Sy=[Cy Sy(1:(length(Sy)-1))];         %update state state of Sz
            y(k)=sum(Sy.*Sw);                     %filtered output 
            e(k)=Xp(k)-sum(Sy.*Sw);               %sense the error
            Cx1=[X(k) Cx1(1:(Nums-1))];           %update state of sez
            Xf=[sum(Cx1.*Sew) Xf(1:(Numf-1))];    %estimate secondary path xsk
            Cw=Cw+(Muf/(Cx*Cx'+beta))*e(k)*Xf;    %adjust the controller weight use normalised step size
        end

    case 'leaky'
    %start leaky FxLMS
        for k=1:T
            Cx=[X(k) Cx(1:(Numf-1))];             %update controller state
            Cy=sum(Cx.*Cw);                       %controller output yc
            Sy=[Cy Sy(1:(length(Sy)-1))];         %update state state of Sz
            y(k)=sum(Sy.*Sw);                     %filtered output 
            e(k)=Xp(k)-sum(Sy.*Sw);               %sense the error
            Cx1=[X(k) Cx1(1:(Nums-1))];           %update state of sez
            Xf=[sum(Cx1.*Sew) Xf(1:(Numf-1))];    %estimate secondary path xsk
            Cw=(1-Muf*alpha)*Cw+Muf*e(k)*Xf;      %adjust the controller weight using leaky factor
        end
end

end


