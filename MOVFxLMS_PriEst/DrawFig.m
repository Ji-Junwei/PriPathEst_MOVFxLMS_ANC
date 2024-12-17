clc;clear all;close all;



set(groot,'defaultAxesTickLabelInterpreter','latex');


%%
fs = 16000;
T = 1/fs;
L = 60*fs;
t = (0:L-1)*T;
N = length(t);
T  = 240        ;
t  = 0:1/fs:T ;
N  = length(t);
%% Case 1
% load('Case1_Data.mat');
% %% MSE
%  
% MSE_FXLMS = 10*log10(fxlms_e.^2);
% MSE_MOVFXLMS = 10*log10(E_MOVFXLMS.^2);
% MSE_MOVMFXLMS = 10*log10(E_movmfxlms.^2);
% MSE_ImMOVFXLMS = 10*log10(E_imovfxlms.^2);
% 
% 
% %% power of control signal
% 
% yFXLMS = smooth(fxlms_y.^2,1024*2);
% yMOVFXLMS = smooth(Y_MOVFXLMS.^2,1024*2);
% yMOVMFXLMS = smooth(yc_movmfxlms.^2,1024*2);
% yImMOVFXLMS = smooth(yc_imovfxlms.^2,1024*2);
% 
% 
% %% plot figure
% 
% figure;
% subplot(3,1,1);
% plot(t(100:2880000),smooth(MSE_FXLMS(100:2880000),2048));hold on;
% plot(t(100:2880000),smooth(MSE_MOVFXLMS(100:2880000),2048));hold on;
% plot(t(100:2880000),smooth(MSE_MOVMFXLMS(100:2880000),2048));hold on;
% plot(t(100:2880000),smooth(MSE_ImMOVFXLMS(100:2880000),2048));
% xlabel('Time (Second)','Interpreter','latex');
% ylabel('Magnitude (dB)','Interpreter','latex');
% % legend('FXLMS','MOVFXLMS','MOVMFXLMS','Improved MOVFXLMS','Interpreter','latex');
% title("(a).Time history of squared error",'Interpreter','latex');
% grid on;
% 
% 
% subplot(3,1,2);
% plot(t(1:2880000),yFXLMS(1:2880000));hold on;
% plot(t(1:2880000),yMOVFXLMS(1:2880000));hold on;
% plot(t(1:2880000),yMOVMFXLMS(1:2880000));hold on;
% plot(t(1:2880000),yImMOVFXLMS(1:2880000));
% yline(1,'LineWidth',3,'Color','#77AC30','Label','$\rho^2 = 1$','Interpreter','latex','LabelHorizontalAlignment','right','LabelVerticalAlignment','bottom');
% xlabel('Time (Second)','Interpreter','latex');
% ylabel('Power','Interpreter','latex');
% legend('FxLMS','MOV-FxLMS','MOV-MFxLMS','Improved MOV-FxLMS','Interpreter','latex');
% title("(b). Power of control signal",'Interpreter','latex');
% grid on;
% 
% % figure;
% subplot(3,1,3);
% s_low = 80 ;
% s_high = 5000;
% S = fir1(127,[2*s_low/fs 2*s_high/fs]);
% rho = 1;
% 
% [Gs1,Fx1] = Gs_Calculate(Ref(960002:1920000),S);                           % obtain Gs
% SimD1 = var(Dis(960002:1920000));  
% eta1 = SimD1/(Gs1*rho);
% lambda1 = max(Gs1*(sqrt(eta1)-1),0);                          % optimal alpha
% 
% [Gs2,Fx2] = Gs_Calculate(Ref(1920002:end),S);                           % obtain Gs
% SimD2 = var(Dis(1920002:end));  
% eta2 = SimD2/(Gs2*rho);
% lambda2 = max(Gs2*(sqrt(eta2)-1),0);                          % optimal alpha
% 
% label1 = "$\alpha_1 = $" + num2str(lambda1);
% label2 = "$\alpha_2 = $" + num2str(lambda2);
% % plot(t(1:1920000),smooth(lambda_movmfxlms(1:1920000),1024));hold on;
% plot(t(1:2880000),smooth(lambda_imovfxlms(1:2880000),1024));
% yline(lambda1,'-.','LineWidth',1,'Color','#7E2F8E','Label',label1,'Interpreter','latex','LabelHorizontalAlignment','right','LabelVerticalAlignment','bottom');
% yline(lambda2,'-.','LineWidth',1,'Color','#7E2F8E','Label',label2,'Interpreter','latex','LabelHorizontalAlignment','left','LabelVerticalAlignment','top');
% xlabel('Time (Second)','Interpreter','latex');
% ylabel('$\alpha$','Interpreter','latex');
% title("(c). Time-varying penalty factor for improved MOV-FxLMS",'Interpreter','latex');
% grid on;

%% Case 3


load('Case3_data_nointerference.mat');
load('Case3_data_hasinterference.mat');

%plot error fft figure
f = fs*(0:(N/2))/N;

Dis1fft = fft(Dis1);
P_Dis1 = 10*log10(abs(Dis1fft/N).^2);

MOVFXLMSfft = fft(E_MOVFXLMS1(15000:end));
P_MOVFXLMS = 10*log10(abs(MOVFXLMSfft/N).^2);

MOVMFXLMSfft = fft(E_movmfxlms1(15000:end));
P_MOVMFXLMS = 10*log10(abs(MOVMFXLMSfft/N).^2);

ImMOVFXLMSfft = fft(E_imovfxlms1(15000:end));
P_ImMOVFXLMS = 10*log10(abs(ImMOVFXLMSfft/N).^2);

powerymovfxlms = var(Y_MOVFXLMS1);
powerymovmfxlms = var(yc_movmfxlms1);
poweryimmovfxlms = var(yc_imovfxlms1);
leg1 = "MOV-FxLMS $\delta_y^2 = $" + num2str(powerymovfxlms);
leg2 = "MOV-MFxLMS $\delta_y^2 = $" + num2str(powerymovmfxlms);
leg3 = "Improved MOV-FxLMS $\delta_y^2 = $" + num2str(poweryimmovfxlms);


subplot(2,1,1);
plot(f,smooth(P_Dis1(1:N/2+1),320),'Color','#78C0A8');hold on;
plot(f,smooth(P_MOVFXLMS(1:N/2+1),320),'Color','#D95319');hold on;
plot(f,smooth(P_MOVMFXLMS(1:N/2+1),320),'Color','#EDB120');hold on;
plot(f,smooth(P_ImMOVFXLMS(1:N/2+1),320),'Color','#7E2F8E');
axis([0 3000 -90 inf]);
grid on;
xlabel('Frequency (Hz)','Interpreter','latex');
ylabel('Power (dB/Hz)','Interpreter','latex');
title("(a). Power spectrum of error signal",'Interpreter','latex');
legend('ANC off',leg1,leg2,leg3,'Interpreter','latex');

label1 = "Optimal penalty factor $\alpha = $" + num2str(lambda_movfxlms);
alphamovfxlms = ones(N,1)*lambda_movfxlms1;
subplot(2,1,2);
plot(t(10000:3840000),smooth(alphamovfxlms(10000:3840000),30000),'LineWidth',1.5,'Color','#D95319');hold on;
plot(t(10000:3840000),smooth(lambda_movmfxlms1(10000:3840000),30000),'Color','#EDB120');hold on;
plot(t(10000:3840000),smooth(lambda_imovfxlms1(10000:3840000),30000),'Color','#7E2F8E');
axis([0 240 0.05 inf]);
yline(lambda_movfxlms,'-.','LineWidth',1.5,'Color','#FF009E');
legend('MOV-FxLMS','MOV-MFxLMS','Improved MOV-FxLMS',label1,'Interpreter','latex');
xlabel('Time (Second)','Interpreter','latex');
ylabel('$\alpha$','Interpreter','latex');
title("(b). Penalty factor",'Interpreter','latex');
grid on;


%% plot circle

%Stotistical analysis 
% [H,Sigma_e] = corrmtx(Dis,1);
% Sig_e       = Sigma_e(1,1)  ;
% [H,Rx]      = corrmtx(Fx ,2);
% [Cor_d,lag] = xcorr(Dis,Fx,3,'biased')     ;
% Px          = [Cor_d(lag==0);Cor_d(lag==1)];
% Rx          = Rx(1:2,1:2);
% 
% [H,R]      = corrmtx(Ref ,2);
% R          = R(1:2,1:2)    ;
% eta = var(Dis)/(Gs_movfxlms*1);
% Gamma = max(Gs_movfxlms*(sqrt(eta)-1),0)*R;
% WoptConstrain = (Gamma + Rx)\Px;
% Wopt = Rx\Px;
% 
% x = linspace(-0,2,200);
% y = linspace(-0,2,200);
% [X,Y] = meshgrid(x,y);
% index = size(X);
% index = index(1);
% J     = zeros(index, index);
% for tt = 1:index
%     for jj = 1:index
%         W = [X(tt,jj);Y(tt,jj)];
%         J(tt,jj) = Sig_e-2*W'*Px+W'*Rx*W;
%     end 
% end
% 
% %----------------------------
% J2     = zeros(index, index);
% for tt = 1:index
%     for jj = 1:index
%         W = [X(tt,jj);Y(tt,jj)];
%         J2(tt,jj) = W'*R*W;
%     end 
% end
% %----------------------------
% 
% figure;
% subplot(1,2,1);
% scatter(Wopt(1),Wopt(2),'kh','LineWidth',1);
% hold on;
% scatter(WoptConstrain(1),WoptConstrain(2),'r*');
% hold on;
% contour(X,Y,J2,[1 1],'r--');
% hold on;
% plot(WT_MOVFXLMS(1,:),WT_MOVFXLMS(2,:),'Color','#D95319','LineWidth',1);hold on;
% plot(WT_movmfxlms(1,:),WT_movmfxlms(2,:),'Color','#EDB120');hold on;
% plot(WT_imovfxlms(1,:),WT_imovfxlms(2,:),'Color','#7E2F8E');hold on;
% contour(X,Y,J,25,':');
% legend('Optimal solution without constraint','Optimal solution with constraint','Constraint $\rho^2=1$','MOV-FxLMS','MOV-MFxLMS','Improved MOV-FxLMS','Interpreter','latex');
% xlabel('$\mathrm{w}_1(n)$','Interpreter','latex');
% ylabel('$\mathrm{w}_2(n)$','Interpreter','latex');
% title('(a). Weight convergence paths without interference at error','Interpreter','latex');
% grid on;

% subplot(1,2,2);
% scatter(Wopt(1),Wopt(2),'kh','LineWidth',1);
% hold on;
% scatter(WoptConstrain(1),WoptConstrain(2),'r*');
% hold on;
% contour(X,Y,J2,[1 1],'r--');
% hold on;
% plot(WT_MOVFXLMS1(1,:),WT_MOVFXLMS1(2,:),'Color','#D95319','LineWidth',1);hold on;
% plot(WT_movmfxlms1(1,:),WT_movmfxlms1(2,:),'Color','#EDB120');hold on;
% plot(WT_imovfxlms1(1,:),WT_imovfxlms1(2,:),'Color','#7E2F8E');hold on;
% contour(X,Y,J,25,':');
% %legend('Optimal solution without constraint','Optimal solution with constraint','Constraint $\rho^2=1$','MOVFXLMS','MOVMFXLMS','Improved MOVFXLMS','Interpreter','latex');
% xlabel('$\mathrm{w}_1(n)$','Interpreter','latex');
% ylabel('$\mathrm{w}_2(n)$','Interpreter','latex');
% title('(b). Weight convergence paths with interference at error','Interpreter','latex');
% grid on;

%% Case 2
% load("Case2_data.mat");
% %% MSE
% 
% MSE_FXLMS = 10*log10(fxlms_e.^2);
% MSE_MOVFXLMS = 10*log10(E_MOVFXLMS.^2);
% MSE_MOVMFXLMS = 10*log10(E_movmfxlms.^2);
% MSE_ImMOVFXLMS = 10*log10(E_imovfxlms.^2);
% 
% % 
% %% power of control signal
% 
% yFXLMS = smooth(fxlms_y.^2,1024*2);
% yMOVFXLMS = smooth(Y_MOVFXLMS.^2,1024*2);
% yMOVMFXLMS = smooth(yc_movmfxlms.^2,1024*2);
% yImMOVFXLMS = smooth(yc_imovfxlms.^2,1024*2);
% 
% 
% %% plot figure
% 
% figure;
% subplot(2,1,1);
% plot(t(1:683031),Dis,'Color','#78C0A8');hold on; 
% plot(t(1:683031),E_movmfxlms,'Color','#EDB120');hold on; 
% plot(t(1:683031),E_imovfxlms,'Color','#7E2F8E');hold on;
% plot(t(1:683031),E_MOVFXLMS,'Color','#D95319');hold on;
% plot(t(1:683031),fxlms_e,'Color','#0072BD');
% legend('Disturbance','Interpreter','latex');
% axis([0 42 -8 8]);
% title('(a). Error Signal','Interpreter','latex');
% xlabel('Time (Second)','Interpreter','latex');
% ylabel('Amplitude','Interpreter','latex');
% grid on;
% 
% 
% subplot(2,1,2);
% 
% plot(t(1:683031),yFXLMS(1:683031),'Color','#0072BD');hold on
% plot(t(1:683031),yMOVFXLMS(1:683031),'Color','#D95319');hold on;
% plot(t(1:683031),yMOVMFXLMS(1:683031),'Color','#EDB120');hold on;
% plot(t(1:683031),yImMOVFXLMS(1:683031),'Color','#7E2F8E');
% axis([0 42 -inf inf]);
% yline(0.8,'LineWidth',1,'Color','#FF009E');
% xlabel('Time (Second)','Interpreter','latex');
% ylabel('Power','Interpreter','latex');
% %legend('MOV-MFxLMS','Improved MOV-FxLMS','MOV-FxLMS','FxLMS','Interpreter','latex');
% legend('FxLMS','MOV-FxLMS','MOV-MFxLMS','Improved MOV-FxLMS','Power constrain $\rho^2 = 0.8$','Interpreter','latex');
% title("(b). Power of control signal",'Interpreter','latex');
% grid on;

% figure;
% subplot(2,1,1);
% s_low = 80 ;
% s_high = 5000;
% S = fir1(127,[2*s_low/fs 2*s_high/fs]);
% rho = 1;
% 
% [Gs1,Fx1] = Gs_Calculate(Ref(1:960000),S);                           % obtain Gs
% SimD1 = var(Dis(1:960000));  
% eta1 = SimD1/(Gs1*rho);
% lambda1 = max(Gs1*(sqrt(eta1)-1),0);                          % optimal alpha
% 
% [Gs2,Fx2] = Gs_Calculate(Ref(960002:end),S);                           % obtain Gs
% SimD2 = var(Dis(960002:end));  
% eta2 = SimD2/(Gs2*rho);
% lambda2 = max(Gs2*(sqrt(eta2)-1),0);                          % optimal alpha
% 
% label1 = "$\alpha_1 = $" + num2str(lambda1);
% label2 = "$\alpha_2 = $" + num2str(lambda2);
% % plot(t(1:1920000),smooth(lambda_movmfxlms(1:1920000),1024));hold on;
% plot(t(1:1920000),smooth(lambda_imovfxlms(1:1920000),1024));
% yline(lambda1,'-.','LineWidth',1,'Color','#7E2F8E','Label',label1,'Interpreter','latex','LabelHorizontalAlignment','right','LabelVerticalAlignment','bottom');
% yline(lambda2,'-.','LineWidth',1,'Color','#7E2F8E','Label',label2,'Interpreter','latex','LabelHorizontalAlignment','left','LabelVerticalAlignment','top');
% xlabel('Time (Second)','Interpreter','latex');
% ylabel('$\alpha$','Interpreter','latex');
% title("Time-varying penalty factor",'Interpreter','latex');
% grid on;

%% Case 4
% load("case4_1_data.mat");
% 
% % MSE
% 
% % MSE_FXLMS = 10*log10(e_fxlms.^2);
% % MSE_MOVFXLMS = 10*log10(E.^2);
% 
% % 2-norm control filter
% for i = 1:N
%     WT_norm(i) = norm(WT(:,i),2);
%     Wfxlms_norm(i) = norm(W_fxlms(i,:),2);
% end
% 
% 
% % fft
% L = N;
% f = fs*(0:(N/2))/N;
% 
% % Y = fft(E(1:end-1));
% % P = 10*log10(abs(Y/N).^2);
% % P2 = abs(Y/L);
% % P1 = P2(1:L/2+1);
% % P1(2:end-1) = 2*P1(2:end-1);
% % 
% % Y_fxlms = fft(e_fxlms(1:end-1));
% % P_fxlms = 10*log10(abs(Y_fxlms/N).^2);
% % P2_fxlms = abs(Y_fxlms/L);
% % P1_fxlms = P2_fxlms(1:L/2+1);
% % P1_fxlms(2:end-1) = 2*P1_fxlms(2:end-1);
% 
% [Pxx1,wf1] = pwelch(awgn(E,45));
% [Pxx2,wf2] = pwelch(awgn(e_fxlms,45));
% 
% 
% figure;
% subplot(2,2,1);
% % plot(t(200:41*fs),smooth(MSE_FXLMS(200:41*fs),2048));hold on;
% % plot(t(200:41*fs),smooth(MSE_MOVFXLMS(200:41*fs),2048));hold on;
% plot(wf2/pi*fs/2,smooth(10*log10(Pxx2),32));hold on;
% plot(wf1/pi*fs/2,smooth(10*log10(Pxx1),32),'Color','#7E2F8E');
% xlabel('Frequency (Hz)','Interpreter','latex');
% ylabel('Amplitude','Interpreter','latex');
% legend('FxLMS','Improved MOV-FxLMS','Interpreter','latex');
% title("(a). Amplitude spectrum of error signal",'Interpreter','latex');
% grid on;
% axis([0 5000 -60 -3]);
% 
% T  = 60        ;
% t  = 0:1/fs:T ;
% N  = length(t);
% 
% subplot(2,2,2);
% plot(t,Cy1_fxlms);hold on;
% plot(t,yc1,'Color','#7E2F8E');
% xlim([0.5 0.52]);
% yline(sqrt(2),'-.','LineWidth',1,'Color','#A2142F','Label','Amplitude constraint $A = \sqrt{2}$','Interpreter','latex','LabelHorizontalAlignment','right','LabelVerticalAlignment','top');
% yline(-sqrt(2),'-.','LineWidth',1,'Color','#A2142F','Label','Amplitude constraint $A = -\sqrt{2}$','Interpreter','latex','LabelHorizontalAlignment','right','LabelVerticalAlignment','bottom');
% ylim([-2 2]);
% xlabel('Time (Second)','Interpreter','latex');
% ylabel('Magnitude','Interpreter','latex');
% %legend('FXLMS','Improved MOV-FXLMS','Interpreter','latex');
% title("(b). The control signal output from secondary source",'Interpreter','latex');
% grid on;
% 
% subplot(2,2,3);
% plot(t,e_fxlms);hold on;
% plot(t,E,'Color','#7E2F8E');
% xlim([0.5 0.52]);
% xlabel('Time (Second)','Interpreter','latex');
% ylabel('Magnitude','Interpreter','latex');
% %legend('FXLMS','Improved MOV-FXLMS','Interpreter','latex');
% title("(c). The error signal",'Interpreter','latex');
% grid on;
% 
% 
% 
% subplot(2,2,4);
% plot(t(1:40*fs),Wfxlms_norm(1:40*fs));hold on;
% plot(t(1:40*fs),WT_norm(1:40*fs),'Color','#7E2F8E');
% xlabel('Time (Second)','Interpreter','latex');
% ylabel('$||\mathbf{w}(n)||_2$','Interpreter','latex');
% %legend('FXLMS','Improved MOV-FXLMS','Interpreter','latex');
% title("(d). Time history of control filter",'Interpreter','latex');
% grid on;


% %Stotistical analysis 
% [H,Sigma_e] = corrmtx(Dis,1);
% Sig_e       = Sigma_e(1,1)  ;
% [H,Rx]      = corrmtx(Fx ,2);
% [Cor_d,lag] = xcorr(Dis,Fx,3,'biased')     ;
% Px          = [Cor_d(lag==0);Cor_d(lag==1)];
% Rx          = Rx(1:2,1:2);
% 
% [H,R]      = corrmtx(Ref ,2);
% R          = R(1:2,1:2)    ;
% eta = var(Dis)/(Gs(end)*1);
% Gamma = max(Gs(end)*(sqrt(eta)-1),0)*R;
% WoptConstrain = (Gamma + Rx)\Px;
% Wopt = Rx\Px;
% 
% x = linspace(-0,2,200);
% y = linspace(-0,2,200);
% [X,Y] = meshgrid(x,y);
% index = size(X);
% index = index(1);
% J     = zeros(index, index);
% for tt = 1:index
%     for jj = 1:index
%         W = [X(tt,jj);Y(tt,jj)];
%         J(tt,jj) = Sig_e-2*W'*Px+W'*Rx*W;
%     end 
% end
% 
% %----------------------------
% J2     = zeros(index, index);
% for tt = 1:index
%     for jj = 1:index
%         W = [X(tt,jj);Y(tt,jj)];
%         J2(tt,jj) = W'*R*W;
%     end 
% end
% %----------------------------
% 
% % figure;
% % subplot(1,2,1);
% scatter(Wopt(1),Wopt(2),'kh','LineWidth',1);
% hold on;
% scatter(WoptConstrain(1),WoptConstrain(2),'r*');
% hold on;
% contour(X,Y,J2,[1 1],'r--');
% hold on;
% plot(W_fxlms(:,1),W_fxlms(:,2),'Color','#0072BD');hold on;
% plot(WT(1,:),WT(2,:),'Color',"#D95319");hold on;
% contour(X,Y,J,25,':');
% axis([0 2 0 2]);
% legend('Optimal solution without constraint','Optimal solution with constraint','Constraint $\rho^2=1$','Interpreter','latex');
% xlabel('$\mathrm{w}_1(n)$','Interpreter','latex');
% ylabel('$\mathrm{w}_2(n)$','Interpreter','latex');
% title('(d). Weight convergence paths','Interpreter','latex');
% grid on;

