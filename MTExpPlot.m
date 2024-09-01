clc
close all

%% load the Monte Carlo Experiments result
load('MTE_results_StdEKF.mat','poseNum','feaNum', ...
    'DeltaR1XrFullSet','DeltaR2XrFullSet','DeltaR1XphiFullSet','DeltaR2XphiFullSet','DeltaFFullSet', ...
    'DeltaR2XpFullSet','R2PFullSet','DeltaR1XpFullSet','R1PFullSet')
load('MTE_results_IdeEKF.mat', ...
    'DeltaR1XrIdeFullSet','DeltaR2XrIdeFullSet','DeltaR1XphiIdeFullSet','DeltaR2XphiIdeFullSet','DeltaFIdeFullSet', ...
    'DeltaR2XpIdeFullSet','R2PIdeFullSet','DeltaR1XpIdeFullSet','R1PIdeFullSet')

%% Root Mean Square Error (RMSE)
% standard EKF
[R1XrRMSE,R1XrRMSE_mean] = XrRMSE(poseNum,mcNum,DeltaR1XrFullSet);
[R2XrRMSE,R2XrRMSE_mean] = XrRMSE(poseNum,mcNum,DeltaR2XrFullSet);

[R1XphiRMSE,R1XphiRMSE_mean] = XphiRMSE(poseNum,DeltaR1XphiFullSet);
[R2XphiRMSE,R2XphiRMSE_mean] = XphiRMSE(poseNum,DeltaR2XphiFullSet);

[FRMSE,FRMSE_mean] = XfRMSE(feaNum,mcNum,DeltaFFullSet);

fprintf('Std EKF R1 Position Err.RMS (m): %.2f \n',R1XrRMSE_mean);
fprintf('Std EKF R1 Heading Err.RMS (rad): %.2f \n',R1XphiRMSE_mean);
fprintf('Std EKF R2 Position Err.RMS (m): %.2f \n',R2XrRMSE_mean);
fprintf('Std EKF R2 Heading Err.RMS (rad): %.2f \n',R2XphiRMSE_mean);
fprintf('Std EKF Landmark Position Err.RMS (m): %.2f \n',FRMSE_mean);

% ideal EKF
[R1XrIdeRMSE,R1XrIdeRMSE_mean] = XrRMSE(poseNum,mcNum,DeltaR1XrIdeFullSet);
[R2XrIdeRMSE,R2XrIdeRMSE_mean] = XrRMSE(poseNum,mcNum,DeltaR2XrIdeFullSet);

[R1XphiIdeRMSE,R1XphiIdeRMSE_mean] = XphiRMSE(poseNum,DeltaR1XphiIdeFullSet);
[R2XphiIdeRMSE,R2XphiIdeRMSE_mean] = XphiRMSE(poseNum,DeltaR2XphiIdeFullSet);

[FIdeRMSE,FIdeRMSE_mean] = XfRMSE(feaNum,mcNum,DeltaFIdeFullSet);

fprintf('Ideal EKF R1 Position Err.RMS (m): %.2f \n',R1XrIdeRMSE_mean);
fprintf('Ideal EKF R1 Heading Err.RMS (rad): %.2f \n',R1XphiIdeRMSE_mean);
fprintf('Ideal EKF R2 Position Err.RMS (m): %.2f \n',R2XrIdeRMSE_mean);
fprintf('Ideal EKF R2 Heading Err.RMS (rad): %.2f \n',R2XphiIdeRMSE_mean);
fprintf('Ideal EKF Landmark Position Err.RMS (m): %.2f \n',FIdeRMSE_mean);

figure(1)
subplot(1,2,1)
hold on
% standard EKF
R1XrRMSEP = plot(R1XrRMSE(:,1)',R1XrRMSE(:,2)','-bo','DisplayName','R1 std EKF');
R2XrRMSEP = plot(R2XrRMSE(:,1)',R2XrRMSE(:,2)','-ro','DisplayName','R2 std EKF');

% ideal EKF
R1XrIdeRMSEP = plot(R1XrIdeRMSE(:,1)',R1XrIdeRMSE(:,2)','-co','DisplayName','R1 ideal EKF');
R2XrIdeRMSEP = plot(R2XrIdeRMSE(:,1)',R2XrIdeRMSE(:,2)','-mo','DisplayName','R2 ideal EKF');

xlabel('Steps')
ylabel('Position RMSE (m)')
xlim([0,poseNum])
legend([R1XrRMSEP,R2XrRMSEP,R1XrIdeRMSEP,R2XrIdeRMSEP])

hold off

subplot(1,2,2)
hold on
% standard EKF
R1XphiRMSEP = plot(R1XphiRMSE(:,1)',R1XphiRMSE(:,2)','-bo','DisplayName','R1 std EKF');
R2XphiRMSEP = plot(R2XphiRMSE(:,1)',R2XphiRMSE(:,2)','-ro','DisplayName','R2 std EKF');

% ideal EKF
R1XphiIdeRMSEP = plot(R1XphiIdeRMSE(:,1)',R1XphiIdeRMSE(:,2)','-co','DisplayName','R1 ideal EKF');
R2XphiIdeRMSEP = plot(R2XphiIdeRMSE(:,1)',R2XphiIdeRMSE(:,2)','-mo','DisplayName','R2 ideal EKF');

xlabel('Steps')
ylabel('Heading RMSE (m)')
xlim([0,poseNum])
legend([R1XphiRMSEP,R2XphiRMSEP,R1XphiIdeRMSEP,R2XphiIdeRMSEP])

hold off

%% Average Normalized (state) Estimation Error Squared (ANEES) 最后一步的
% standard EKF
R1NEES_k = [(1:poseNum)',zeros(poseNum,mcNum)];
R2NEES_k = [(0:poseNum)',zeros(poseNum+1,mcNum)];
for k = 0:poseNum
    
    if k == 34 || k == 60
        keyboard;
    end

    for mn = 1:mcNum
        R2xri_e = DeltaR2XpFullSet(k*3+(1:3),mn);
        R2C_xrie = R2PFullSet(k*3+(1:3),(mn-1)*3+(1:3));
        R2NEES_k(k+1,1+mn) = R2xri_e'/R2C_xrie*R2xri_e;

        if k == 0
            continue
        else
            R1xri_e = DeltaR1XpFullSet(k*3+(1:3),mn);
            R1C_xrie = R1PFullSet(k*3+(1:3),(mn-1)*3+(1:3));
            R1NEES_k(k,1+mn) = R1xri_e'/R1C_xrie*R1xri_e;
        end
    end
end

R1XpNEES = [R1NEES_k(:,1),mean(R1NEES_k(:,2:end),2)];
R2XpNEES = [R2NEES_k(:,1),mean(R2NEES_k(:,2:end),2)];

R1XpNEES_mean = mean(R1XpNEES(:,2));

R2XpNEES_mean = mean(R2XpNEES(:,2));

XfNEES_k = [XfFullSet(1:2:(end-1),1),zeros(feaNum,mcNum)];
for j = 1:feaNum
    for mn = 1:mcNum
        xfi_e = DeltaFFullSet((j-1)*2+(1:2),mn);
        C_xfie = PfFullSet((j-1)*2+(1:2),(mn-1)*feaNum*2+(j-1)*2+(1:2));
        XfNEES_k(j,1+mn) = xfi_e'/C_xfie*xfi_e;
    end
end

XfNEES = [XfNEES_k(:,1),mean(XfNEES_k(:,2:end),2)];
XfNEES_mean = mean(XfNEES(:,2));



fprintf('Std EKF R1 Pose ANEES: %.2f \n',R1XpNEES_mean);
fprintf('Std EKF R2 Pose ANEES: %.2f \n',R2XpNEES_mean);

% ideal EKF
R1IdeNEES_k = [(1:poseNum)',zeros(poseNum,mcNum)];
R2IdeNEES_k = [(0:poseNum)',zeros(poseNum+1,mcNum)];
for k = 0:poseNum
    
    if k == 34 || k == 60
        keyboard;
    end

    for mn = 1:mcNum
        R2Idexri_e = DeltaR2XpIdeFullSet(k*3+(1:3),mn);
        R2IdeC_xrie = R2PIdeFullSet(k*3+(1:3),(mn-1)*3+(1:3));
        R2IdeNEES_k(k+1,1+mn) = R2Idexri_e'/R2IdeC_xrie*R2Idexri_e;

        if k == 0
            continue
        else
            R1Idexri_e = DeltaR1XpIdeFullSet(k*3+(1:3),mn);
            R1IdeC_xrie = R1PIdeFullSet(k*3+(1:3),(mn-1)*3+(1:3));
            R1IdeNEES_k(k,1+mn) = R1Idexri_e'/R1IdeC_xrie*R1Idexri_e;
        end
    end
end

R1XpIdeNEES = [R1IdeNEES_k(:,1),mean(R1IdeNEES_k(:,2:end),2)];
R2XpIdeNEES = [R2IdeNEES_k(:,1),mean(R2IdeNEES_k(:,2:end),2)];

R1XpIdeNEES_mean = mean(R1XpIdeNEES(:,2));

R2XpIdeNEES_mean = mean(R2XpIdeNEES(:,2));

XfIdeNEES_k = [XfIdeFullSet(1:2:(end-1),1),zeros(feaNum,mcNum)];
for j = 1:feaNum
    for mn = 1:mcNum
        Idexfi_e = DeltaFIdeFullSet((j-1)*2+(1:2),mn);
        IdeC_xfie = PfFullSet((j-1)*2+(1:2),(mn-1)*feaNum*2+(j-1)*2+(1:2));
        XfIdeNEES_k(j,1+mn) = Idexfi_e'/IdeC_xfie*Idexfi_e;
    end
end

XfIdeNEES = [XfIdeNEES_k(:,1),mean(XfIdeNEES_k(:,2:end),2)];
XfIdeNEES_mean = mean(XfIdeNEES(:,2));



fprintf('Ideal EKF R1 Pose ANEES: %.2f \n',R1XpIdeNEES_mean);
fprintf('Ideal EKF R2 Pose ANEES: %.2f \n',R2XpIdeNEES_mean);

figure(2)
hold on

% standard EKF
R1XpNEESP = plot(R1XpNEES(:,1)',R1XpNEES(:,2)','-bo','DisplayName','R1 std EKF');
R2XpNEESP = plot(R2XpNEES(:,1)',R2XpNEES(:,2)','-ro','DisplayName','R2 std EKF');

% ideal EKF
R1XpIdeNEESP = plot(R1XpIdeNEES(:,1)',R1XpIdeNEES(:,2)','-co','DisplayName','R1 ideal EKF');
R2XpIdeNEESP = plot(R2XpIdeNEES(:,1)',R2XpIdeNEES(:,2)','-mo','DisplayName','R2 ideal EKF');

xlabel('Steps')
ylabel('Pose ANEES')
xlim([0,poseNum])
legend([R1XpNEESP,R2XpNEESP,R1XpIdeNEESP,R2XpIdeNEESP])
hold off