clear
close all

Config;

%% save output figures
currentFolder = fileparts(mfilename('fullpath'));
subFolder = 'saved_figures';
figuresFolderPath = fullfile(currentFolder, subFolder);
if ~exist(figuresFolderPath, 'dir')
    mkdir(figuresFolderPath);
end



%% load the Monte Carlo Experiments result

if ec == 1
    if env == 1
        load('MTE_results_StdEKF_20fea_1.mat','poseNum','feaNum', ...
            'DeltaR1XrFullSet','DeltaR2XrFullSet','DeltaR1XphiFullSet','DeltaR2XphiFullSet', ...
            'DeltaR2XpFullSet','R2PFullSet','DeltaR1XpFullSet','R1PFullSet', ...
            'DeltaXfFullSet','PfFullSet')
        load('MTE_results_IdeEKF_20fea_1.mat', ...
            'DeltaR1XrIdeFullSet','DeltaR2XrIdeFullSet','DeltaR1XphiIdeFullSet','DeltaR2XphiIdeFullSet', ...
            'DeltaR2XpIdeFullSet','R2PIdeFullSet','DeltaR1XpIdeFullSet','R1PIdeFullSet', ...
            'DeltaXfIdeFullSet','PfIdeFullSet')
        load('MTE_results_FejEKF_20fea_1.mat', ...
            'DeltaR1XrFejFullSet','DeltaR2XrFejFullSet','DeltaR1XphiFejFullSet','DeltaR2XphiFejFullSet', ...
            'DeltaR2XpFejFullSet','R2PFejFullSet','DeltaR1XpFejFullSet','R1PFejFullSet', ...
            'DeltaXfFejFullSet','PfFejFullSet')
    else
        load('MTE_results_StdEKF_20fea_2.mat','poseNum','feaNum', ...
            'DeltaR1XrFullSet','DeltaR2XrFullSet','DeltaR1XphiFullSet','DeltaR2XphiFullSet', ...
            'DeltaR2XpFullSet','R2PFullSet','DeltaR1XpFullSet','R1PFullSet', ...
            'DeltaXfFullSet','PfFullSet')
        load('MTE_results_IdeEKF_20fea_2.mat', ...
            'DeltaR1XrIdeFullSet','DeltaR2XrIdeFullSet','DeltaR1XphiIdeFullSet','DeltaR2XphiIdeFullSet', ...
            'DeltaR2XpIdeFullSet','R2PIdeFullSet','DeltaR1XpIdeFullSet','R1PIdeFullSet', ...
            'DeltaXfIdeFullSet','PfIdeFullSet')
        load('MTE_results_FejEKF_20fea_2.mat', ...
            'DeltaR1XrFejFullSet','DeltaR2XrFejFullSet','DeltaR1XphiFejFullSet','DeltaR2XphiFejFullSet', ...
            'DeltaR2XpFejFullSet','R2PFejFullSet','DeltaR1XpFejFullSet','R1PFejFullSet', ...
            'DeltaXfFejFullSet','PfFejFullSet')
    end
elseif ec == 2
    if env == 1
        load('MTE_results_StdEKF_60fea_1.mat','poseNum','feaNum', ...
            'DeltaR1XrFullSet','DeltaR2XrFullSet','DeltaR1XphiFullSet','DeltaR2XphiFullSet', ...
            'DeltaR2XpFullSet','R2PFullSet','DeltaR1XpFullSet','R1PFullSet', ...
            'DeltaXfFullSet','PfFullSet')
        load('MTE_results_IdeEKF_60fea_1.mat', ...
            'DeltaR1XrIdeFullSet','DeltaR2XrIdeFullSet','DeltaR1XphiIdeFullSet','DeltaR2XphiIdeFullSet', ...
            'DeltaR2XpIdeFullSet','R2PIdeFullSet','DeltaR1XpIdeFullSet','R1PIdeFullSet', ...
            'DeltaXfIdeFullSet','PfIdeFullSet')
        load('MTE_results_FejEKF_60fea_1.mat', ...
            'DeltaR1XrFejFullSet','DeltaR2XrFejFullSet','DeltaR1XphiFejFullSet','DeltaR2XphiFejFullSet', ...
            'DeltaR2XpFejFullSet','R2PFejFullSet','DeltaR1XpFejFullSet','R1PFejFullSet', ...
            'DeltaXfFejFullSet','PfFejFullSet')
    else
        load('MTE_results_StdEKF_60fea_2.mat','poseNum','feaNum', ...
            'DeltaR1XrFullSet','DeltaR2XrFullSet','DeltaR1XphiFullSet','DeltaR2XphiFullSet', ...
            'DeltaR2XpFullSet','R2PFullSet','DeltaR1XpFullSet','R1PFullSet', ...
            'DeltaXfFullSet','PfFullSet')
        load('MTE_results_IdeEKF_60fea_2.mat', ...
            'DeltaR1XrIdeFullSet','DeltaR2XrIdeFullSet','DeltaR1XphiIdeFullSet','DeltaR2XphiIdeFullSet', ...
            'DeltaR2XpIdeFullSet','R2PIdeFullSet','DeltaR1XpIdeFullSet','R1PIdeFullSet', ...
            'DeltaXfIdeFullSet','PfIdeFullSet')
        load('MTE_results_FejEKF_60fea_2.mat', ...
            'DeltaR1XrFejFullSet','DeltaR2XrFejFullSet','DeltaR1XphiFejFullSet','DeltaR2XphiFejFullSet', ...
            'DeltaR2XpFejFullSet','R2PFejFullSet','DeltaR1XpFejFullSet','R1PFejFullSet', ...
            'DeltaXfFejFullSet','PfFejFullSet')
    end
elseif ec == 3
    if env == 1
        load('MTE_results_StdEKF_100fea_1.mat','poseNum','feaNum', ...
            'DeltaR1XrFullSet','DeltaR2XrFullSet','DeltaR1XphiFullSet','DeltaR2XphiFullSet', ...
            'DeltaR2XpFullSet','R2PFullSet','DeltaR1XpFullSet','R1PFullSet', ...
            'DeltaXfFullSet','PfFullSet')
        load('MTE_results_IdeEKF_100fea_1.mat', ...
            'DeltaR1XrIdeFullSet','DeltaR2XrIdeFullSet','DeltaR1XphiIdeFullSet','DeltaR2XphiIdeFullSet', ...
            'DeltaR2XpIdeFullSet','R2PIdeFullSet','DeltaR1XpIdeFullSet','R1PIdeFullSet', ...
            'DeltaXfIdeFullSet','PfIdeFullSet')
        load('MTE_results_FejEKF_100fea_1.mat', ...
            'DeltaR1XrFejFullSet','DeltaR2XrFejFullSet','DeltaR1XphiFejFullSet','DeltaR2XphiFejFullSet', ...
            'DeltaR2XpFejFullSet','R2PFejFullSet','DeltaR1XpFejFullSet','R1PFejFullSet', ...
            'DeltaXfFejFullSet','PfFejFullSet')
    else
        load('MTE_results_StdEKF_100fea_2.mat','poseNum','feaNum', ...
            'DeltaR1XrFullSet','DeltaR2XrFullSet','DeltaR1XphiFullSet','DeltaR2XphiFullSet', ...
            'DeltaR2XpFullSet','R2PFullSet','DeltaR1XpFullSet','R1PFullSet', ...
            'DeltaXfFullSet','PfFullSet')
        load('MTE_results_IdeEKF_100fea_2.mat', ...
            'DeltaR1XrIdeFullSet','DeltaR2XrIdeFullSet','DeltaR1XphiIdeFullSet','DeltaR2XphiIdeFullSet', ...
            'DeltaR2XpIdeFullSet','R2PIdeFullSet','DeltaR1XpIdeFullSet','R1PIdeFullSet', ...
            'DeltaXfIdeFullSet','PfIdeFullSet')
        load('MTE_results_FejEKF_100fea_2.mat', ...
            'DeltaR1XrFejFullSet','DeltaR2XrFejFullSet','DeltaR1XphiFejFullSet','DeltaR2XphiFejFullSet', ...
            'DeltaR2XpFejFullSet','R2PFejFullSet','DeltaR1XpFejFullSet','R1PFejFullSet', ...
            'DeltaXfFejFullSet','PfFejFullSet')
    end
elseif ec == 4
    load('VicP_results_StdEKF.mat','poseNum','feaNum', ...
        'DeltaR1XrFullSet','DeltaR2XrFullSet','DeltaR1XphiFullSet','DeltaR2XphiFullSet', ...
        'DeltaR2XpFullSet','R2PFullSet','DeltaR1XpFullSet','R1PFullSet', ...
        'DeltaXfFullSet','PfFullSet')
    % load('VicP_results_IdeEKF.mat','poseNum','feaNum', ...
    %     'DeltaR1XrIdeFullSet','DeltaR2XrIdeFullSet','DeltaR1XphiIdeFullSet','DeltaR2XphiIdeFullSet', ...
    %     'DeltaR2XpIdeFullSet','R2PIdeFullSet','DeltaR1XpIdeFullSet','R1PIdeFullSet', ...
    %     'DeltaXfIdeFullSet','PfIdeFullSet')
    load('VicP_results_FejEKF.mat','poseNum','feaNum', ...
        'DeltaR1XrFejFullSet','DeltaR2XrFejFullSet','DeltaR1XphiFejFullSet','DeltaR2XphiFejFullSet', ...
        'DeltaR2XpFejFullSet','R2PFejFullSet','DeltaR1XpFejFullSet','R1PFejFullSet', ...
        'DeltaXfFejFullSet','PfFejFullSet')

    mcNum = 1;
end

%% Root Mean Square Error (RMSE)
% standard EKF
[R1XrRMSE,R1XrRMSE_mean] = XrRMSE(poseNum-1,mcNum,DeltaR1XrFullSet);
[R2XrRMSE,R2XrRMSE_mean] = XrRMSE(poseNum,mcNum,DeltaR2XrFullSet);

[R1XphiRMSE,R1XphiRMSE_mean] = XphiRMSE(poseNum-1,mcNum,DeltaR1XphiFullSet);
[R2XphiRMSE,R2XphiRMSE_mean] = XphiRMSE(poseNum,mcNum,DeltaR2XphiFullSet);

[XfStdRMSE,XfStdRMSE_mean] = XFRMSE(feaNum,mcNum,DeltaXfFullSet);



% ideal EKF
if ec ~= 4
    [R1XrIdeRMSE,R1XrIdeRMSE_mean] = XrRMSE(poseNum-1,mcNum,DeltaR1XrIdeFullSet);
    [R2XrIdeRMSE,R2XrIdeRMSE_mean] = XrRMSE(poseNum,mcNum,DeltaR2XrIdeFullSet);

    [R1XphiIdeRMSE,R1XphiIdeRMSE_mean] = XphiRMSE(poseNum-1,mcNum,DeltaR1XphiIdeFullSet);
    [R2XphiIdeRMSE,R2XphiIdeRMSE_mean] = XphiRMSE(poseNum,mcNum,DeltaR2XphiIdeFullSet);

    [XfIdeRMSE,XfIdeRMSE_mean] = XFRMSE(feaNum,mcNum,DeltaXfIdeFullSet);
end


% FEJ EKF
[R1XrFejRMSE,R1XrFejRMSE_mean] = XrRMSE(poseNum-1,mcNum,DeltaR1XrFejFullSet);
[R2XrFejRMSE,R2XrFejRMSE_mean] = XrRMSE(poseNum,mcNum,DeltaR2XrFejFullSet);

[R1XphiFejRMSE,R1XphiFejRMSE_mean] = XphiRMSE(poseNum-1,mcNum,DeltaR1XphiFejFullSet);
[R2XphiFejRMSE,R2XphiFejRMSE_mean] = XphiRMSE(poseNum,mcNum,DeltaR2XphiFejFullSet);

[XfFejRMSE,XfFejRMSE_mean] = XFRMSE(feaNum,mcNum,DeltaXfFejFullSet);

poseNum_1round = size(DeltaR1XrFullSet,1)/cy/2;

% Plot RMSE
% R1 postion
figure((ec-1)*6+1)
hold on
% standard EKF
R1XrRMSEP = plot(R1XrRMSE(:,1)',R1XrRMSE(:,2)','-b.','DisplayName','Standard EKF');
% ideal EKF
if ec ~= 4
    R1XrIdeRMSEP = plot(R1XrIdeRMSE(:,1)',R1XrIdeRMSE(:,2)','-k','DisplayName','Ideal EKF');
end
% FEJ EKF
R1XrFejRMSEP = plot(R1XrFejRMSE(:,1)',R1XrFejRMSE(:,2)','--r','DisplayName','FEJ-EKF');

if ec ~= 4
    xline(poseNum_1round,'--g',sprintf('step = %d', poseNum_1round))
end

xlabel('Steps')
ylabel('R1 Position RMSE (m)')
xlim([0,xplot_ub])
if ec ~= 4
    legend([R1XrRMSEP,R1XrIdeRMSEP,R1XrFejRMSEP])
elseif ec == 4
    legend([R1XrRMSEP,R1XrFejRMSEP])
end
set(gcf, 'Color', 'w');  % 将整个图背景设置为白色
set(gca, 'Box', 'on', 'LineWidth', 1);  % 使边框显示，并增加边框宽度

hold off

%% R2 postion
figure((ec-1)*6+2)
hold on
R2XrRMSEP = plot(R2XrRMSE(:,1)',R2XrRMSE(:,2)','-b.','DisplayName','Standard EKF');

if ec ~= 4
    R2XrIdeRMSEP = plot(R2XrIdeRMSE(:,1)',R2XrIdeRMSE(:,2)','-k','DisplayName','Ideal EKF');
end

R2XrFejRMSEP = plot(R2XrFejRMSE(:,1)',R2XrFejRMSE(:,2)','--r','DisplayName','FEJ-EKF');

if ec ~= 4
    xline(poseNum_1round,'--g',sprintf('step = %d', poseNum_1round))
end

xlabel('Steps')
ylabel('R2 Position RMSE (m)')
xlim([0,xplot_ub])
if ec ~= 4
    legend([R2XrRMSEP,R2XrIdeRMSEP,R2XrFejRMSEP])
else
    legend([R2XrRMSEP,R2XrFejRMSEP])
end
set(gcf, 'Color', 'w');  % 将整个图背景设置为白色
set(gca, 'Box', 'on', 'LineWidth', 1);  % 使边框显示，并增加边框宽度

hold off

%% R1 orientation
figure((ec-1)*6+3)
hold on
% standard EKF
R1XphiRMSEP = plot(R1XphiRMSE(:,1)',R1XphiRMSE(:,2)','-b.','DisplayName','Standard EKF');
% ideal EKF
if ec ~= 4
    R1XphiIdeRMSEP = plot(R1XphiIdeRMSE(:,1)',R1XphiIdeRMSE(:,2)','-k','DisplayName','Ideal EKF');
end
% FEJ EKF
R1XphiFejRMSEP = plot(R1XphiFejRMSE(:,1)',R1XphiFejRMSE(:,2)','--r','DisplayName','FEJ-EKF');

if ec ~= 4
    xline(poseNum_1round,'--g',sprintf('step = %d', poseNum_1round))
end

xlabel('Steps')
ylabel('R1 Heading RMSE (rad)')
xlim([0,xplot_ub])
if ec ~= 4
    legend([R1XphiRMSEP,R1XphiIdeRMSEP,R1XphiFejRMSEP])
else
    legend([R1XphiRMSEP,R1XphiFejRMSEP])
end
set(gcf, 'Color', 'w');  % 将整个图背景设置为白色
set(gca, 'Box', 'on', 'LineWidth', 1);  % 使边框显示，并增加边框宽度

hold off

%% R2 orientation
figure((ec-1)*6+4)
hold on
R2XphiRMSEP = plot(R2XphiRMSE(:,1)',R2XphiRMSE(:,2)','-b.','DisplayName','Standard EKF');
if ec ~= 4
    R2XphiIdeRMSEP = plot(R2XphiIdeRMSE(:,1)',R2XphiIdeRMSE(:,2)','-k','DisplayName','Ideal EKF');
end
R2XphiFejRMSEP = plot(R2XphiFejRMSE(:,1)',R2XphiFejRMSE(:,2)','--r','DisplayName','FEJ-EKF');

if ec ~= 4
    xline(poseNum_1round,'--g',sprintf('step = %d', poseNum_1round))
end

xlabel('Steps')
ylabel('R2 Heading RMSE (rad)')
xlim([0,xplot_ub])
if ec ~= 4
    legend([R2XphiRMSEP,R2XphiIdeRMSEP,R2XphiFejRMSEP])
else
    legend([R2XphiRMSEP,R2XphiFejRMSEP])
end
set(gcf, 'Color', 'w');  % 将整个图背景设置为白色
set(gca, 'Box', 'on', 'LineWidth', 1);  % 使边框显示，并增加边框宽度

hold off



%% Plot Average Normalized (state) Estimation Error Squared (ANEES) for the last step
%% standard EKF
R1NEES_k = [(1:poseNum)',zeros(poseNum,mcNum)];
R2NEES_k = [(0:poseNum)',zeros(poseNum+1,mcNum)];

for k = 0:poseNum

    for mn = 1:mcNum
        R2xri_e = DeltaR2XpFullSet(k*3+(1:3),1+mn);
        R2C_xrie = R2PFullSet(k*3+(1:3),(mn-1)*3+(1:3));
        R2NEES_k(k+1,1+mn) = R2xri_e'/R2C_xrie*R2xri_e;

        if k ~= 0
            R1xri_e = DeltaR1XpFullSet((k-1)*3+(1:3),1+mn);
            R1C_xrie = R1PFullSet((k-1)*3+(1:3),(mn-1)*3+(1:3));
            R1NEES_k(k,1+mn) = R1xri_e'/R1C_xrie*R1xri_e;
        end
    end
end

R1XpNEES = [R1NEES_k(:,1),mean(R1NEES_k(:,2:end),2)/3];
R2XpNEES = [R2NEES_k(:,1),mean(R2NEES_k(:,2:end),2)/3];

R1XpNEES_mean = mean(R1XpNEES(:,2));

R2XpNEES_mean = mean(R2XpNEES(:,2));

XfNEES_k = [DeltaXfFullSet(1:2:(end-1),1),zeros(feaNum,mcNum)];

for j = 1:feaNum
    for mn = 1:mcNum
        xfi_e = DeltaXfFullSet((j-1)*2+(1:2),1+mn);
        C_xfie = PfFullSet((j-1)*2+(1:2),(mn-1)*feaNum*2+(j-1)*2+(1:2));
        XfNEES_k(j,1+mn) = xfi_e'/C_xfie*xfi_e;
    end
end

XfNEES = [XfNEES_k(:,1),mean(XfNEES_k(:,2:end),2)/2];
XfNEES_mean = mean(XfNEES(:,2));



%% ideal EKF
if ec ~= 4
    R1IdeNEES_k = [(1:poseNum)',zeros(poseNum,mcNum)];
    R2IdeNEES_k = [(0:poseNum)',zeros(poseNum+1,mcNum)];

    for k = 0:poseNum

        for mn = 1:mcNum
            R2Idexri_e = DeltaR2XpIdeFullSet(k*3+(1:3),1+mn);
            R2IdeC_xrie = R2PIdeFullSet(k*3+(1:3),(mn-1)*3+(1:3));
            R2IdeNEES_k(k+1,1+mn) = R2Idexri_e'/R2IdeC_xrie*R2Idexri_e;

            if k ~= 0
                R1Idexri_e = DeltaR1XpIdeFullSet((k-1)*3+(1:3),1+mn);
                R1IdeC_xrie = R1PIdeFullSet((k-1)*3+(1:3),(mn-1)*3+(1:3));
                R1IdeNEES_k(k,1+mn) = R1Idexri_e'/R1IdeC_xrie*R1Idexri_e;
            end
        end
    end

    R1XpIdeNEES = [R1IdeNEES_k(:,1),mean(R1IdeNEES_k(:,2:end),2)/3];
    R2XpIdeNEES = [R2IdeNEES_k(:,1),mean(R2IdeNEES_k(:,2:end),2)/3];

    R1XpIdeNEES_mean = mean(R1XpIdeNEES(:,2));

    R2XpIdeNEES_mean = mean(R2XpIdeNEES(:,2));

    XfIdeNEES_k = [DeltaXfIdeFullSet(1:2:(end-1),1),zeros(feaNum,mcNum)];

    for j = 1:feaNum
        for mn = 1:mcNum
            Idexfi_e = DeltaXfIdeFullSet((j-1)*2+(1:2),1+mn);
            IdeC_xfie = PfIdeFullSet((j-1)*2+(1:2),(mn-1)*feaNum*2+(j-1)*2+(1:2));
            XfIdeNEES_k(j,1+mn) = Idexfi_e'/IdeC_xfie*Idexfi_e;
        end
    end

    XfIdeNEES = [XfIdeNEES_k(:,1),mean(XfIdeNEES_k(:,2:end),2)/2];
    XfIdeNEES_mean = mean(XfIdeNEES(:,2));
end

%% FEJ EKF
R1FejNEES_k = [(1:poseNum)',zeros(poseNum,mcNum)];
R2FejNEES_k = [(0:poseNum)',zeros(poseNum+1,mcNum)];

for k = 0:poseNum

    for mn = 1:mcNum
        R2Fejxri_e = DeltaR2XpFejFullSet(k*3+(1:3),1+mn);
        R2FejC_xrie = R2PFejFullSet(k*3+(1:3),(mn-1)*3+(1:3));
        R2FejNEES_k(k+1,1+mn) = R2Fejxri_e'/R2FejC_xrie*R2Fejxri_e;

        if k ~= 0
            R1Fejxri_e = DeltaR1XpFejFullSet((k-1)*3+(1:3),1+mn);
            R1FejC_xrie = R1PFejFullSet((k-1)*3+(1:3),(mn-1)*3+(1:3));
            R1FejNEES_k(k,1+mn) = R1Fejxri_e'/R1FejC_xrie*R1Fejxri_e;
        end
    end
end

R1XpFejNEES = [R1FejNEES_k(:,1),mean(R1FejNEES_k(:,2:end),2)/3];
R2XpFejNEES = [R2FejNEES_k(:,1),mean(R2FejNEES_k(:,2:end),2)/3];

R1XpFejNEES_mean = mean(R1XpFejNEES(:,2));
R2XpFejNEES_mean = mean(R2XpFejNEES(:,2));

XfFejNEES_k = [DeltaXfFejFullSet(1:2:(end-1),1),zeros(feaNum,mcNum)];

for j = 1:feaNum
    for mn = 1:mcNum
        Fejxfi_e = DeltaXfFejFullSet((j-1)*2+(1:2),1+mn);
        FejC_xfie = PfFejFullSet((j-1)*2+(1:2),(mn-1)*feaNum*2+(j-1)*2+(1:2));
        XfFejNEES_k(j,1+mn) = Fejxfi_e'/FejC_xfie*Fejxfi_e;
    end
end

XfFejNEES = [XfFejNEES_k(:,1),mean(XfFejNEES_k(:,2:end),2)/2];
XfFejNEES_mean = mean(XfFejNEES(:,2));



%% R1 pose
figure((ec-1)*6+5)
hold on
% standard EKF
R1XpNEESP = plot(R1XpNEES(:,1)',R1XpNEES(:,2)','-b.','DisplayName','Standard EKF');
% ideal EKF
if ec ~= 4
    R1XpIdeNEESP = plot(R1XpIdeNEES(:,1)',R1XpIdeNEES(:,2)','-k','DisplayName','Ideal EKF');
end
% FEJ EKF
R1XpFejNEESP = plot(R1XpFejNEES(:,1)',R1XpFejNEES(:,2)','--r','DisplayName','FEJ-EKF');

if ec ~= 4
    xline(poseNum_1round,'--g',sprintf('step = %d', poseNum_1round))
end

xlabel('Steps')
ylabel('R1 Pose NEES')
xlim([0,xplot_ub])
if ec ~= 4
    legend([R1XpNEESP,R1XpIdeNEESP,R1XpFejNEESP])
else
    legend([R1XpNEESP,R1XpFejNEESP])
end
set(gcf, 'Color', 'w');  % 将整个图背景设置为白色
set(gca, 'Box', 'on', 'LineWidth', 1);  % 使边框显示，并增加边框宽度

hold off

%% R2 pose
figure((ec-1)*6+6)
hold on
R2XpNEESP = plot(R2XpNEES(:,1)',R2XpNEES(:,2)','-b.','DisplayName','Standard EKF');
if ec ~= 4
    R2XpIdeNEESP = plot(R2XpIdeNEES(:,1)',R2XpIdeNEES(:,2)','-k','DisplayName','Ideal EKF');
end
R2XpFejNEESP = plot(R2XpFejNEES(:,1)',R2XpFejNEES(:,2)','--r','DisplayName','FEJ-EKF');

if ec ~= 4
    xline(poseNum_1round,'--g',sprintf('step = %d', poseNum_1round))
end

xlabel('Steps')
ylabel('R2 Pose NEES')
xlim([0,xplot_ub])
if ec ~= 4
    legend([R2XpNEESP,R2XpIdeNEESP,R2XpFejNEESP])
else
    legend([R2XpNEESP,R2XpFejNEESP])
end
set(gcf, 'Color', 'w');  % 将整个图背景设置为白色
set(gca, 'Box', 'on', 'LineWidth', 1);  % 使边框显示，并增加边框宽度

hold off

%% Display noise levels
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Noise levels %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
fprintf(['R1sigma_0r = %f, R1sigma_0phi = %f, R2sigma_0r = %f, R2sigma_0phi = %f\n' ...
    'R1sigma_uv = %f, R1sigma_uw = %f, R2sigma_uv = %f, R2sigma_uw = %f\n' ...
    'R1sigma_zv = %f, R2sigma_zv = %f\n'], ...
    R1sigma_0r,R1sigma_0phi,R2sigma_0r,R2sigma_0phi,R1sigma_uv,R1sigma_uw,R2sigma_uv,R2sigma_uw,R1sigma_zv,R2sigma_zv)
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')

%% Display the mean
if ec == 1

    disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 20 features %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
    IdeEKFColumn = [R1XpIdeNEES_mean;R1XrIdeRMSE_mean;R1XphiIdeRMSE_mean;R2XpIdeNEES_mean;R2XrIdeRMSE_mean;R2XphiIdeRMSE_mean;XfIdeNEES_mean;XfIdeRMSE_mean];
    StdEKFColumn = [R1XpNEES_mean;R1XrRMSE_mean;R1XphiRMSE_mean;R2XpNEES_mean;R2XrRMSE_mean;R2XphiRMSE_mean;XfNEES_mean;XfStdRMSE_mean];
    FejEKFColumn = [R1XpFejNEES_mean;R1XrFejRMSE_mean;R1XphiFejRMSE_mean;R2XpFejNEES_mean;R2XrFejRMSE_mean;R2XphiFejRMSE_mean;XfFejNEES_mean;XfFejRMSE_mean];

    IdeEKFColumn = arrayfun(@(x) sprintf('%.4f', x), IdeEKFColumn, 'UniformOutput', false);
    StdEKFColumn = arrayfun(@(x) sprintf('%.4f', x), StdEKFColumn, 'UniformOutput', false);
    FejEKFColumn = arrayfun(@(x) sprintf('%.4f', x), FejEKFColumn, 'UniformOutput', false);

    % 创建表格并插入动态列名称
    T = table(IdeEKFColumn, StdEKFColumn,FejEKFColumn,'VariableNames',{'Ideal EKF','Std EKF','FEJ EKF'});

    % 设置行名称
    T.Properties.RowNames = {'R1 Pose NEES','R1 Position Err.RMS (m)','R1 Heading Err.RMS (rad)','R2 Pose NEES','R2 Position Err.RMS (m)','R2 Heading Err.RMS (rad)','Feature Position NEES','Feature Position Err.RMS (m)'};

    % 显示表格
    disp(T);
    disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')

elseif ec == 2

    disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 60 features %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
    IdeEKFColumn = [R1XpIdeNEES_mean;R1XrIdeRMSE_mean;R1XphiIdeRMSE_mean;R2XpIdeNEES_mean;R2XrIdeRMSE_mean;R2XphiIdeRMSE_mean;XfIdeNEES_mean;XfIdeRMSE_mean];
    StdEKFColumn = [R1XpNEES_mean;R1XrRMSE_mean;R1XphiRMSE_mean;R2XpNEES_mean;R2XrRMSE_mean;R2XphiRMSE_mean;XfNEES_mean;XfStdRMSE_mean];
    FejEKFColumn = [R1XpFejNEES_mean;R1XrFejRMSE_mean;R1XphiFejRMSE_mean;R2XpFejNEES_mean;R2XrFejRMSE_mean;R2XphiFejRMSE_mean;XfFejNEES_mean;XfFejRMSE_mean];

    IdeEKFColumn = arrayfun(@(x) sprintf('%.4f', x), IdeEKFColumn, 'UniformOutput', false);
    StdEKFColumn = arrayfun(@(x) sprintf('%.4f', x), StdEKFColumn, 'UniformOutput', false);
    FejEKFColumn = arrayfun(@(x) sprintf('%.4f', x), FejEKFColumn, 'UniformOutput', false);

    % 创建表格并插入动态列名称
    T = table(IdeEKFColumn, StdEKFColumn,FejEKFColumn,'VariableNames',{'Ideal EKF','Std EKF','FEJ EKF'});

    % 设置行名称
    T.Properties.RowNames = {'R1 Pose NEES','R1 Position Err.RMS (m)','R1 Heading Err.RMS (rad)','R2 Pose NEES','R2 Position Err.RMS (m)','R2 Heading Err.RMS (rad)','Feature Position NEES','Feature Position Err.RMS (m)'};

    % 显示表格
    disp(T);
    disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')

elseif ec == 3

    disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 100 features %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
    IdeEKFColumn = [R1XpIdeNEES_mean;R1XrIdeRMSE_mean;R1XphiIdeRMSE_mean;R2XpIdeNEES_mean;R2XrIdeRMSE_mean;R2XphiIdeRMSE_mean;XfIdeNEES_mean;XfIdeRMSE_mean];
    StdEKFColumn = [R1XpNEES_mean;R1XrRMSE_mean;R1XphiRMSE_mean;R2XpNEES_mean;R2XrRMSE_mean;R2XphiRMSE_mean;XfNEES_mean;XfStdRMSE_mean];
    FejEKFColumn = [R1XpFejNEES_mean;R1XrFejRMSE_mean;R1XphiFejRMSE_mean;R2XpFejNEES_mean;R2XrFejRMSE_mean;R2XphiFejRMSE_mean;XfFejNEES_mean;XfFejRMSE_mean];

    IdeEKFColumn = arrayfun(@(x) sprintf('%.4f', x), IdeEKFColumn, 'UniformOutput', false);
    StdEKFColumn = arrayfun(@(x) sprintf('%.4f', x), StdEKFColumn, 'UniformOutput', false);
    FejEKFColumn = arrayfun(@(x) sprintf('%.4f', x), FejEKFColumn, 'UniformOutput', false);

    % 创建表格并插入动态列名称
    T = table(IdeEKFColumn, StdEKFColumn,FejEKFColumn,'VariableNames',{'Ideal EKF','Std EKF','FEJ EKF'});

    % 设置行名称
    T.Properties.RowNames = {'R1 Pose NEES','R1 Position Err.RMS (m)','R1 Heading Err.RMS (rad)','R2 Pose NEES','R2 Position Err.RMS (m)','R2 Heading Err.RMS (rad)','Feature Position NEES','Feature Position Err.RMS (m)'};

    % 显示表格
    disp(T);
    disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')

elseif ec == 4

    disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% VictoriaParkDataset %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
    % IdeEKFColumn = [R1XpIdeNEES_mean;R1XrIdeRMSE_mean;R1XphiIdeRMSE_mean;R2XpIdeNEES_mean;R2XrIdeRMSE_mean;R2XphiIdeRMSE_mean;XfIdeNEES_mean;XfIdeRMSE_mean];
    StdEKFColumn = [R1XpNEES_mean;R1XrRMSE_mean;R1XphiRMSE_mean;R2XpNEES_mean;R2XrRMSE_mean;R2XphiRMSE_mean;XfNEES_mean;XfStdRMSE_mean];
    FejEKFColumn = [R1XpFejNEES_mean;R1XrFejRMSE_mean;R1XphiFejRMSE_mean;R2XpFejNEES_mean;R2XrFejRMSE_mean;R2XphiFejRMSE_mean;XfFejNEES_mean;XfFejRMSE_mean];

    % IdeEKFColumn = arrayfun(@(x) sprintf('%.4f', x), IdeEKFColumn, 'UniformOutput', false);
    StdEKFColumn = arrayfun(@(x) sprintf('%.4f', x), StdEKFColumn, 'UniformOutput', false);
    FejEKFColumn = arrayfun(@(x) sprintf('%.4f', x), FejEKFColumn, 'UniformOutput', false);

    % 创建表格并插入动态列名称
    % T = table(IdeEKFColumn, StdEKFColumn,FejEKFColumn,'VariableNames',{'Ideal EKF','Std EKF','FEJ EKF'});
    T = table(StdEKFColumn,FejEKFColumn,'VariableNames',{'Std EKF','FEJ EKF'});


    % 设置行名称
    T.Properties.RowNames = {'R1 Pose NEES','R1 Position Err.RMS (m)','R1 Heading Err.RMS (rad)','R2 Pose NEES','R2 Position Err.RMS (m)','R2 Heading Err.RMS (rad)','Feature Position NEES','Feature Position Err.RMS (m)'};

    % 显示表格
    disp(T);
    disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')

end

%% 3 sigma bound plot

% load('MTE_results_StdEKF_20fea.mat','poseNum','feaNum', ...
%     'DeltaR1XrFullSet','DeltaR2XrFullSet','DeltaR1XphiFullSet','DeltaR2XphiFullSet', ...
%     'DeltaR2XpFullSet','R2PFullSet','DeltaR1XpFullSet','R1PFullSet', ...
%     'DeltaXfFullSet','PfFullSet')
% load('MTE_results_IdeEKF_20fea.mat', ...
%     'DeltaR1XrIdeFullSet','DeltaR2XrIdeFullSet','DeltaR1XphiIdeFullSet','DeltaR2XphiIdeFullSet', ...
%     'DeltaR2XpIdeFullSet','R2PIdeFullSet','DeltaR1XpIdeFullSet','R1PIdeFullSet', ...
%     'DeltaXfIdeFullSet','PfIdeFullSet')
% load('MTE_results_FejEKF_20fea.mat', ...
%     'DeltaR1XrFejFullSet','DeltaR2XrFejFullSet','DeltaR1XphiFejFullSet','DeltaR2XphiFejFullSet', ...
%     'DeltaR2XpFejFullSet','R2PFejFullSet','DeltaR1XpFejFullSet','R1PFejFullSet', ...
%     'DeltaXfFejFullSet','PfFejFullSet')

if ec == 1
    %% R1
    % position
    %% x
    figure(25)
    hold on
    grid on
    set(gcf, 'Color', 'w');  % 将整个图背景设置为白色
    set(gca, 'Box', 'on', 'LineWidth', 1);  % 使边框显示，并增加边框宽度
    xlabel('step')
    ylabel('m')
    xlim([0,xplot_ub])
    title('R1 position x')



    if mcNum ~= 1
        % x error mean
        % Std EKF
        R1AvgErr_x = mean(abs(DeltaR1XrFullSet(1:2:(end-1),2:end)),2);
        % FEJ EKF
        R1FEJAvgErr_x = mean(abs(DeltaR1XrFejFullSet(1:2:(end-1),2:end)),2);
    else
        R1AvgErr_x = DeltaR1XrFullSet(1:2:(end-1),2:end);
        R1FEJAvgErr_x = DeltaR1XrFejFullSet(1:2:(end-1),2:end);
    end

    R1AvgErrxP = plot((1:poseNum)',R1AvgErr_x,'b','DisplayName','Std error');
    R1FEJAvgErrxP = plot((1:poseNum)',R1FEJAvgErr_x,'r','DisplayName','FEJ error');

    % x standard deviation mean
    % Std EKF
    R1PFullSet_x = R1PFullSet(1:3:(end-2),1:3:(end-2));
    R1AvgSD_x = mean(sqrt(R1PFullSet_x),2); % standard deviation
    % FEJ EKF
    R1FEJPFullSet_x = R1PIdeFullSet(1:3:(end-2),1:3:(end-2));
    R1FEJAvgSD_x = mean(sqrt(R1FEJPFullSet_x),2); % standard deviation

    if mcNum ~= 1
        R1AvgPx_P = plot((1:poseNum)',3*R1AvgSD_x','c--','DisplayName','Std 3{\sigma}-bound');
        R1FEJAvgPx_P = plot((1:poseNum)',3*R1FEJAvgSD_x','m--','DisplayName','FEJ 3{\sigma}-bound');
    else
        R1AvgPx_P = plot((1:poseNum)',3*R1AvgSD_x','c--','DisplayName','Std 3{\sigma}-bound');
        plot((1:poseNum)',-3*R1AvgSD_x','c--')
        R1FEJAvgPx_P = plot((1:poseNum)',3*R1FEJAvgSD_x','m--','DisplayName','FEJ 3{\sigma}-bound');
        plot((1:poseNum)',-3*R1FEJAvgSD_x','m--')
    end

    legend([R1AvgErrxP,R1FEJAvgErrxP,R1AvgPx_P,R1FEJAvgPx_P])
    hold off

    %% y
    figure(26)
    hold on
    grid on
    set(gcf, 'Color', 'w');  % 将整个图背景设置为白色
    set(gca, 'Box', 'on', 'LineWidth', 1);  % 使边框显示，并增加边框宽度
    xlabel('step')
    ylabel('m')
    xlim([0,xplot_ub])
    title('R1 position y')

    if mcNum ~= 1
        % y error mean
        % Std EKF
        R1AvgErr_y = mean(abs(DeltaR1XrFullSet(2:2:(end),2:end)),2);
        % FEJ EKF
        R1FEJAvgErr_y = mean(abs(DeltaR1XrFejFullSet(2:2:(end),2:end)),2);
    else
        R1AvgErr_y = DeltaR1XrFullSet(2:2:(end),2:end);
        R1FEJAvgErr_y = DeltaR1XrFejFullSet(2:2:(end),2:end);
    end
    R1AvgErryP = plot((1:poseNum)',R1AvgErr_y,'b','DisplayName','Std error');
    R1FEJAvgErryP = plot((1:poseNum)',R1FEJAvgErr_y,'r','DisplayName','FEJ error');


    % y standard deviation mean
    % Std EKF
    R1PFullSet_y = R1PFullSet(2:3:(end-1),(2:3:(end-1)));
    R1AvgSD_y = mean(sqrt(R1PFullSet_y),2); % standard deviation

    % FEJ EKF
    R1FEJPFullSet_y = R1PFejFullSet(2:3:(end-1),(2:3:(end-1)));
    R1FEJAvgSD_y = mean(sqrt(R1FEJPFullSet_y),2); % standard deviation

    if mcNum ~= 1
        R1AvgPy_P = plot((1:poseNum)',3*R1AvgSD_y','c--','DisplayName','Std 3{\sigma}-bound');
        R1FEJAvgPy_P = plot((1:poseNum)',3*R1FEJAvgSD_y','m--','DisplayName','FEJ 3{\sigma}-bound');
    else
        R1AvgPy_P = plot((1:poseNum)',3*R1AvgSD_y','c--','DisplayName','Std 3{\sigma}-bound');
        plot((1:poseNum)',-3*R1AvgSD_y','c--');
        R1FEJAvgPy_P = plot((1:poseNum)',3*R1FEJAvgSD_y','m--','DisplayName','FEJ 3{\sigma}-bound');
        plot((1:poseNum)',-3*R1FEJAvgSD_y','m--');
    end

    legend([R1AvgErryP,R1FEJAvgErryP,R1AvgPy_P,R1FEJAvgPy_P])
    hold off

    %% heading (phi)
    figure(27)
    hold on
    grid on
    set(gcf, 'Color', 'w');  % 将整个图背景设置为白色
    set(gca, 'Box', 'on', 'LineWidth', 1);  % 使边框显示，并增加边框宽度
    xlabel('step')
    ylabel('rad')
    xlim([0,xplot_ub])
    title('R1 heading \phi')

    if mcNum ~= 1
        % phi error mean
        % Std EKF
        R1AvgErr_phi = mean(abs(DeltaR1XphiFullSet(:,2:end)),2);
        % FEJ EKF
        R1FEJAvgErr_phi = mean(abs(DeltaR1XphiFejFullSet(:,2:end)),2);
    else
        R1AvgErr_phi = DeltaR1XphiFullSet(:,2:end);
        R1FEJAvgErr_phi = DeltaR1XphiFejFullSet(:,2:end);
    end
    R1AvgErrphiP = plot((1:poseNum)',R1AvgErr_phi,'b','DisplayName','Std error');
    R1FEJAvgErrphiP = plot((1:poseNum)',R1FEJAvgErr_phi,'r','DisplayName','FEJ error');

    % phi standard deviation mean
    % Std EKF
    R1PFullSet_phi = R1PFullSet(3:3:(end),3:3:(end));
    R1AvgSD_phi = mean(sqrt(R1PFullSet_phi),2); % standard deviation
    % FEJ EKF
    R1FEJPFullSet_phi = R1PFejFullSet(3:3:(end),3:3:(end));
    R1FEJAvgSD_phi = mean(sqrt(R1FEJPFullSet_phi),2); % standard deviation

    if mcNum ~= 1
        R1AvgPphi_P = plot((1:poseNum)',3*R1AvgSD_phi,'c--','DisplayName','Std 3{\sigma}-bound');
        R1FEJAvgPphi_P = plot((1:poseNum)',3*R1FEJAvgSD_phi,'m--','DisplayName','FEJ 3{\sigma}-bound');
    else
        R1AvgPphi_P = plot((1:poseNum)',3*R1AvgSD_phi,'c--','DisplayName','Std 3{\sigma}-bound');
        plot((1:poseNum)',-3*R1AvgSD_phi,'c--');
        R1FEJAvgPphi_P = plot((1:poseNum)',3*R1FEJAvgSD_phi,'m--','DisplayName','FEJ 3{\sigma}-bound');
        plot((1:poseNum)',-3*R1FEJAvgSD_phi,'m--');
    end
    legend([R1AvgErrphiP,R1FEJAvgErrphiP,R1AvgPphi_P,R1FEJAvgPphi_P])
    hold off

    %% R2
    % position
    %% x
    figure(28)
    hold on
    grid on
    set(gcf, 'Color', 'w');  % 将整个图背景设置为白色
    set(gca, 'Box', 'on', 'LineWidth', 1);  % 使边框显示，并增加边框宽度
    xlabel('step')
    ylabel('m')
    xlim([0,xplot_ub])
    title('R2 position x')

    if mcNum ~= 1
        % x error mean
        % Std EKF
        R2AvgErr_x = mean(abs(DeltaR2XrFullSet(1:2:(end-1),2:end)),2);
        % FEJ EKF
        R2FEJAvgErr_x = mean(abs(DeltaR2XrFejFullSet(1:2:(end-1),2:end)),2);
    else
        R2AvgErr_x = DeltaR2XrFullSet(1:2:(end-1),2:end);
        R2FEJAvgErr_x = DeltaR2XrFejFullSet(1:2:(end-1),2:end);
    end
    R2AvgErrxP = plot((0:poseNum)',R2AvgErr_x,'b','DisplayName','Std error');
    R2FEJAvgErrxP = plot((0:poseNum)',R2FEJAvgErr_x,'r','DisplayName','FEJ error');

    % x standard deviation mean
    % Std EKF
    R2PFullSet_x = R2PFullSet(1:3:(end-2),1:3:(end-2));
    R2AvgSD_x = mean(sqrt(R2PFullSet_x),2); % standard deviation
    % FEJ EKF
    R2FEJPFullSet_x = R2PIdeFullSet(1:3:(end-2),1:3:(end-2));
    R2FEJAvgSD_x = mean(sqrt(R2FEJPFullSet_x),2); % standard deviation

    if mcNum ~= 1
        R2AvgPx_P = plot((0:poseNum)',3*R2AvgSD_x','c--','DisplayName','Std 3{\sigma}-bound');
        R2FEJAvgPx_P = plot((0:poseNum)',3*R2FEJAvgSD_x','m--','DisplayName','FEJ 3{\sigma}-bound');
    else
        R2AvgPx_P = plot((0:poseNum)',3*R2AvgSD_x','c--','DisplayName','Std 3{\sigma}-bound');
        plot((0:poseNum)',-3*R2AvgSD_x','c--');
        R2FEJAvgPx_P = plot((0:poseNum)',3*R2FEJAvgSD_x','m--','DisplayName','FEJ 3{\sigma}-bound');
        plot((0:poseNum)',-3*R2FEJAvgSD_x','m--');
    end
    legend([R2AvgErrxP,R2FEJAvgErrxP,R2AvgPx_P,R2FEJAvgPx_P])
    hold off

    %% y
    figure(29)
    hold on
    grid on
    set(gcf, 'Color', 'w');  % 将整个图背景设置为白色
    set(gca, 'Box', 'on', 'LineWidth', 1);  % 使边框显示，并增加边框宽度
    xlabel('step')
    ylabel('m')
    xlim([0,xplot_ub])
    title('R2 position y')

    if mcNum ~= 1
        % y error mean
        % Std EKF
        R2AvgErr_y = mean(abs(DeltaR2XrFullSet(2:2:(end),2:end)),2);
        % FEJ EKF
        R2FEJAvgErr_y = mean(abs(DeltaR2XrFejFullSet(2:2:(end),2:end)),2);
    else
        R2AvgErr_y = DeltaR2XrFullSet(2:2:(end),2:end);
        R2FEJAvgErr_y = DeltaR2XrFejFullSet(2:2:(end),2:end);
    end

    R2AvgErryP = plot((0:poseNum)',R2AvgErr_y,'b','DisplayName','Std error');
    R2FEJAvgErryP = plot((0:poseNum)',R2FEJAvgErr_y,'r','DisplayName','FEJ error');

    % y standard deviation mean
    % Std EKF
    R2PFullSet_y = R2PFullSet(2:3:(end-1),(2:3:(end-1)));
    R2AvgSD_y = mean(sqrt(R2PFullSet_y),2); % standard deviation
    % FEJ EKF
    R2FEJPFullSet_y = R2PFejFullSet(2:3:(end-1),(2:3:(end-1)));
    R2FEJAvgSD_y = mean(sqrt(R2FEJPFullSet_y),2); % standard deviation

    if mcNum ~= 1
        R2AvgPy_P = plot((0:poseNum)',3*R2AvgSD_y','c--','DisplayName','Std 3{\sigma}-bound');
        R2FEJAvgPy_P = plot((0:poseNum)',3*R2FEJAvgSD_y','m--','DisplayName','FEJ 3{\sigma}-bound');
    else
        R2AvgPy_P = plot((0:poseNum)',3*R2AvgSD_y','c--','DisplayName','Std 3{\sigma}-bound');
        plot((0:poseNum)',-3*R2AvgSD_y','c--');
        R2FEJAvgPy_P = plot((0:poseNum)',3*R2FEJAvgSD_y','m--','DisplayName','FEJ 3{\sigma}-bound');
        plot((0:poseNum)',-3*R2FEJAvgSD_y','m--');
    end

    legend([R2AvgErryP,R2FEJAvgErryP,R2AvgPy_P,R2FEJAvgPy_P])
    hold off

    %% heading (phi)
    figure(30)
    hold on
    grid on
    set(gcf, 'Color', 'w');  % 将整个图背景设置为白色
    set(gca, 'Box', 'on', 'LineWidth', 1);  % 使边框显示，并增加边框宽度
    xlabel('step')
    ylabel('rad')
    xlim([0,xplot_ub])
    title('R2 heading \phi')

    if mcNum ~= 1
        % phi error mean
        % Std EKF
        R2AvgErr_phi = mean(abs(DeltaR2XphiFullSet(:,2:end)),2);
        % FEJ EKF
        R2FEJAvgErr_phi = mean(abs(DeltaR2XphiFejFullSet(:,2:end)),2);
    else
        R2AvgErr_phi = DeltaR2XphiFullSet(:,2:end);
        R2FEJAvgErr_phi = DeltaR2XphiFejFullSet(:,2:end);
    end
    R2AvgErrphiP = plot((0:poseNum)',R2AvgErr_phi,'b','DisplayName','Std error');
    R2FEJAvgErrphiP = plot((0:poseNum)',R2FEJAvgErr_phi,'r','DisplayName','FEJ error');

    % phi standard deviation mean
    % Std EKF
    R2PFullSet_phi = R2PFullSet(3:3:(end),3:3:(end));
    R2AvgSD_phi = mean(sqrt(R2PFullSet_phi),2); % standard deviation
    % FEJ EKF
    R2FEJPFullSet_phi = R2PFejFullSet(3:3:(end),3:3:(end));
    R2FEJAvgSD_phi = mean(sqrt(R2FEJPFullSet_phi),2); % standard deviation

    if mcNum ~= 1
        R2AvgPphi_P = plot((0:poseNum)',3*R2AvgSD_phi,'c--','DisplayName','Std 3{\sigma}-bound');
        R2FEJAvgPphi_P = plot((0:poseNum)',3*R2FEJAvgSD_phi,'m--','DisplayName','FEJ 3{\sigma}-bound');
    else
        R2AvgPphi_P = plot((0:poseNum)',3*R2AvgSD_phi,'c--','DisplayName','Std 3{\sigma}-bound');
        plot((0:poseNum)',-3*R2AvgSD_phi,'c--');
        R2FEJAvgPphi_P = plot((0:poseNum)',3*R2FEJAvgSD_phi,'m--','DisplayName','FEJ 3{\sigma}-bound');
        plot((0:poseNum)',-3*R2FEJAvgSD_phi,'m--');
    end

    legend([R2AvgErrphiP,R2FEJAvgErrphiP,R2AvgPphi_P,R2FEJAvgPphi_P])
    hold off

    if env == 1
        export_fig(fullfile(figuresFolderPath, 'R1Posi_x_3SigB_20feas_1.jpg'), '-jpg', '-r300', figure(25));
        export_fig(fullfile(figuresFolderPath, 'R1Posi_y_3SigB_20feas_1.jpg'), '-jpg', '-r300', figure(26));
        export_fig(fullfile(figuresFolderPath, 'R1Head_phi_3SigB_20feas_1.jpg'), '-jpg', '-r300', figure(27));

        export_fig(fullfile(figuresFolderPath, 'R2Posi_x_3SigB_20feas_1.jpg'), '-jpg', '-r300', figure(28));
        export_fig(fullfile(figuresFolderPath, 'R2Posi_y_3SigB_20feas_1.jpg'), '-jpg', '-r300', figure(29));
        export_fig(fullfile(figuresFolderPath, 'R2Head_phi_3SigB_20feas_1.jpg'), '-jpg', '-r300', figure(30));
    else
        export_fig(fullfile(figuresFolderPath, 'R1Posi_x_3SigB_20feas_2.jpg'), '-jpg', '-r300', figure(25));
        export_fig(fullfile(figuresFolderPath, 'R1Posi_y_3SigB_20feas_2.jpg'), '-jpg', '-r300', figure(26));
        export_fig(fullfile(figuresFolderPath, 'R1Head_phi_3SigB_20feas_2.jpg'), '-jpg', '-r300', figure(27));

        export_fig(fullfile(figuresFolderPath, 'R2Posi_x_3SigB_20feas_2.jpg'), '-jpg', '-r300', figure(28));
        export_fig(fullfile(figuresFolderPath, 'R2Posi_y_3SigB_20feas_2.jpg'), '-jpg', '-r300', figure(29));
        export_fig(fullfile(figuresFolderPath, 'R2Head_phi_3SigB_20feas_2.jpg'), '-jpg', '-r300', figure(30));
    end
end



%% 使用export_fig代替exportgraphics来导出高质量的图像
if ec == 1
    if env == 1
        export_fig(fullfile(figuresFolderPath, 'R1Postion_RMSE_20feas_1.jpg'), '-jpg', '-r300', figure(1));
        export_fig(fullfile(figuresFolderPath, 'R2Postion_RMSE_20feas_1.jpg'), '-jpg', '-r300', figure(2));
        export_fig(fullfile(figuresFolderPath, 'R1Orientation_RMSE_20feas_1.jpg'), '-jpg', '-r300', figure(3));
        export_fig(fullfile(figuresFolderPath, 'R2Orientation_RMSE_20feas_1.jpg'), '-jpg', '-r300', figure(4));
        export_fig(fullfile(figuresFolderPath, 'R1Pose_ANEES_20feas_1.jpg'), '-jpg', '-r300', figure(5));
        export_fig(fullfile(figuresFolderPath, 'R2Pose_ANEES_20feas_1.jpg'), '-jpg', '-r300', figure(6));
    else
        export_fig(fullfile(figuresFolderPath, 'R1Postion_RMSE_20feas_2.jpg'), '-jpg', '-r300', figure(1));
        export_fig(fullfile(figuresFolderPath, 'R2Postion_RMSE_20feas_2.jpg'), '-jpg', '-r300', figure(2));
        export_fig(fullfile(figuresFolderPath, 'R1Orientation_RMSE_20feas_2.jpg'), '-jpg', '-r300', figure(3));
        export_fig(fullfile(figuresFolderPath, 'R2Orientation_RMSE_20feas_2.jpg'), '-jpg', '-r300', figure(4));
        export_fig(fullfile(figuresFolderPath, 'R1Pose_ANEES_20feas_2.jpg'), '-jpg', '-r300', figure(5));
        export_fig(fullfile(figuresFolderPath, 'R2Pose_ANEES_20feas_2.jpg'), '-jpg', '-r300', figure(6));
    end
elseif ec == 2
    if env == 1
        export_fig(fullfile(figuresFolderPath, 'R1Postion_RMSE_60feas_1.jpg'), '-jpg', '-r300', figure(7));
        export_fig(fullfile(figuresFolderPath, 'R2Postion_RMSE_60feas_1.jpg'), '-jpg', '-r300', figure(8));
        export_fig(fullfile(figuresFolderPath, 'R1Orientation_RMSE_60feas_1.jpg'), '-jpg', '-r300', figure(9));
        export_fig(fullfile(figuresFolderPath, 'R2Orientation_RMSE_60feas_1.jpg'), '-jpg', '-r300', figure(10));
        export_fig(fullfile(figuresFolderPath, 'R1Pose_ANEES_60feas_1.jpg'), '-jpg', '-r300', figure(11));
        export_fig(fullfile(figuresFolderPath, 'R2Pose_ANEES_60feas_1.jpg'), '-jpg', '-r300', figure(12));
    else
        export_fig(fullfile(figuresFolderPath, 'R1Postion_RMSE_60feas_2.jpg'), '-jpg', '-r300', figure(7));
        export_fig(fullfile(figuresFolderPath, 'R2Postion_RMSE_60feas_2.jpg'), '-jpg', '-r300', figure(8));
        export_fig(fullfile(figuresFolderPath, 'R1Orientation_RMSE_60feas_2.jpg'), '-jpg', '-r300', figure(9));
        export_fig(fullfile(figuresFolderPath, 'R2Orientation_RMSE_60feas_2.jpg'), '-jpg', '-r300', figure(10));
        export_fig(fullfile(figuresFolderPath, 'R1Pose_ANEES_60feas_2.jpg'), '-jpg', '-r300', figure(11));
        export_fig(fullfile(figuresFolderPath, 'R2Pose_ANEES_60feas_2.jpg'), '-jpg', '-r300', figure(12));
    end

elseif ec == 3
    if env == 1
        export_fig(fullfile(figuresFolderPath, 'R1Postion_RMSE_100feas_1.jpg'), '-jpg', '-r300', figure(13));
        export_fig(fullfile(figuresFolderPath, 'R2Postion_RMSE_100feas_1.jpg'), '-jpg', '-r300', figure(14));
        export_fig(fullfile(figuresFolderPath, 'R1Orientation_RMSE_100feas_1.jpg'), '-jpg', '-r300', figure(15));
        export_fig(fullfile(figuresFolderPath, 'R2Orientation_RMSE_100feas_1.jpg'), '-jpg', '-r300', figure(16));
        export_fig(fullfile(figuresFolderPath, 'R1Pose_ANEES_100feas_1.jpg'), '-jpg', '-r300', figure(17));
        export_fig(fullfile(figuresFolderPath, 'R2Pose_ANEES_100feas_1.jpg'), '-jpg', '-r300', figure(18));
    else
        export_fig(fullfile(figuresFolderPath, 'R1Postion_RMSE_100feas_2.jpg'), '-jpg', '-r300', figure(13));
        export_fig(fullfile(figuresFolderPath, 'R2Postion_RMSE_100feas_2.jpg'), '-jpg', '-r300', figure(14));
        export_fig(fullfile(figuresFolderPath, 'R1Orientation_RMSE_100feas_2.jpg'), '-jpg', '-r300', figure(15));
        export_fig(fullfile(figuresFolderPath, 'R2Orientation_RMSE_100feas_2.jpg'), '-jpg', '-r300', figure(16));
        export_fig(fullfile(figuresFolderPath, 'R1Pose_ANEES_100feas_2.jpg'), '-jpg', '-r300', figure(17));
        export_fig(fullfile(figuresFolderPath, 'R2Pose_ANEES_100feas_2.jpg'), '-jpg', '-r300', figure(18));
    end

elseif ec == 4
    export_fig(fullfile(figuresFolderPath, 'R1Postion_RMSE_VicP.jpg'), '-jpg', '-r300', figure(19));
    export_fig(fullfile(figuresFolderPath, 'R2Postion_RMSE_VicP.jpg'), '-jpg', '-r300', figure(20));
    export_fig(fullfile(figuresFolderPath, 'R1Orientation_RMSE_VicP.jpg'), '-jpg', '-r300', figure(21));
    export_fig(fullfile(figuresFolderPath, 'R2Orientation_RMSE_VicP.jpg'), '-jpg', '-r300', figure(22));
    export_fig(fullfile(figuresFolderPath, 'R1Pose_ANEES_VicP.jpg'), '-jpg', '-r300', figure(23));
    export_fig(fullfile(figuresFolderPath, 'R2Pose_ANEES_VicP.jpg'), '-jpg', '-r300', figure(24));
end