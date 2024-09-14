clc
close all

%% save output figures
currentFolder = fileparts(mfilename('fullpath'));
subFolder = 'saved_figures';
figuresFolderPath = fullfile(currentFolder, subFolder);
if ~exist(figuresFolderPath, 'dir')
    mkdir(figuresFolderPath);
end

Config;

%% load the Monte Carlo Experiments result
for fc = 1:3 % feature choice
    if fc == 1
        load('MTE_results_StdEKF_20fea.mat','poseNum','feaNum', ...
            'DeltaR1XrFullSet','DeltaR2XrFullSet','DeltaR1XphiFullSet','DeltaR2XphiFullSet', ...
            'DeltaR2XpFullSet','R2PFullSet','DeltaR1XpFullSet','R1PFullSet', ...
            'DeltaXfFullSet','PfFullSet')
        load('MTE_results_IdeEKF_20fea.mat', ...
            'DeltaR1XrIdeFullSet','DeltaR2XrIdeFullSet','DeltaR1XphiIdeFullSet','DeltaR2XphiIdeFullSet', ...
            'DeltaR2XpIdeFullSet','R2PIdeFullSet','DeltaR1XpIdeFullSet','R1PIdeFullSet', ...
            'DeltaXfIdeFullSet','PfIdeFullSet')
        load('MTE_results_FejEKF_20fea.mat', ...
            'DeltaR1XrFejFullSet','DeltaR2XrFejFullSet','DeltaR1XphiFejFullSet','DeltaR2XphiFejFullSet', ...
            'DeltaR2XpFejFullSet','R2PFejFullSet','DeltaR1XpFejFullSet','R1PFejFullSet', ...
            'DeltaXfFejFullSet','PfFejFullSet')
    elseif fc == 2
        load('MTE_results_StdEKF_60fea.mat','poseNum','feaNum', ...
            'DeltaR1XrFullSet','DeltaR2XrFullSet','DeltaR1XphiFullSet','DeltaR2XphiFullSet', ...
            'DeltaR2XpFullSet','R2PFullSet','DeltaR1XpFullSet','R1PFullSet', ...
            'DeltaXfFullSet','PfFullSet')
        load('MTE_results_IdeEKF_60fea.mat', ...
            'DeltaR1XrIdeFullSet','DeltaR2XrIdeFullSet','DeltaR1XphiIdeFullSet','DeltaR2XphiIdeFullSet', ...
            'DeltaR2XpIdeFullSet','R2PIdeFullSet','DeltaR1XpIdeFullSet','R1PIdeFullSet', ...
            'DeltaXfIdeFullSet','PfIdeFullSet')
        load('MTE_results_FejEKF_60fea.mat', ...
            'DeltaR1XrFejFullSet','DeltaR2XrFejFullSet','DeltaR1XphiFejFullSet','DeltaR2XphiFejFullSet', ...
            'DeltaR2XpFejFullSet','R2PFejFullSet','DeltaR1XpFejFullSet','R1PFejFullSet', ...
            'DeltaXfFejFullSet','PfFejFullSet')
    else
        load('MTE_results_StdEKF_100fea.mat','poseNum','feaNum', ...
            'DeltaR1XrFullSet','DeltaR2XrFullSet','DeltaR1XphiFullSet','DeltaR2XphiFullSet', ...
            'DeltaR2XpFullSet','R2PFullSet','DeltaR1XpFullSet','R1PFullSet', ...
            'DeltaXfFullSet','PfFullSet')
        load('MTE_results_IdeEKF_100fea.mat', ...
            'DeltaR1XrIdeFullSet','DeltaR2XrIdeFullSet','DeltaR1XphiIdeFullSet','DeltaR2XphiIdeFullSet', ...
            'DeltaR2XpIdeFullSet','R2PIdeFullSet','DeltaR1XpIdeFullSet','R1PIdeFullSet', ...
            'DeltaXfIdeFullSet','PfIdeFullSet')
        load('MTE_results_FejEKF_100fea.mat', ...
            'DeltaR1XrFejFullSet','DeltaR2XrFejFullSet','DeltaR1XphiFejFullSet','DeltaR2XphiFejFullSet', ...
            'DeltaR2XpFejFullSet','R2PFejFullSet','DeltaR1XpFejFullSet','R1PFejFullSet', ...
            'DeltaXfFejFullSet','PfFejFullSet')
    end

    %% Root Mean Square Error (RMSE)
    % standard EKF
    [R1XrRMSE,R1XrRMSE_mean] = XrRMSE(poseNum,mcNum,DeltaR1XrFullSet);
    [R2XrRMSE,R2XrRMSE_mean] = XrRMSE(poseNum,mcNum,DeltaR2XrFullSet);

    [R1XphiRMSE,R1XphiRMSE_mean] = XphiRMSE(poseNum,mcNum,DeltaR1XphiFullSet);
    [R2XphiRMSE,R2XphiRMSE_mean] = XphiRMSE(poseNum,mcNum,DeltaR2XphiFullSet);

    [XfStdRMSE,XfStdRMSE_mean] = XFRMSE(feaNum,mcNum,DeltaXfFullSet);



    % ideal EKF
    [R1XrIdeRMSE,R1XrIdeRMSE_mean] = XrRMSE(poseNum,mcNum,DeltaR1XrIdeFullSet);
    [R2XrIdeRMSE,R2XrIdeRMSE_mean] = XrRMSE(poseNum,mcNum,DeltaR2XrIdeFullSet);

    [R1XphiIdeRMSE,R1XphiIdeRMSE_mean] = XphiRMSE(poseNum,mcNum,DeltaR1XphiIdeFullSet);
    [R2XphiIdeRMSE,R2XphiIdeRMSE_mean] = XphiRMSE(poseNum,mcNum,DeltaR2XphiIdeFullSet);

    [XfIdeRMSE,XfIdeRMSE_mean] = XFRMSE(feaNum,mcNum,DeltaXfIdeFullSet);



    % FEJ EKF
    [R1XrFejRMSE,R1XrFejRMSE_mean] = XrRMSE(poseNum,mcNum,DeltaR1XrFejFullSet);
    [R2XrFejRMSE,R2XrFejRMSE_mean] = XrRMSE(poseNum,mcNum,DeltaR2XrFejFullSet);

    [R1XphiFejRMSE,R1XphiFejRMSE_mean] = XphiRMSE(poseNum,mcNum,DeltaR1XphiFejFullSet);
    [R2XphiFejRMSE,R2XphiFejRMSE_mean] = XphiRMSE(poseNum,mcNum,DeltaR2XphiFejFullSet);

    [XfFejRMSE,XfFejRMSE_mean] = XFRMSE(feaNum,mcNum,DeltaXfFejFullSet);

    poseNum_1round = size(DeltaR1XrIdeFullSet,1)/cy;

    % Plot RMSE
    % R1 postion
    figure((fc-1)*6+1)
    hold on
    % standard EKF
    R1XrRMSEP = plot(R1XrRMSE(:,1)',R1XrRMSE(:,2)','--b','DisplayName','R1 std EKF');
    % ideal EKF
    R1XrIdeRMSEP = plot(R1XrIdeRMSE(:,1)',R1XrIdeRMSE(:,2)','-k','DisplayName','R1 ideal EKF');
    % FEJ EKF
    R1XrFejRMSEP = plot(R1XrFejRMSE(:,1)',R1XrFejRMSE(:,2)','-r.','DisplayName','R1 FEJ EKF');

    xline(poseNum_1round,'--g',sprintf('x = %d', poseNum_1round))

    xlabel('Steps')
    ylabel('R1 Position RMSE (m)')
    xlim([0,poseNum])
    legend([R1XrRMSEP,R1XrIdeRMSEP,R1XrFejRMSEP])
    title(sprintf('%d features',feaNum))

    hold off

    %% R2 postion
    figure((fc-1)*6+2)
    hold on
    R2XrRMSEP = plot(R2XrRMSE(:,1)',R2XrRMSE(:,2)','--b','DisplayName','R2 std EKF');
    R2XrIdeRMSEP = plot(R2XrIdeRMSE(:,1)',R2XrIdeRMSE(:,2)','-k','DisplayName','R2 ideal EKF');
    R2XrFejRMSEP = plot(R2XrFejRMSE(:,1)',R2XrFejRMSE(:,2)','-r.','DisplayName','R2 FEJ EKF');

    xline(poseNum_1round,'--g',sprintf('x = %d', poseNum_1round))

    xlabel('Steps')
    ylabel('R2 Position RMSE (m)')
    xlim([0,poseNum])
    legend([R2XrRMSEP,R2XrIdeRMSEP,R2XrFejRMSEP])
    title(sprintf('%d features',feaNum))

    hold off

    %% R1 orientation
    figure((fc-1)*6+3)
    hold on
    % standard EKF
    R1XphiRMSEP = plot(R1XphiRMSE(:,1)',R1XphiRMSE(:,2)','--b','DisplayName','R1 std EKF');
    % ideal EKF
    R1XphiIdeRMSEP = plot(R1XphiIdeRMSE(:,1)',R1XphiIdeRMSE(:,2)','-k','DisplayName','R1 ideal EKF');
    % FEJ EKF
    R1XphiFejRMSEP = plot(R1XphiFejRMSE(:,1)',R1XphiFejRMSE(:,2)','-r.','DisplayName','R1 FEJ EKF');

    xline(poseNum_1round,'--g',sprintf('x = %d', poseNum_1round))

    xlabel('Steps')
    ylabel('R1 Heading RMSE (rad)')
    xlim([0,poseNum])
    legend([R1XphiRMSEP,R1XphiIdeRMSEP,R1XphiFejRMSEP])
    title(sprintf('%d features',feaNum))

    hold off

    %% R2 orientation
    figure((fc-1)*6+4)
    hold on
    R2XphiRMSEP = plot(R2XphiRMSE(:,1)',R2XphiRMSE(:,2)','--b','DisplayName','R2 std EKF');
    R2XphiIdeRMSEP = plot(R2XphiIdeRMSE(:,1)',R2XphiIdeRMSE(:,2)','-k','DisplayName','R2 ideal EKF');
    R2XphiFejRMSEP = plot(R2XphiFejRMSE(:,1)',R2XphiFejRMSE(:,2)','-r.','DisplayName','R2 FEJ EKF');

    xline(poseNum_1round,'--g',sprintf('x = %d', poseNum_1round))

    xlabel('Steps')
    ylabel('R2 Heading RMSE (rad)')
    xlim([0,poseNum])
    legend([R2XphiRMSEP,R2XphiIdeRMSEP,R2XphiFejRMSEP])
    title(sprintf('%d features',feaNum))

    hold off



    %% Plot Average Normalized (state) Estimation Error Squared (ANEES) for the last step
    % standard EKF
    R1NEES_k = [(1:poseNum)',zeros(poseNum,mcNum)];
    R2NEES_k = [(0:poseNum)',zeros(poseNum+1,mcNum)];
    for k = 0:poseNum

        %% Debug
        % if k == 34 || k == 60
        %     keyboard;
        % end
        %%

        for mn = 1:mcNum
            R2xri_e = DeltaR2XpFullSet(k*3+(1:3),1+mn);
            R2C_xrie = R2PFullSet(k*3+(1:3),(mn-1)*3+(1:3));
            R2NEES_k(k+1,1+mn) = R2xri_e'/R2C_xrie*R2xri_e;

            if k == 0
                continue
            else
                R1xri_e = DeltaR1XpFullSet(k*3+(1:3),1+mn);
                R1C_xrie = R1PFullSet(k*3+(1:3),(mn-1)*3+(1:3));
                R1NEES_k(k,1+mn) = R1xri_e'/R1C_xrie*R1xri_e;
            end
        end
    end

    R1XpNEES = [R1NEES_k(:,1),mean(R1NEES_k(:,2:end),2)];
    R2XpNEES = [R2NEES_k(:,1),mean(R2NEES_k(:,2:end),2)];

    R1XpNEES_mean = mean(R1XpNEES(:,2))/3;

    R2XpNEES_mean = mean(R2XpNEES(:,2))/3;

    XfNEES_k = [DeltaXfFullSet(1:2:(end-1),1),zeros(feaNum,mcNum)];

    for j = 1:feaNum
        for mn = 1:mcNum
            xfi_e = DeltaXfFullSet((j-1)*2+(1:2),1+mn);
            C_xfie = PfFullSet((j-1)*2+(1:2),(mn-1)*feaNum*2+(j-1)*2+(1:2));
            XfNEES_k(j,1+mn) = xfi_e'/C_xfie*xfi_e;
        end
    end

    XfNEES = [XfNEES_k(:,1),mean(XfNEES_k(:,2:end),2)];
    XfNEES_mean = mean(XfNEES(:,2))/2;



    % ideal EKF
    R1IdeNEES_k = [(1:poseNum)',zeros(poseNum,mcNum)];
    R2IdeNEES_k = [(0:poseNum)',zeros(poseNum+1,mcNum)];
    for k = 0:poseNum

        for mn = 1:mcNum
            R2Idexri_e = DeltaR2XpIdeFullSet(k*3+(1:3),1+mn);
            R2IdeC_xrie = R2PIdeFullSet(k*3+(1:3),(mn-1)*3+(1:3));
            R2IdeNEES_k(k+1,1+mn) = R2Idexri_e'/R2IdeC_xrie*R2Idexri_e;

            if k == 0
                continue
            else
                R1Idexri_e = DeltaR1XpIdeFullSet(k*3+(1:3),1+mn);
                R1IdeC_xrie = R1PIdeFullSet(k*3+(1:3),(mn-1)*3+(1:3));
                R1IdeNEES_k(k,1+mn) = R1Idexri_e'/R1IdeC_xrie*R1Idexri_e;
            end
        end
    end

    R1XpIdeNEES = [R1IdeNEES_k(:,1),mean(R1IdeNEES_k(:,2:end),2)];
    R2XpIdeNEES = [R2IdeNEES_k(:,1),mean(R2IdeNEES_k(:,2:end),2)];

    R1XpIdeNEES_mean = mean(R1XpIdeNEES(:,2))/3;

    R2XpIdeNEES_mean = mean(R2XpIdeNEES(:,2))/3;

    XfIdeNEES_k = [DeltaXfIdeFullSet(1:2:(end-1),1),zeros(feaNum,mcNum)];

    for j = 1:feaNum
        for mn = 1:mcNum
            Idexfi_e = DeltaXfIdeFullSet((j-1)*2+(1:2),1+mn);
            IdeC_xfie = PfIdeFullSet((j-1)*2+(1:2),(mn-1)*feaNum*2+(j-1)*2+(1:2));
            XfIdeNEES_k(j,1+mn) = Idexfi_e'/IdeC_xfie*Idexfi_e;
        end
    end

    XfIdeNEES = [XfIdeNEES_k(:,1),mean(XfIdeNEES_k(:,2:end),2)];
    XfIdeNEES_mean = mean(XfIdeNEES(:,2))/2;



    %% Debug
    % if feaNum == 20
    %     keyboard
    % end

    % FEJ EKF
    R1FejNEES_k = [(1:poseNum)',zeros(poseNum,mcNum)];
    R2FejNEES_k = [(0:poseNum)',zeros(poseNum+1,mcNum)];

    for k = 0:poseNum

        for mn = 1:mcNum
            R2Fejxri_e = DeltaR2XpFejFullSet(k*3+(1:3),1+mn);
            R2FejC_xrie = R2PFejFullSet(k*3+(1:3),(mn-1)*3+(1:3));
            R2FejNEES_k(k+1,1+mn) = R2Fejxri_e'/R2FejC_xrie*R2Fejxri_e;

            if k == 0
                continue
            else
                R1Fejxri_e = DeltaR1XpFejFullSet(k*3+(1:3),1+mn);
                R1FejC_xrie = R1PFejFullSet(k*3+(1:3),(mn-1)*3+(1:3));
                R1FejNEES_k(k,1+mn) = R1Fejxri_e'/R1FejC_xrie*R1Fejxri_e;
            end
        end
    end

    R1XpFejNEES = [R1FejNEES_k(:,1),mean(R1FejNEES_k(:,2:end),2)];
    R2XpFejNEES = [R2FejNEES_k(:,1),mean(R2FejNEES_k(:,2:end),2)];

    R1XpFejNEES_mean = mean(R1XpFejNEES(:,2))/3;
    R2XpFejNEES_mean = mean(R2XpFejNEES(:,2))/3;

    XfFejNEES_k = [DeltaXfFejFullSet(1:2:(end-1),1),zeros(feaNum,mcNum)];

    for j = 1:feaNum
        for mn = 1:mcNum
            Fejxfi_e = DeltaXfFejFullSet((j-1)*2+(1:2),1+mn);
            FejC_xfie = PfFejFullSet((j-1)*2+(1:2),(mn-1)*feaNum*2+(j-1)*2+(1:2));
            XfFejNEES_k(j,1+mn) = Fejxfi_e'/FejC_xfie*Fejxfi_e;
        end
    end

    XfFejNEES = [XfFejNEES_k(:,1),mean(XfFejNEES_k(:,2:end),2)];
    XfFejNEES_mean = mean(XfFejNEES(:,2))/2;



    %% R1 pose
    figure((fc-1)*6+5)
    hold on
    % standard EKF
    R1XpNEESP = plot(R1XpNEES(:,1)',R1XpNEES(:,2)','--b','DisplayName','R1 std EKF');
    % ideal EKF
    R1XpIdeNEESP = plot(R1XpIdeNEES(:,1)',R1XpIdeNEES(:,2)','-k','DisplayName','R1 ideal EKF');
    % FEJ EKF
    R1XpFejNEESP = plot(R1XpFejNEES(:,1)',R1XpFejNEES(:,2)','-r.','DisplayName','R1 FEJ EKF');   
    
    xline(poseNum_1round,'--g',sprintf('x = %d', poseNum_1round))

    xlabel('Steps')
    ylabel('R1 Pose ANEES')
    xlim([0,poseNum])
    legend([R1XpNEESP,R1XpIdeNEESP,R1XpFejNEESP])
    title(sprintf('%d features',feaNum))

    hold off

    %% R2 pose
    figure((fc-1)*6+6)
    hold on
    R2XpNEESP = plot(R2XpNEES(:,1)',R2XpNEES(:,2)','--b','DisplayName','R2 std EKF');
    R2XpIdeNEESP = plot(R2XpIdeNEES(:,1)',R2XpIdeNEES(:,2)','-k','DisplayName','R2 ideal EKF');
    R2XpFejNEESP = plot(R2XpFejNEES(:,1)',R2XpFejNEES(:,2)','-r.','DisplayName','R2 FEJ EKF');
    
    xline(poseNum_1round,'--g',sprintf('x = %d', poseNum_1round))

    xlabel('Steps')
    ylabel('R2 Pose ANEES')
    xlim([0,poseNum])
    legend([R2XpNEESP,R2XpIdeNEESP,R2XpFejNEESP])
    title(sprintf('%d features',feaNum))

    hold off



    %% Display the mean
    if fc == 1

        disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 20 features %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
        IdeEKFColumn = [R1XrIdeRMSE_mean;R1XphiIdeRMSE_mean;R2XrIdeRMSE_mean;R2XphiIdeRMSE_mean;XfIdeRMSE_mean;R1XpIdeNEES_mean;R2XpIdeNEES_mean;XfIdeNEES_mean];
        StdEKFColumn = [R1XrRMSE_mean;R1XphiRMSE_mean;R2XrRMSE_mean;R2XphiRMSE_mean;XfStdRMSE_mean;R1XpNEES_mean;R2XpNEES_mean;XfNEES_mean];
        FejEKFColumn = [R1XrFejRMSE_mean;R1XphiFejRMSE_mean;R2XrFejRMSE_mean;R2XphiFejRMSE_mean;XfFejRMSE_mean;R1XpFejNEES_mean;R2XpFejNEES_mean;XfFejNEES_mean];

        % 创建表格并插入动态列名称
        T = table(IdeEKFColumn, StdEKFColumn,FejEKFColumn,'VariableNames',{'Ideal EKF','Std EKF','FEJ EKF'});

        % 设置行名称
        T.Properties.RowNames = {'R1 Position Err.RMS (m)','R1 Heading Err.RMS (rad)','R2 Position Err.RMS (m)','R2 Heading Err.RMS (rad)','Landmark Position Err.RMS (m)','R1 Pose ANEES','R2 Pose ANEES','Feature ANEES'};

        % 显示表格
        disp(T);
        disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')

    elseif fc == 2

        disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 60 features %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
        IdeEKFColumn = [R1XrIdeRMSE_mean;R1XphiIdeRMSE_mean;R2XrIdeRMSE_mean;R2XphiIdeRMSE_mean;XfIdeRMSE_mean;R1XpIdeNEES_mean;R2XpIdeNEES_mean;XfIdeNEES_mean];
        StdEKFColumn = [R1XrRMSE_mean;R1XphiRMSE_mean;R2XrRMSE_mean;R2XphiRMSE_mean;XfStdRMSE_mean;R1XpNEES_mean;R2XpNEES_mean;XfNEES_mean];
        FejEKFColumn = [R1XrFejRMSE_mean;R1XphiFejRMSE_mean;R2XrFejRMSE_mean;R2XphiFejRMSE_mean;XfFejRMSE_mean;R1XpFejNEES_mean;R2XpFejNEES_mean;XfFejNEES_mean];

        % 创建表格并插入动态列名称
        T = table(IdeEKFColumn, StdEKFColumn,FejEKFColumn,'VariableNames',{'Ideal EKF','Std EKF','FEJ EKF'});

        % 设置行名称
        T.Properties.RowNames = {'R1 Position Err.RMS (m)','R1 Heading Err.RMS (rad)','R2 Position Err.RMS (m)','R2 Heading Err.RMS (rad)','Landmark Position Err.RMS (m)','R1 Pose ANEES','R2 Pose ANEES','Feature ANEES'};

        % 显示表格
        disp(T);
        disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')

    else

        disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 100 features %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
        IdeEKFColumn = [R1XrIdeRMSE_mean;R1XphiIdeRMSE_mean;R2XrIdeRMSE_mean;R2XphiIdeRMSE_mean;XfIdeRMSE_mean;R1XpIdeNEES_mean;R2XpIdeNEES_mean;XfIdeNEES_mean];
        StdEKFColumn = [R1XrRMSE_mean;R1XphiRMSE_mean;R2XrRMSE_mean;R2XphiRMSE_mean;XfStdRMSE_mean;R1XpNEES_mean;R2XpNEES_mean;XfNEES_mean];
        FejEKFColumn = [R1XrFejRMSE_mean;R1XphiFejRMSE_mean;R2XrFejRMSE_mean;R2XphiFejRMSE_mean;XfFejRMSE_mean;R1XpFejNEES_mean;R2XpFejNEES_mean;XfFejNEES_mean];

        % 创建表格并插入动态列名称
        T = table(IdeEKFColumn, StdEKFColumn,FejEKFColumn,'VariableNames',{'Ideal EKF','Std EKF','FEJ EKF'});

        % 设置行名称
        T.Properties.RowNames = {'R1 Position Err.RMS (m)','R1 Heading Err.RMS (rad)','R2 Position Err.RMS (m)','R2 Heading Err.RMS (rad)','Landmark Position Err.RMS (m)','R1 Pose ANEES','R2 Pose ANEES','Feature ANEES'};

        % 显示表格
        disp(T);
        disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')

    end

end

exportgraphics(figure(1),fullfile(figuresFolderPath, 'R1Postion_RMSE_20feas.jpg'), 'Resolution', 300)
exportgraphics(figure(2),fullfile(figuresFolderPath, 'R2Postion_RMSE_20feas.jpg'), 'Resolution', 300)
exportgraphics(figure(3),fullfile(figuresFolderPath, 'R1Orientation_RMSE_20feas.jpg'), 'Resolution', 300)
exportgraphics(figure(4),fullfile(figuresFolderPath, 'R2Orientation_RMSE_20feas.jpg'), 'Resolution', 300)
exportgraphics(figure(5),fullfile(figuresFolderPath, 'R1Pose_ANEES_20feas.jpg'), 'Resolution', 300)
exportgraphics(figure(6),fullfile(figuresFolderPath, 'R2Pose_ANEES_20feas.jpg'), 'Resolution', 300)

exportgraphics(figure(7),fullfile(figuresFolderPath, 'R1Postion_RMSE_60feas.jpg'), 'Resolution', 300)
exportgraphics(figure(8),fullfile(figuresFolderPath, 'R2Postion_RMSE_60feas.jpg'), 'Resolution', 300)
exportgraphics(figure(9),fullfile(figuresFolderPath, 'R1Orientation_RMSE_60feas.jpg'), 'Resolution', 300)
exportgraphics(figure(10),fullfile(figuresFolderPath, 'R2Orientation_RMSE_60feas.jpg'), 'Resolution', 300)
exportgraphics(figure(11),fullfile(figuresFolderPath, 'R1Pose_ANEES_60feas.jpg'), 'Resolution', 300)
exportgraphics(figure(12),fullfile(figuresFolderPath, 'R2Pose_ANEES_60feas.jpg'), 'Resolution', 300)

exportgraphics(figure(13),fullfile(figuresFolderPath, 'R1Postion_RMSE_100feas.jpg'), 'Resolution', 300)
exportgraphics(figure(14),fullfile(figuresFolderPath, 'R2Postion_RMSE_100feas.jpg'), 'Resolution', 300)
exportgraphics(figure(15),fullfile(figuresFolderPath, 'R1Orientation_RMSE_100feas.jpg'), 'Resolution', 300)
exportgraphics(figure(16),fullfile(figuresFolderPath, 'R2Orientation_RMSE_100feas.jpg'), 'Resolution', 300)
exportgraphics(figure(17),fullfile(figuresFolderPath, 'R1Pose_ANEES_100feas.jpg'), 'Resolution', 300)
exportgraphics(figure(18),fullfile(figuresFolderPath, 'R2Pose_ANEES_100feas.jpg'), 'Resolution', 300)