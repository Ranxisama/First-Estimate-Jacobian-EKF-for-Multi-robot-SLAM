clc
clear
close all

Config;

load Zstate_VicPark_6898_loops

poseNum = 6897; % maximum 6897

odo = Zstate(Zstate(:,2)==1,[4,3,1]);
odo_idx = find(odo(:,2)==poseNum,1,"last");
Odo = odo(1:odo_idx,:);

obs = Zstate(Zstate(:,2)==2,[4,3,1]);
obs_idx = find(obs(:,1)==poseNum,1,"last");
Obs = obs(1:obs_idx,:);
feaIDs = unique(Obs(:,2));
feaNum = numel(feaIDs);

if ~exist(fullfile(pwd,'X_GNI.mat'),'file')
    %% Gauss-Newton Iteration finding revisited robot position

    % esitmate X state
    robPose_k = [0;0;0];
    robPose = zeros(3,2);

    feaPosis_k = [];
    feaPosis = Obs;

    for k = 0:poseNum
        if k ~= 0
            Odo_k = Odo(Odo(:,2)==k,3);
            robPose_k = [robPose_k(1:2,1)+Rot(robPose_k(3,1))*Odo_k(1:2,1);wrap(robPose_k(3,1)+Odo_k(3,1))];
            robPose = [robPose;k*ones(3,1),robPose_k];
        end

        Obs_k = Obs(Obs(:,1)==k,:);
        for kj1 = 1:(size(Obs_k,1)/2)
            Obs_kj = Obs_k((kj1-1)*2+(1:2),:);
            feaPosi_kj = Rot(robPose_k(3,1))*Obs_kj(1:2,3)+ robPose_k(1:2,1);
            feaPosis_k = [feaPosis_k;feaPosi_kj];
        end
    end


    feaPosis(:,3) = feaPosis_k;

    [~, feaPosis_idx] = unique(feaPosis(:,2), 'first');
    feaPosi_idx = zeros(2*size(feaPosis_idx,1),1);
    for j = 1:size(feaPosis_idx,1)
        feaPosi_idx((j-1)*2+(1:2),1) = [feaPosis_idx(j,1);feaPosis_idx(j,1)+1];
    end
    feaPosi = feaPosis(feaPosi_idx,2:3);

    X = [ones(size(robPose,1)-3,1) robPose(4:end,:);2*ones(size(feaPosi,1),1) feaPosi];


    figure(1)
    hold on
    plot(robPose(1:3:(end-2),2),robPose(2:3:(end-1),2),'ro')
    hold off

    % Gauss-Newton Iteration start
    Z = [Odo;Obs];


    for i = 1:100
        FX_odo = [];
        FX_obs = [];

        FX = Z;
        Jab = sparse(size(Z,1),poseNum*3+feaNum*2);

        for k = 0:(poseNum-1)
            if k == 0
                Xrk1 = robPose(1:3,2);
                Xrk2 = X(X(:,1)==1 & X(:,2)==1,3);

                Jab(1:3,1:3) = ...
                    [Rot(Xrk1(3,1))',zeros(2,1); ...
                    0,0,1];
            else
                Xrk1 = X(X(:,1)==1 & X(:,2)==k,3);
                Xrk2 = X(X(:,1)==1 & X(:,2)==k+1,3);

                % Jab(k*3+(1:3),(k-1)*3+(1:6)) = ...
                %     [-Rot(Xrk1(3,1))',Rcon(-J*Rot(Xrk1(3,1))'*(Xrk2(1:2,1)-Xrk1(1:2,1)),CovT),Rot(Xrk1(3,1))',zeros(2,1); ...
                %     0,0,-1,0,0,1];
                Jab(k*3+(1:3),(k-1)*3+(1:6)) = ...
                    [-Rot(Xrk1(3,1))',-J*Rot(Xrk1(3,1))'*(Xrk2(1:2,1)-Xrk1(1:2,1)),Rot(Xrk1(3,1))',zeros(2,1); ...
                    0,0,-1,0,0,1];
            end

            FX_odo = [FX_odo;Rot(Xrk1(3,1))'*(Xrk2(1:2,1)-Xrk1(1:2,1));wrap(Xrk2(3,1)-Xrk1(3,1))];
        end



        for kj1 = 1:(size(Obs,1)/2)
            Obs_kj = Obs((kj1-1)*2+(1:2),:);
            feakID = Obs_kj(1,2);
            robkID = Obs_kj(1,1);
            feakID_idx = find(feaIDs(:,1)==feakID);

            Xfj = X(X(:,1)==2 & X(:,2)==feakID,3);
            if robkID == 0
                Xrk = robPose(1:3,2);

                Jab(size(Odo,1)+(kj1-1)*2+(1:2),size(Odo,1)+(feakID_idx-1)*2+(1:2)) = Rot(Xrk(3,1))';
            else
                Xrk = X(X(:,1)==1 & X(:,2)==robkID,3);

                % Jab(size(Odo,1)+(kj-1)*2+(1:2),(robkID-1)*3+(1:3)) = ...
                %     [-Rot(Xrk(3,1))',Rcon(-J*Rot(Xrk(3,1))'*(Xfj(1:2,1)-Xrk(1:2,1)),CovT)];
                Jab(size(Odo,1)+(kj1-1)*2+(1:2),(robkID-1)*3+(1:3)) = ...
                    [-Rot(Xrk(3,1))',-J*Rot(Xrk(3,1))'*(Xfj(1:2,1)-Xrk(1:2,1))];

                Jab(size(Odo,1)+(kj1-1)*2+(1:2),size(Odo,1)+(feakID_idx-1)*2+(1:2)) = Rot(Xrk(3,1))';
            end

            FX_obs = [FX_obs;Rot(Xrk(3,1))'*(Xfj(1:2,1)-Xrk(1:2,1))];
        end



        FX(:,3) = [FX_odo;FX_obs];

        Xold = X(:,3);

        % Jab = Rcon(Jab,CovT);

        % X_b = Rcon(Jab'*(Z(:,3)-FX(:,3)),CovT);
        % full(Jab)
        % DtX = Rcon(pinv(Jab'*Jab)*X_b,CovT);

        Xb = Jab'*(Z(:,3)-FX(:,3));
        DtX = (Jab'*Jab)\Xb;

        X(:,3) = X(:,3) + DtX;

        Xnew = X(:,3);

        D = Xnew - Xold;
        DD = D'*D;
        if DD < CC
            break
        end
    end

    figure(2)
    hold on
    plot(X(1:3:(poseNum*3-2),3),X(2:3:(poseNum*3-1),3),'bo');
    hold off

    save('X_GNI.mat','X')
else
    load('X_GNI.mat','X')
end

%% %%%%%%%%%%% find the suitable robot poses for R1 and R2 %%%%%%%%%%%%%

% figure(3)
% hold on
% robPosiP = plot(X(1:3:(poseNum*3-2),3),X(2:3:(poseNum*3-1),3),'bo');
% % for k = 1:poseNum
% %     text(X((k-1)*3+1,3),X((k-1)*3+2,3),num2str(X((k-1)*3+1,2)),'Color','b')
% % end
% 
% feaPosiP = plot(X(poseNum*3+(1:2:(feaNum*2-1)),3),X(poseNum*3+(2:2:(feaNum*2)),3),'r^');
% hold off

% %% find pose with more than 8 shared features
% K = [];
% for k = 1:poseNum
%     Obs_k = Obs(Obs(:,1)==k,:);
% 
%     ObskNum = size(Obs_k,1)/2;
% 
%     %% 选第几步？
%     if ObskNum >= 8
%         K = [K;k];
% 
%         if k > 3000 && k < 4000
%             figure(3)
%             hold on
%             sharedPoseP = plot(X((k-1)*3+1,3),X((k-1)*3+2,3),'mo');
%             text(X((k-1)*3+1,3)-0.1,X((k-1)*3+2,3),num2str(k),'Color','m');
%             hold off
%         end
%     end
% 
%     %% 可以选3265步(65, 8)左右，现在找出方向
%     dist = sqrt((X((k-1)*3+1,3)-65)^2+(X((k-1)*3+2,3)-8)^2);
%     if dist < 5
%         figure(3)
%         hold on
%         shared3265P = plot(X((k-1)*3+1,3),X((k-1)*3+2,3),'go');
%         text(X((k-1)*3+1,3)+0.1,X((k-1)*3+2,3),num2str(k),'Color','g');
%         hold off
%     end
% end

%% %%%%%%%%%% 可以选719步开始是R1,3265步开始是R2,两者前进方向相似 %%%%%%%%%%%%%%%%%%%%%

% State 的真值使用高斯牛顿优化的值

% R1的初始位姿测量值（第719步）都使用高斯牛顿迭代后的保留后四位小数的值：[65.0492;7.7931;3.3150];

% R2的初始位姿测量值（第3265步）使用直接计算得到的保留后四位小数的值：[40.1386;-64.1211;-0.8182];

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
XrTrue = X(X(:,1)==1,2:3);
XfTrue = X(X(:,1)==2,2:3);
% Parameters
R1XTrue = XrTrue(((719-1)*3+1):((3264-1)*3+3),:);
R2XTrue = XrTrue(((3265-1)*3+1):((5810-1)*3+3),:);

R1XrT = R1XTrue;
R1XrT(3:3:end,:) = [];

R2XrT = R2XTrue;
R2XrT(3:3:end,:) = [];

R1XphiTrue = R1XTrue(3:3:end,:);
R1XphiTrue(:,2) = wrap(R1XphiTrue(:,2));

R2XphiTrue = R2XTrue(3:3:end,:);
R2XphiTrue(:,2) = wrap(R2XphiTrue(:,2));

% Measurements
R1Xp0Set = [65.0492;7.7931;3.3150];
R1Xp0Set(3,1) = wrap(R1Xp0Set(3,1));

R2Xp0Set = [40.1386;-64.1211;-0.8182];
R2Xp0Set(3,1) = wrap(R2Xp0Set(3,1));

R1ObsIDs = unique(R1XrT(:,1));
R1OdoIDs = R1ObsIDs;
R1OdoIDs(end,:) = [];
[~,R1Odo_idx] = ismember(Odo(:,1),R1OdoIDs);
R1Odoset = Odo(find(R1Odo_idx),:);
[~,R1Obs_idx] = ismember(Obs(:,1),R1ObsIDs);
R1ObsSet = Obs(find(R1Obs_idx),:);

R2ObsIDs = unique(R2XrT(:,1));
R2OdoIDs = R2ObsIDs;
R2OdoIDs(end,:) = [];
[~,R2Odo_idx] = ismember(Odo(:,1),R2OdoIDs);
R2Odoset = Odo(find(R2Odo_idx),:);
[~,R2Obs_idx] = ismember(Obs(:,1),R2ObsIDs);
R2ObsSet = Obs(find(R2Obs_idx),:);

mfeaIDs = unique([R1ObsSet(:,2);R2ObsSet(:,2)]); % measurement feature ID

% Parameters
[~,XfTrue_idx] = ismember(XfTrue(:,1),mfeaIDs);
XfTrueAll = XfTrue(find(XfTrue_idx),:);

PoseIDs1 = (0:(2545))'; % 3264 - 719 = 2545
PoseIDs2 = reshape(repmat(PoseIDs1', 2, 1), [], 1);
OdoIDs3 = reshape(repmat(0:(2544), 3, 1), [], 1);

R1XrTrue = [PoseIDs2,R1XrT(:,2)];
R2XrTrue = [PoseIDs2,R2XrT(:,2)];

R1XphiT = [PoseIDs1,R1XphiTrue(:,2)];
R2XphiT = [PoseIDs1,R2XphiTrue(:,2)];

% Measurements
R1OdoSet = [OdoIDs3,OdoIDs3+1,R1Odoset(:,3)];
R2OdoSet = [OdoIDs3,OdoIDs3+1,R2Odoset(:,3)];

for k =0:(3264-719)
    R1ObsSet(R1ObsSet(:,1)==719+k,1)=k;
    R2ObsSet(R2ObsSet(:,1)==3265+k,1)=k;
end

% Parameters

%% OdoT
R1OdoT = R1OdoSet;
R2OdoT = R2OdoSet;

R1Xpk1 = [];
R1Xpk2 = [];
R2Xpk1 = [];
R2Xpk2 = [];

for k = 0:(2545-1)
    % R1 OdoT
    R1Xpk2 = [R1XrTrue(R1XrTrue(:,1)==k+1,2); ...
        R1XphiT(R1XphiT(:,1)==k+1,2)];

    R1Xpk1 = [R1XrTrue(R1XrTrue(:,1)==k,2); ...
        R1XphiT(R1XphiT(:,1)==k,2)];

    R1OdoT(k*3+(1:3),3) = [Rot(R1Xpk1(3,1))'*(R1Xpk2(1:2,1)-R1Xpk1(1:2,1)); ...
        R1Xpk2(3,1)-R1Xpk1(3,1)];
    % R2 OdoT
    R2Xpk2 = [R2XrTrue(R2XrTrue(:,1)==k+1,2); ...
        R2XphiT(R2XphiT(:,1)==k+1,2)];

    R2Xpk1 = [R2XrTrue(R2XrTrue(:,1)==k,2); ...
        R2XphiT(R2XphiT(:,1)==k,2)];

    R2OdoT(k*3+(1:3),3) = [Rot(R2Xpk1(3,1))'*(R2Xpk2(1:2,1)-R2Xpk1(1:2,1)); ...
        R2Xpk2(3,1)-R2Xpk1(3,1)];
end

%% ObsT
R1ObsT = R1ObsSet;
R2ObsT = R2ObsSet;

R1Xpk = [];
R2Xpk = [];
R1Xfj = [];
R2Xfj = [];

for kj1 = 1:(size(R1ObsT,1)/2)
    % R1 ObsT
    R1ObsT_k = R1ObsT((kj1-1)*2+(1:2),:);
    poseID = R1ObsT_k(1,1);
    feaID = R1ObsT_k(1,2);

    R1Xpk = [R1XrTrue(R1XrTrue(:,1)==poseID,2); ...
        R1XphiT(R1XphiT(:,1)==poseID,2)];

    R1Xfj = XfTrueAll(XfTrueAll(:,1)==feaID,2);

    R1ObsT((kj1-1)*2+(1:2),3) = Rot(R1Xpk(3,1))'*(R1Xfj-R1Xpk(1:2,1));
end

for kj2 = 1:(size(R2ObsT,1)/2)
    % R1 ObsT
    R2ObsT_k = R2ObsT((kj2-1)*2+(1:2),:);
    poseID = R2ObsT_k(1,1);
    feaID = R2ObsT_k(1,2);

    R2Xpk = [R2XrTrue(R2XrTrue(:,1)==poseID,2); ...
        R2XphiT(R2XphiT(:,1)==poseID,2)];

    R2Xfj = XfTrueAll(XfTrueAll(:,1)==feaID,2);

    R2ObsT((kj2-1)*2+(1:2),3) = Rot(R2Xpk(3,1))'*(R2Xfj-R2Xpk(1:2,1));
end

save('VicP_Parameters.mat','R1XrTrue','R1XphiT','R2XrTrue','R2XphiT','XfTrueAll','R1OdoT','R2OdoT','R1ObsT','R2ObsT')
save('VicP_Measurements.mat','R1Xp0Set','R1OdoSet','R1ObsSet','R2Xp0Set','R2OdoSet','R2ObsSet')

figure(4)
hold on
grid on
R1XrTrueP = plot(R1XrTrue(1:2:(end-1),2),R1XrTrue(2:2:end,2),'c-','DisplayName','R1 trajectory','MarkerSize',2);
R2XrTrueP = plot(R2XrTrue(1:2:(end-1),2),R2XrTrue(2:2:end,2),'m--','DisplayName','R2 trajectory','MarkerSize',2);
FeaTrueP = plot(XfTrueAll(1:2:(end-1),2),XfTrueAll(2:2:end,2),'k^','DisplayName','Feature position','MarkerSize',3);

% 为 legend 创建正常尺寸的标记（不显示在图中）
R1XrTruePHandle = plot(NaN, NaN, 'c-','DisplayName','R1 trajectory', 'MarkerSize', 8);  % 正常尺寸标记
R2XrTruePHandle = plot(NaN, NaN, 'm--','DisplayName','R2 trajectory', 'MarkerSize', 8);  % 正常尺寸标记
FeaTruePHandle = plot(NaN, NaN, 'k^','DisplayName','Feature position', 'MarkerSize', 8);  % 正常尺寸标记

legend([R1XrTruePHandle,R2XrTruePHandle,FeaTruePHandle])

xlabel('x (m)')
ylabel('y (m)')

set(gcf, 'Color', 'w');  % 将整个图背景设置为白色
set(gca, 'Box', 'on', 'LineWidth', 1, 'GridLineStyle', '--', 'GridAlpha', 0.1);  % 使边框显示，并增加边框宽度
hold off

%% save output figures
currentFolder = fileparts(mfilename('fullpath'));
subFolder = 'saved_figures';
figuresFolderPath = fullfile(currentFolder, subFolder);
if ~exist(figuresFolderPath, 'dir')
    mkdir(figuresFolderPath);
end

export_fig(fullfile(figuresFolderPath, 'VicP_Xposi_GNI.jpg'), '-jpg', '-r300', figure(4));