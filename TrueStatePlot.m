close all
clear

Config;

%% save output figures
currentFolder = fileparts(mfilename('fullpath'));
subFolder = 'saved_figures';
figuresFolderPath = fullfile(currentFolder, subFolder);
if ~exist(figuresFolderPath, 'dir')
    mkdir(figuresFolderPath);
end


    if ec == 1
        load('MT_Parameters_20fea.mat', ...
            'R1XrTrue','R1XphiT','R2XrTrue','R2XphiT','XfTrueAll')
    elseif ec == 2
        load('MT_Parameters_60fea.mat', ...
            'R1XrTrue','R1XphiT','R2XrTrue','R2XphiT','XfTrueAll')
    elseif ec == 3
        load('MT_Parameters_100fea.mat', ...
            'R1XrTrue','R1XphiT','R2XrTrue','R2XphiT','XfTrueAll')
    end

    if ec == 1
        R1xTrue1 = R1XrTrue(1:2:(end-3),2);
        R1yTrue1 = R1XrTrue(2:2:(end-2),2);
        R1xTrue2 = R1XrTrue(3:2:(end-1),2);
        R1yTrue2 = R1XrTrue(4:2:(end),2);
        % Average position odometry
        R1AvepOdo = mean(sqrt((R1xTrue1-R1xTrue2).^2+(R1yTrue1-R1yTrue2).^2));

        R2xTrue1 = R2XrTrue(1:2:(end-3),2);
        R2yTrue1 = R2XrTrue(2:2:(end-2),2);
        R2xTrue2 = R2XrTrue(3:2:(end-1),2);
        R2yTrue2 = R2XrTrue(4:2:(end),2);
        % Average position odometry
        R2AvepOdo = mean(sqrt((R2xTrue1-R2xTrue2).^2+(R2yTrue1-R2yTrue2).^2));

        R1phiTrue1 = R1XphiT(1:(end-1),2);
        R1phiTrue2 = R1XphiT(2:(end),2);
        % Average heading odometry
        R1AvehOdo = mean(abs(R1phiTrue2-R1phiTrue1));

        R2phiTrue1 = R2XphiT(1:(end-1),2);
        R2phiTrue2 = R2XphiT(2:(end),2);
        % Average heading odometry
        R2AvehOdo = mean(abs(R2phiTrue2-R2phiTrue1));

        fprintf('average position true odometry of R1: %f\n',R1AvepOdo)
        fprintf('average position true odometry of R2: %f\n',R2AvepOdo)

        fprintf('average heading true odometry of R1: %f\n',R1AvehOdo)
        fprintf('average heading true odometry of R2: %f\n',R2AvehOdo)
    end

figure(ec)
hold on
grid on
% axis([fea_xlb,fea_xub,fea_ylb,fea_yub])
axis([-50,50,-50,50])
R1WaypointsTP = plot(R1XrTrue(1:2:(end-1),2),R1XrTrue(2:2:end,2),'co','MarkerSize',2,'DisplayName','R_1 position');
R2WaypointsTP = plot(R2XrTrue(1:2:(end-1),2),R2XrTrue(2:2:end,2),'ms','MarkerSize',2.5,'DisplayName','R_2 position');

% for legend
R1WaypointsTPHandle = plot(NaN,NaN,'co','MarkerSize',8,'DisplayName','R_1 position');
R2WaypointsTPHandle = plot(NaN,NaN,'ms','MarkerSize',8,'DisplayName','R_2 position');

% arrow length
R1AL = 0.5*sqrt((R1XrTrue(3:2:(end-1),2) - R1XrTrue(1:2:(end-3),2)).^2 + (R1XrTrue(4:2:end,2) - R1XrTrue(2:2:(end-2),2)).^2);
R1AL(end+1,:) = R1AL(end,:);

R2AL = 0.5*sqrt((R2XrTrue(3:2:(end-1),2) - R2XrTrue(1:2:(end-3),2)).^2 + (R2XrTrue(4:2:end,2) - R2XrTrue(2:2:(end-2),2)).^2);
R2AL(end+1,:) = R2AL(end,:);

R1dx = R1AL.* cos(R1XphiT(:,2));
R1dy = R1AL.* sin(R1XphiT(:,2));
R2dx = R2AL.* cos(R2XphiT(:,2));
R2dy = R2AL.* sin(R2XphiT(:,2));

R1HeadingTP = quiver(R1XrTrue(1:2:(end-1),2),R1XrTrue(2:2:end,2),R1dx,R1dy,0,'c','LineWidth',0.5,'DisplayName','R_1 heading');
R2HeadingTP = quiver(R2XrTrue(1:2:(end-1),2),R2XrTrue(2:2:end,2),R2dx,R2dy,0,'-.m','LineWidth',0.5,'DisplayName','R_2 heading');

FeaposiTP = plot(XfTrueAll(1:2:(end-1),2),XfTrueAll(2:2:end,2),'k^','MarkerSize',2,'DisplayName','Feature position');
% for legend
FeaposiTPHandle = plot(NaN,NaN,'k^','MarkerSize',8,'DisplayName','Feature position');

% 创建自定义图例
% hLegend = legend('R_1 true position', 'R_2 true position', 'R_1 true heading', 'R_2 true heading', 'True feature', 'Location', 'northeast','FontSize', FS);
hLegend = legend([R1WaypointsTPHandle,R1HeadingTP,R2WaypointsTPHandle,R2HeadingTP,FeaposiTPHandle]);
xlabel('x (m)')
ylabel('y (m)')

set(gcf, 'Color', 'w');  % 将整个图背景设置为白色
set(gca, 'Box', 'on', 'LineWidth', 1, 'GridLineStyle', '--', 'GridAlpha', 0.1);  % 使边框显示，并增加边框宽度
hold off

if ec == 1
    export_fig(fullfile(figuresFolderPath, 'TrueState20fea.jpg'), '-jpg', '-r300', figure(1));
elseif ec == 2
    export_fig(fullfile(figuresFolderPath, 'TrueState60fea.jpg'), '-jpg', '-r300', figure(2));
elseif ec == 3
    export_fig(fullfile(figuresFolderPath, 'TrueState100fea.jpg'), '-jpg', '-r300', figure(3));
end