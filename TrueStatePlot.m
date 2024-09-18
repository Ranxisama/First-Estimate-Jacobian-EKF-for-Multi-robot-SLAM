clc
close all
clear

Config;

load('MT_Parameters_20fea.mat', ...
    'R1XrTrue','R1XphiT','R2XrTrue','R2XphiT','XfTrueAll')

figure(1)
hold on
grid on
axis([fea_xlb,fea_xub,fea_ylb,fea_yub])
R1WaypointsT = plot(R1XrTrue(1:2:(end-1),2),R1XrTrue(2:2:end,2),'bo','MarkerSize',2);
R2WaypointsT = plot(R2XrTrue(1:2:(end-1),2),R2XrTrue(2:2:end,2),'rs','MarkerSize',2.5);

% arrow length
R1AL = 0.5*sqrt((R1XrTrue(3:2:(end-1),2) - R1XrTrue(1:2:(end-3),2)).^2 + (R1XrTrue(4:2:end,2) - R1XrTrue(2:2:(end-2),2)).^2);
R1AL(end+1,:) = R1AL(end,:);

R2AL = 0.5*sqrt((R2XrTrue(3:2:(end-1),2) - R2XrTrue(1:2:(end-3),2)).^2 + (R2XrTrue(4:2:end,2) - R2XrTrue(2:2:(end-2),2)).^2);
R2AL(end+1,:) = R2AL(end,:);

R1dx = R1AL.* cos(R1XphiT(:,2));
R1dy = R1AL.* sin(R1XphiT(:,2));
R2dx = R2AL.* cos(R2XphiT(:,2));
R2dy = R2AL.* sin(R2XphiT(:,2));

quiver(R1XrTrue(1:2:(end-1),2),R1XrTrue(2:2:end,2),R1dx,R1dy,0,'b','LineWidth',0.5)
quiver(R2XrTrue(1:2:(end-1),2),R2XrTrue(2:2:end,2),R2dx,R2dy,0,'-.r','LineWidth',0.5)
% quiver(R1XrTrue(1:2:(end-1),2),R1XrTrue(2:2:end,2),R1dx,R1dy,0,'b', 'LineWidth', 2)
% quiver(R2XrTrue(1:2:(end-1),2),R2XrTrue(2:2:end,2),R2dx,R2dy,0,'r', 'LineWidth', 2)

plot(XfTrueAll(1:2:(end-1),2),XfTrueAll(2:2:end,2),'g*','MarkerSize',2)

% 创建自定义图例
hLegend = legend('R_1 true position', 'R_2 true position', 'R_1 true heading', 'R_2 true heading', 'True feature', 'Location', 'northeast','FontSize', 3);
xlabel('x (m)')
ylabel('y (m)')

set(gcf, 'Color', 'w');  % 将整个图背景设置为白色
set(gca, 'Box', 'on', 'LineWidth', 1, 'GridLineStyle', '--', 'GridAlpha', 0.1);  % 使边框显示，并增加边框宽度
hold off

export_fig(fullfile(pwd, 'TrueState20fea.jpg'), '-jpg', '-r300', figure(1));