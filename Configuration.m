clear
close all
clc
% number of steps and features
num_of_steps = 50;
num_of_circles = 3;
num_of_feas = 300;

fea_xlb = -25; % lower boundary of features in x axis
fea_xub = 25; % upper boundary of features in x axis
fea_ylb = -25; % lower boundary of features in x axis
fea_yub = 25; % upper boundary of features in x axis




Dfr_threshold = 5; % Distance threshold from jth feature position to kth robot posture

ReqSFeaNum = 8; % Required number of shared features at step 0

%% Gaussian noise level settings
% standard deviation of 1th robot position at time 0 (equals to 0 as the origin of the 1D coordinate system)
sigma_0v_R1 = 0; % 0.01: 0.01 m/time_step
sigma_0w_R1 = 0; % pi/360: 0.5°/time_step
% standard deviation of the zero mean Gaussian process noise w(k) of 1th robot
sigma_uv_R1 = 0.025; % 0.025 m/time_step
sigma_uw_R1 = pi/180; % 1°/time_step
% standard deviation of the zero mean Gaussian observation noise v(k) of 1th robot
sigmaz_R1 = 0.05;

% standard deviation of robot position at time 0 (equals to 0 as the origin of the 1D coordinate system)
sigma_0v_R2 = 0.005; % 0.005: 0.005 m/time_step
sigma_0w_R2 = pi/720; % pi/720: 0.25°/time_step
% standard deviation of the zero mean Gaussian process noise w(k) of 2nd robot
sigma_uv_R2 = 0.025; % 0.025 m/time_step
sigma_uw_R2 = pi/180; % 1°/time_step
% standard deviation of the zero mean Gaussian observation noise v(k) of 2nd robot
sigmaz_R2 = 0.05;

%% Set the true trajectories of two robots R1 and R2 as two circles,
% R1's center is (cx1, cy1), radius is r1, Radian of the starting point is initRad1;
cx1 = fea_xlb+0.75*(fea_xub-fea_xlb);
% cy1 = fea_ylb+0.5*(fea_yub-fea_ylb);
cy1 = 0;
r1 = 0.01*(fea_yub-fea_ylb);
initRad1 = pi;
% R2's center is (cx2, cy2), radius is r2, Radian of the starting point is initRad2.
cx2 = fea_xlb+0.25*(fea_xub-fea_xlb);
% cy2 = fea_ylb+0.5*(fea_yub-fea_ylb);
cy2 = 0;
r2 = 0.01*(fea_yub-fea_ylb);
initRad2 = 0;

%% the ground truth of robot pose at each time step (1st row: time step; 2nd~4th row: position and orientation)
t = linspace(0, num_of_circles*2*pi, num_of_steps)';
Xr_true_R1 = [cx1 + r1*t.*cos(initRad1-t), cy1 + r1*t.*sin(initRad1-t)];
Xphi_true_R1 = wrap(pi/2 - t);
Xp_true_R1 = [(0:num_of_steps-1)', Xr_true_R1, Xphi_true_R1];

Xr_true_R2 = [cx2 + r2*t.*cos(initRad2+t), cy2 + r2*t.*sin(initRad2+t)];
Xphi_true_R2 = wrap(pi/2 + t);
Xp_true_R2 = [(0:num_of_steps-1)', Xr_true_R2, Xphi_true_R2];

hold on
plot(Xr_true_R1(:,1),Xr_true_R1(:,2),'-bo')
plot(Xr_true_R2(:,1),Xr_true_R2(:,2),'-co')
xlim([fea_xlb, fea_xub])
ylim([fea_ylb, fea_yub])
axis equal
ax = gca;
% 设置坐标轴属性，使其穿过原点
ax.XAxisLocation = 'origin'; % x 轴穿过原点
ax.YAxisLocation = 'origin'; % y 轴穿过原点
hold off

%% Generate data
% Xr0: initial robot posture with uncertainty
% U: control input with process noise
% Z: observation with observation noise

sharedFeaNum = 0;
while sharedFeaNum < ReqSFeaNum

    %% the ground truth of feature position at each time step (1st row: feature id; 2nd~3rd row: position)
    Xf_true = [(1:num_of_feas)', fea_xlb+(fea_xub-fea_xlb)*rand(num_of_feas,1), fea_ylb+(fea_yub-fea_ylb)*rand(num_of_feas,1)];

    %% R1: 1th robot
    Xp0_true_R1 = Xp_true_R1(1,2:4);
    % the estimated initial robot pose with uncertainty
    Xp0_R1 = Xp0_true_R1 + [sigma_0v_R1*randn(1,2), sigma_0w_R1*randn];

    U_true_R1 = []; % Odometry ground truth of control input U
    U_R1 = []; % Odometry measurements
    Z_true_R1 = []; % Observation ground truth
    Z_R1 = []; % Observation measurements

    for k = 0:num_of_steps-1
        firk_true_R1 = Xp_true_R1(Xp_true_R1(:,1)==k,4);
        R_k_R1 = [cos(firk_true_R1) -sin(firk_true_R1);
            sin(firk_true_R1) cos(firk_true_R1)]; % R_k: Rotation matrix
        if k ~= num_of_steps-1
            U_true_R1(3*k+(1:3),1:2) = repmat([k, k+1],3,1);
            U_true_R1(3*k+(1:2),3) = R_k_R1' * (Xp_true_R1(Xp_true_R1(:,1)==k+1, 2:3) - Xp_true_R1(Xp_true_R1(:,1)==k, 2:3))';
            U_true_R1(3*k+3,3) = Xp_true_R1(Xp_true_R1(:,1)==k+1, 4) - Xp_true_R1(Xp_true_R1(:,1)==k, 4)';
            
            U_R1(3*k+(1:3),1:2) = repmat([k, k+1],3,1);
            U_R1(3*k+(1:2),3) = R_k_R1' * ((Xp_true_R1(Xp_true_R1(:,1)==k+1, 2:3) - Xp_true_R1(Xp_true_R1(:,1)==k, 2:3))' + (sigma_uv_R1*randn(1,2))');
            U_R1(3*k+3,3) = U_true_R1(3*k+3,3) + sigma_uw_R1*randn;
        end

        for j = 1:num_of_feas
            % distance from jth true feature position to kth true robot posture
            Dfr_R1 = sqrt(sum((Xf_true(j,2:3) - Xp_true_R1(Xp_true_R1(:,1)==k,2:3)).^2));
            if Dfr_R1 <= Dfr_threshold
                Z_true_R1(end+1:end+2,1:2) = repmat([Xp_true_R1(Xp_true_R1(:,1)==k,1), Xf_true(j,1)],2,1); % feature & robot pose IDs
                Z_true_R1(end-1:end,3) = R_k_R1' * (Xf_true(j,2:3) - Xp_true_R1(Xp_true_R1(:,1)==k,2:3))';
                Z_R1(end+1:end+2,1:2) = repmat([Xp_true_R1(Xp_true_R1(:,1)==k,1), Xf_true(j,1)],2,1);
                Z_R1(end-1:end,3) = R_k_R1' *((Xf_true(j,2:3) - Xp_true_R1(Xp_true_R1(:,1)==k,2:3))' + sigmaz_R1*randn(2,1));
            end
        end
    end



    %% R2: 2nd robot
    Xp0_true_R2 = Xp_true_R2(1,2:4);
    % the estimated initial robot pose with uncertainty
    Xr0_R2 = Xp0_true_R2 + [sigma_0v_R2*randn(1,2) sigma_0w_R2*randn];

    U_true_R2 = []; % Odometry ground truth of control input U
    U_R2 = []; % Odometry measurements
    Z_true_R2 = []; % Observation ground truth
    Z_R2 = []; % Observation measurements

    for k = 0:num_of_steps-1
        firk_true_R2 = Xp_true_R2(Xp_true_R2(:,1)==k,4);
        R_k_R2 = [cos(firk_true_R2) -sin(firk_true_R2);
            sin(firk_true_R2) cos(firk_true_R2)]; % R_k: Rotation matrix
        if k ~= num_of_steps-1
            U_true_R2(3*k+(1:3),1:2) = repmat([k, k+1],3,1);
            U_true_R2(3*k+(1:2),3) = R_k_R2' * (Xp_true_R2(Xp_true_R2(:,1)==k+1, 2:3) - Xp_true_R2(Xp_true_R2(:,1)==k, 2:3))';
            U_true_R2(3*k+3,3) = Xp_true_R2(Xp_true_R2(:,1)==k+1, 4) - Xp_true_R2(Xp_true_R2(:,1)==k, 4)';
            
            U_R2(3*k+(1:3),1:2) = repmat([k, k+1],3,1);
            U_R2(3*k+(1:2),3) = R_k_R2' * ((Xp_true_R2(Xp_true_R2(:,1)==k+1, 2:3) - Xp_true_R2(Xp_true_R2(:,1)==k, 2:3))' + (sigma_uv_R2*randn(1,2))');
            U_R2(3*k+3,3) = U_true_R2(3*k+3,3) + sigma_uw_R2*randn;
        end

        for j = 1:num_of_feas
            % distance from jth true feature position to kth true robot posture
            Dfr_R2 = sqrt(sum((Xf_true(j,2:3) - Xp_true_R2(Xp_true_R2(:,1)==k,2:3)).^2));
            if Dfr_R2 <= Dfr_threshold
                Z_true_R2(end+1:end+2,1:2) = repmat([Xp_true_R2(Xp_true_R2(:,1)==k,1), Xf_true(j,1)],2,1); % feature & robot pose IDs
                Z_true_R2(end-1:end,3) = R_k_R2' * (Xf_true(j,2:3) - Xp_true_R2(Xp_true_R2(:,1)==k,2:3))';
                Z_R2(end+1:end+2,1:2) = repmat([Xp_true_R2(Xp_true_R2(:,1)==k,1), Xf_true(j,1)],2,1);
                Z_R2(end-1:end,3) = R_k_R2' *((Xf_true(j,2:3) - Xp_true_R2(Xp_true_R2(:,1)==k,2:3))' + sigmaz_R2*randn(2,1));
            end
        end
    end
    
    
    sharedFeas = intersect(Z_R1(Z_R1(:,1) == 0, 2), Z_R2(Z_R2(:,1) == 0, 2));
    sharedFeaNum = numel(sharedFeas);
    
end

sharedFeaNum

% %% Plot the ground truth of trajectories and landmarks
% figure(1)
% hold on
% plot(Xr_true_R1(:,2),Xr_true_R1(:,3),'bo')
% plot(Xr_true_R2(:,2),Xr_true_R2(:,3),'go')
% plot(Xf_true(:,2),Xf_true(:,3),'r^')
% legend('1th robot true poses', '2nd robot true poses', 'True feature positions')
% xlabel('x')
% xlabel('y')
% title('True value of X state')
% saveas(gcf,'trueX.png')
% hold off

dataWithNoise = 'generatedDataWithNoise';
dataWithNoisePath = fullfile(pwd, dataWithNoise);
if ~exist(dataWithNoisePath, 'dir')
    mkdir(dataWithNoisePath)
end

dataName = 'dataWithNoise.mat';
dataPath = fullfile(dataWithNoisePath, dataName);
save(dataPath, 'num_of_steps', ...
    'Xp0_R1', ...
    'Xr0_R2', ...
    'U_R1', ...
    'U_R2', ...
    'Z_R1', ...
    'Z_R2')

disp(['Data with noise saved to: ', dataPath]);

trueData = 'generatedTrueData';
trueData = fullfile(pwd, trueData);
if ~exist(trueData, 'dir')
    mkdir(trueData)
end

tDataName = 'trueData.mat';
tDataPath = fullfile(trueData, tDataName);
save(tDataPath, 'Xp0_true_R1', 'sigma_0v_R1', 'sigma_0w_R1', ...
    'Xp0_true_R2', 'sigma_0v_R2', 'sigma_0w_R2', ...
    'U_true_R1', 'sigma_uv_R1', 'sigma_uw_R1', ...
    'U_true_R2', 'sigma_uv_R2', 'sigma_uw_R2', ...
    'Z_true_R1','sigmaz_R1', ...
    'Z_true_R2','sigmaz_R2', ...
    'fea_xlb', 'fea_xub', 'fea_ylb', 'fea_yub', 'Dfr_threshold')

disp(['True data saved to: ', tDataPath]);