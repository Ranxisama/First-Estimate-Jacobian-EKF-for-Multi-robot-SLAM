%% choose different experiment
% ec =
% 1, Simulating data with 20 features
% 2, Simulating data with 60 features
% 3, Simulating data with 100 features
% 4, Victoria Park dataset
ec = 1;

%% choose simulation environment
if ec ~= 4
    % env =
    % 1, environment 1: two intersecting circles
    % 2, environment 2: an '8' in a circle
    env = 1;
end

%% Simulation noise choice
snc = 0; % if 0, for test; if 1, for display;

%% Number of Monte Carlo experiments
mcNum = 1; % if 1, plot nornmal error and 3-sigma bound; if > 1, plot absolute value of error and 3-sigma bound

%% Number of robot running cycles
cy = 5;

%% Trajectory plot
TrajP = 1; % if 1, plot the trajectory

%% Jacobian value switch
% Ideal EKF
step0GNI_ide = 1; % if 0, Jacobian of GNI at step 0 use optimized value; if 1 (good), use true value
step0FI_ide = 1; % (does not make difference) % if 0, Jacobian of feature initialization at step 0 use optimized value; if 1, use true value

stepkFI_ide = 1; % if 0, Jacobian of feature initialization at step k (k~=0) use estimated value; if 1 (good), use true value

% FEJ EKF
step0GNI_fej = 0; % if 0, Jacobian of GNI at step 0 use optimized value; if 1, use first estimated value
step0FI_fej = 0; % (does not make difference) % if 0, Jacobian of feature initialization at step 0 use optimized value; if 1, use first estimated value

if ec ~= 4 % For Simulation
    %% Noise Switch
    R1addPose0Noise = 0; % if 0, R1 initial position is accurate
    R2addPose0Noise = 1; % if 0, R2 initial position is accurate

    R1addOdoNoise = 1; % if 0, R1 odometry is perfect
    R2addOdoNoise = 1; % if 0, R2 odometry is perfect

    R1addObsNoise = 1; % if 0, R1 observation is perfect
    R2addObsNoise = 1; % if 0, R2 observation is perfect

    %% Gaussian noise level settings
    if env == 1
        if snc == 0 % for test, feel free to change the noise level
            % standard deviation of the zero mean Gaussian process noise w(k) of 1th robot
            R1sigma_uv = 1; % 0.025 m/time_step
            R1sigma_uw = pi/18; % pi: 180°/time_step

            % standard deviation of the zero mean Gaussian process noise w(k) of 2nd robot
            R2sigma_uv = 1; % m/time_step
            R2sigma_uw = pi/18; % °/time_step

            % standard deviation of the zero mean Gaussian observation noise v(k) of 1th robot
            R1sigma_zv = 1; % m/time_step

            % standard deviation of the zero mean Gaussian observation noise v(k) of 2nd robot
            R2sigma_zv = 1; % m/time_step

            % R1
            % standard deviation of 1th robot position at time 0 (equals to 0 as the origin of the 1D coordinate system)
            R1sigma_0r = 0.5; % 0.01: 0.01 m
            R1sigma_0phi = pi/18; % pi/180: 1°/time_step

            % R2
            % standard deviation of robot position at time 0 (equals to 0 as the origin of the 1D coordinate system)
            R2sigma_0r = 0.5; % m
            R2sigma_0phi = pi/18; % pi: 180°

        elseif snc == 1 % for display (best)
            % standard deviation of the zero mean Gaussian process noise w(k) of 1th robot
            R1sigma_uv = 0.3; % 0.025 m/time_step
            R1sigma_uw = pi/45; % pi: 180°/time_step

            % standard deviation of the zero mean Gaussian process noise w(k) of 2nd robot
            R2sigma_uv = 0.3; % m/time_step
            R2sigma_uw = pi/45; % °/time_step

            % standard deviation of the zero mean Gaussian observation noise v(k) of 1th robot
            R1sigma_zv = 0.5; % m/time_step

            % standard deviation of the zero mean Gaussian observation noise v(k) of 2nd robot
            R2sigma_zv = 0.5; % m/time_step

            % R1
            % standard deviation of 1th robot position at time 0 (equals to 0 as the origin of the 1D coordinate system)
            R1sigma_0r = 0.2; % 0.01: 0.01 m
            R1sigma_0phi = 0.02; % pi/180: 1°/time_step

            % R2
            % standard deviation of robot position at time 0 (equals to 0 as the origin of the 1D coordinate system)
            R2sigma_0r = 1; % m
            R2sigma_0phi = 0.1; % pi: 180°

        else
            error('Simulation Noise Choice (snc) can onlt be 0 or 1')
        end
    elseif env == 2
        if snc == 0 % for test
            % R1
            % standard deviation of 1th robot position at time 0 (equals to 0 as the origin of the 1D coordinate system)
            R1sigma_0r = 0.2; % 0.01: 0.01 m
            R1sigma_0phi = 0.02; % pi/180: 1°/time_step

            % R2
            % standard deviation of robot position at time 0 (equals to 0 as the origin of the 1D coordinate system)
            R2sigma_0r = 1; % m
            R2sigma_0phi = 0.1; % pi: 180°


            % standard deviation of the zero mean Gaussian process noise w(k) of 1th robot
            R1sigma_uv = 0.2; % 0.025 m/time_step
            R1sigma_uw = pi/60; % pi: 180°/time_step

            % standard deviation of the zero mean Gaussian process noise w(k) of 2nd robot
            R2sigma_uv = 0.2; % m/time_step
            R2sigma_uw = pi/60; % °/time_step

            % standard deviation of the zero mean Gaussian observation noise v(k) of 1th robot
            R1sigma_zv = 0.6; % m/time_step

            % standard deviation of the zero mean Gaussian observation noise v(k) of 2nd robot
            R2sigma_zv = 0.6; % m/time_step

        elseif snc == 1 % for display (best)
            % R1
            % standard deviation of 1th robot position at time 0 (equals to 0 as the origin of the 1D coordinate system)
            R1sigma_0r = 0.2; % 0.01: 0.01 m
            R1sigma_0phi = 0.02; % pi/180: 1°/time_step

            % R2
            % standard deviation of robot position at time 0 (equals to 0 as the origin of the 1D coordinate system)
            R2sigma_0r = 1; % m
            R2sigma_0phi = 0.1; % pi: 180°


            % standard deviation of the zero mean Gaussian process noise w(k) of 1th robot
            R1sigma_uv = 0.2; % 0.025 m/time_step
            R1sigma_uw = pi/60; % pi: 180°/time_step

            % standard deviation of the zero mean Gaussian process noise w(k) of 2nd robot
            R2sigma_uv = 0.2; % m/time_step
            R2sigma_uw = pi/60; % °/time_step

            % standard deviation of the zero mean Gaussian observation noise v(k) of 1th robot
            R1sigma_zv = 0.6; % m/time_step

            % standard deviation of the zero mean Gaussian observation noise v(k) of 2nd robot
            R2sigma_zv = 0.6; % m/time_step
        else
            error('Simulation Noise Choice (snc) can onlt be 0 or 1')
        end
    end

else % For Victoria Park Dataset experiment
    %% Noise Switch
    R1addPose0Noise = 1; % if 0, R1 initial position is accurate
    R2addPose0Noise = 1; % if 0, R2 initial position is accurate

    R1addOdoNoise = 1; % if 0, R1 odometry is perfect
    R2addOdoNoise = 1; % if 0, R2 odometry is perfect

    R1addObsNoise = 1; % if 0, R1 observation is perfect
    R2addObsNoise = 1; % if 0, R2 observation is perfect

    %% Gaussian noise level settings
    % R1
    % standard deviation of 1th robot position at time 0 (equals to 0 as the origin of the 1D coordinate system)
    R1sigma_0r = 0.005; % 0.01: 0.01 m
    R1sigma_0phi = 0.005*pi; % pi/180: 1°


    % standard deviation of the zero mean Gaussian process noise w(k) of 1th robot
    R1sigma_uv = 0.075; % 0.025 m/time_step
    R1sigma_uw = 0.015*pi; % 0.5°/time_step

    % standard deviation of the zero mean Gaussian observation noise v(k) of 1th robot
    R1sigma_zv = 0.075; % m/time_step

    % R2
    % standard deviation of robot position at time 0 (equals to 0 as the origin of the 1D coordinate system)
    R2sigma_0r = 100; % m
    R2sigma_0phi = pi; % pi: 180°

    % standard deviation of the zero mean Gaussian process noise w(k) of 2nd robot
    R2sigma_uv = 0.075; % m/time_step
    R2sigma_uw = 0.015*pi; % °/time_step


    % standard deviation of the zero mean Gaussian observation noise v(k) of 2nd robot
    R2sigma_zv = 0.075; % m/time_step

end


%% Robot bearing ranges
% R1bearingRange = 0; % pi/60: 3°
% R2bearingRange = 0; % pi/60: 3°
R1bearingRange = pi/9; % pi/12: 15°
R2bearingRange = pi/9; % pi/12: 15°

%% Sensor Range
R1sensorRange = 15; % m/s, R1's observation range for the features
R2sensorRange = 15; % m/s, R2's observation range for the features

%% Required number of shared feature observations at step 0
reqSharedObsNum = 2; % Required number of shared feature observations

%% Check Switch
% Measurements check
measurementsCheck = 1; % if 1, check the measurements by running MeasurementsGeneration.m

% Simulation check
% real time check, use more computer performance by running EKF_SLAM_simulation.m
% Cov check
robPositionCovCheck =  0; % if 1, check the position Cov of R1 and R2
feaCovCheck = 0; % if 1, check the features Cov



%% MarkerSize
MS = 2;

%% Arrow length
al = 1;

%% LineWidth
LW = 2;

%% FontSize
FontSize = 3;

%% boundary of features
fea_xlb = -50; % lower boundary of features in x axis
fea_xub = 50; % upper boundary of features in x axis
fea_ylb = -50; % lower boundary of features in x axis
fea_yub = 50; % upper boundary of features in x axis

%% converge coefficient of Gauss-Newton iteration
CC = 1e-9;

%% confidence interval;
CI = 0.9973; % 0.9973: Confidence level of 99.73% corresponding to 3-sigma bound

%% Covariance threshold
CovT = 1e-16; % set the matrix's elements that are less than CovT to zero.
% This can be useful for dealing with numerical errors or avoiding unnecessary imaginary parts in calculations.

%% feature color
brightGreen = [0, 255, 0] / 255;
paleGreen = [152, 251, 152]/255;
darkGreen = [0, 100, 0] / 255;

%% RMSE & ANEES color
cadetBlue = [0.373, 0.620, 0.627]; % R1
darkMagenta =  [0.545, 0, 0.545]; % R2


if R1addPose0Noise == 0
    R1sigma_0r = 0;
    R1sigma_0phi = 0;
end
R1O = [R1sigma_0r^2,0,0;
    0,R1sigma_0r^2,0;
    0,0,R1sigma_0phi^2];

if R1addOdoNoise == 0
    R1sigma_uv = 0;
    R1sigma_uw = 0;
end
R1Q = [R1sigma_uv^2,0,0;
    0,R1sigma_uv^2,0;
    0,0,R1sigma_uw^2];

if R1addObsNoise == 0
    R1sigma_zv = 0;
end
R1R = [R1sigma_zv^2,0;
    0,R1sigma_zv^2];

if R2addPose0Noise == 0
    R2sigma_0r = 0;
    R2sigma_0phi = 0;
end
R2O = [R2sigma_0r^2,0,0;
    0,R2sigma_0r^2,0;
    0,0,R2sigma_0phi^2];

if R2addOdoNoise == 0
    R2sigma_uv = 0;
    R2sigma_uw = 0;
end
R2Q = [R2sigma_uv^2,0,0;
    0,R2sigma_uv^2,0;
    0,0,R2sigma_uw^2];

if R2addObsNoise == 0
    R2sigma_zv = 0;
end
R2R = [R2sigma_zv^2,0;
    0,R2sigma_zv^2];

if robPositionCovCheck ==  0 && feaCovCheck == 1
    error('To check feature Cov, you must switch on robPositionCovCheck')
end

J = [0,-1;1,0];

if step0GNI_ide ~= 0 && step0GNI_ide ~= 1
    error('step0GNI_ide only can be 0 or 1')
end
if step0FI_ide ~= 0 && step0FI_ide ~= 1
    error('step0FI_ide only can be 0 or 1')
end
if stepkFI_ide ~= 0 && stepkFI_ide ~= 1
    error('stepkFI_ide only can be 0 or 1')
end

if step0GNI_fej ~= 0 && step0GNI_fej ~= 1
    error('step0GNI_fej only can be 0 or 1')
end
if step0FI_fej ~= 0 && step0FI_fej ~= 1
    error('step0FI_fej only can be 0 or 1')
end

if ec ~= 4
    if env == 1
        xplot_ub = 350;
    elseif env == 2
        xplot_ub = 250;
    end
elseif ec == 4
    xplot_ub = 3000;
end

% if ec ~= 4
%     if env == 1
%         disp('----------- The simulation environment is of two intersecting circles -----------')
%     elseif env == 2
%         disp('----------- The simulation environment is of an 8 in a circle -----------')
%     else
%         error('----------- variable env can only be 1 or 2 -----------')
%     end
% end