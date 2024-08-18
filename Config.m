%% Check Switch
measurementsCheck = 1; % if 1, check the measurements by running MeasurementsGeneration.m
realTimeCheck = 1; % if 1, check the real-time state at each step after by running EKF_SLAM_simulation.m
CovCheck = 1; % if 1, check the confidence interval ellipse at each step

%% Noise Switch
R1addPose0Noise = 0; % if 0, R1 initial position is accurate
R2addPose0Noise = 1; % if 0, R2 initial position is accurate

R1addOdoNoise = 1; % if 0, R1 odometry is perfect
R2addOdoNoise = 1; % if 0, R2 odometry is perfect

R1addObsNoise = 1; % if 0, R1 observation is perfect
R2addObsNoise = 1; % if 0, R2 observation is perfect


%% Sensor Range
R1sensorRange = 8; % m/s, R1's observation range for the features
R2sensorRange = 8; % m/s, R2's observation range for the features

%% Required number of shared feature observations at step 0 
reqSharedObsNum = 8; % Required number of shared feature observations

%% Robot bearing ranges
R1bearingRange = pi/60; % pi/60: 3°
R2bearingRange = pi/60; % pi/60: 3°

%% Gaussian noise level settings
%% R1
% standard deviation of 1th robot position at time 0 (equals to 0 as the origin of the 1D coordinate system)
R1sigma_0r = 0.01; % 0.01: 0.01 m
R1sigma_0phi = pi/180; % pi/180: 1°


% standard deviation of the zero mean Gaussian process noise w(k) of 1th robot
R1sigma_uv = 0.025; % 0.025 m/time_step
R1sigma_uw = pi/360; % 0.5°/time_step


% standard deviation of the zero mean Gaussian observation noise v(k) of 1th robot
R1sigma_zv = 0.05; % 0.05 m/time_step

%% R2
% standard deviation of robot position at time 0 (equals to 0 as the origin of the 1D coordinate system)
R2sigma_0r = 0.02; % 0.01: 0.01 m
R2sigma_0phi = pi/90; % pi/90: 2°


% standard deviation of the zero mean Gaussian process noise w(k) of 2nd robot
R2sigma_uv = 0.05; % 0.025 m/time_step
R2sigma_uw = pi/180; % 1°/time_step


% standard deviation of the zero mean Gaussian observation noise v(k) of 2nd robot
R2sigma_zv = 0.1;



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
lightGreen = [144, 238, 144] / 255; % 浅绿色
cyanGreen = [0, 255, 127] / 255;   % 青绿色
yellowGreen = [154, 205, 50] / 255; % 黄绿色
oliveGreen = [85, 107, 47] / 255;   % 橄榄绿色



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