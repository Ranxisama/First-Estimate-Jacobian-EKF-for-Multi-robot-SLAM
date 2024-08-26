%% Number of Monte Carlo experiments
mcNum = 100;

%% Check Switch
%% Measurements check
measurementsCheck = 0; % if 1, check the measurements by running MeasurementsGeneration.m

%% Simulation check
% real time check, use more computer performance by running EKF_SLAM_simulation.m
% Cov check
poseCovCheck =  1; % if 1, check the pose Cov of R1 and R2
feaCovCheck = 0; % if 1, check the features Cov

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

%% MarkerSize
MS = 2;

%% Robot bearing ranges
R1bearingRange = pi/60; % pi/60: 3°
R2bearingRange = pi/60; % pi/60: 3°

%% Gaussian noise level settings
%% R1
% standard deviation of 1th robot position at time 0 (equals to 0 as the origin of the 1D coordinate system)
R1sigma_0r = 0.02; % 0.01: 0.01 m
R1sigma_0phi = pi/90; % pi/180: 1°


% standard deviation of the zero mean Gaussian process noise w(k) of 1th robot
R1sigma_uv = 0.05; % 0.025 m/time_step
R1sigma_uw = pi/180; % 0.5°/time_step


% standard deviation of the zero mean Gaussian observation noise v(k) of 1th robot
R1sigma_zv = 0.1; % m/time_step

%% R2
% standard deviation of robot position at time 0 (equals to 0 as the origin of the 1D coordinate system)
R2sigma_0r = 0.04; % m
R2sigma_0phi = pi/45; % 0.01*pi: 1.8°


% standard deviation of the zero mean Gaussian process noise w(k) of 2nd robot
R2sigma_uv = 0.1; % m/time_step
R2sigma_uw = pi/90; % °/time_step


% standard deviation of the zero mean Gaussian observation noise v(k) of 2nd robot
R2sigma_zv = 0.2; % m/time_step



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

if poseCovCheck ==  0 & feaCovCheck == 1
    error('To check feature Cov, you must switch on poseCovCheck')
end