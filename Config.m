fea_xlb = -50; % lower boundary of features in x axis
fea_xub = 50; % upper boundary of features in x axis
fea_ylb = -50; % lower boundary of features in x axis
fea_yub = 50; % upper boundary of features in x axis

CC = 1e-9; % CC: converge coefficient of Gauss-Newton iteration

%% Robot bearing ranges
R1bearingRange = pi/60; % pi/60: 3°
R2bearingRange = pi/60; % pi/60: 3°

%% Gaussian noise level settings
% standard deviation of 1th robot position at time 0 (equals to 0 as the origin of the 1D coordinate system)
R1sigma_0r = 0.01; % 0.01: 0.01 m
R1sigma_0phi = pi/180; % pi/180: 1°
R1O = [R1sigma_0r^2,0,0;
    0,R1sigma_0r^2,0;
    0,0,R1sigma_0phi^2];

% standard deviation of the zero mean Gaussian process noise w(k) of 1th robot
R1sigma_uv = 0.025; % 0.025 m/time_step
R1sigma_uw = pi/360; % 0.5°/time_step
R1Q = [R1sigma_uv^2,0,0;
    0,R1sigma_uv^2,0;
    0,0,R1sigma_uw^2];

% standard deviation of the zero mean Gaussian observation noise v(k) of 1th robot
R1sigma_zv = 0.05; % 0.05 m/time_step
R1R = [R1sigma_zv^2,0;
    0,R1sigma_zv^2];
% R1sigma_zw = pi/180; % 1°/time_step


% standard deviation of robot position at time 0 (equals to 0 as the origin of the 1D coordinate system)
R2sigma_0r = 0.01; % 0.01: 0.01 m
R2sigma_0phi = pi/36; % pi/36: 5°
R2O = [R2sigma_0r^2,0,0;
    0,R2sigma_0r^2,0;
    0,0,R2sigma_0phi^2];

% standard deviation of the zero mean Gaussian process noise w(k) of 2nd robot
R2sigma_uv = 0.025; % 0.025 m/time_step
R2sigma_uw = pi/180; % 1°/time_step
R2Q = [R2sigma_uv^2,0,0;
    0,R2sigma_uv^2,0;
    0,0,R2sigma_uw^2];

% standard deviation of the zero mean Gaussian observation noise v(k) of 2nd robot
R2sigma_zv = 0.05;
R2R = [R2sigma_zv^2,0;
    0,R2sigma_zv^2];

%% Switch to change
R1addOdoNoise = 1; % if 0, R1 odometry is perfect
R2addOdoNoise = 0; % if 0, R2 odometry is perfect

R1addObsNoise = 1; % if 0, R1 observation is perfect
R2addObsNoise = 1; % if 0, R2 observation is perfect

R1addPose0Noise = 0; % if 0, R1 initial position is accurate
R2addPose0Noise = 0; % if 0, R2 initial position is accurate

R1sensorRange = 8; % m/s, R1's observation range for the features
R2sensorRange = 8; % m/s, R2's observation range for the features

reqSharedObsNum = 8; % Required number of shared feature observations