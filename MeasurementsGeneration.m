clc
close all

load('TrueState\RobotsWaypoints.mat')
load('TrueState\CheckedTrueState.mat')

R1XrTrue = dataModify(fliplr(wp1));
R2XrTrue = dataModify(wp2);
XfTrueAll = dataModify(lm);
XfTrueAll(:,1)=XfTrueAll(:,1)+1;

% hold on
% plot(R1XrTrue(1:2:(end-1),2),R1XrTrue(2:2:end,2),'-bo')
% plot(R2XrTrue(1:2:(end-1),2),R2XrTrue(2:2:end,2),'-ro')
% plot(XfTrueAll(1:2:(end-1),2),XfTrueAll(2:2:end,2),'g*')
% text(XfTrueAll(1:2:(end-1),2),XfTrueAll(2:2:end,2),num2str(XfTrueAll(2:2:end,1)))
% xlim([-50,50])
% ylim([-50,50])
% hold off

R1PosNum = size(R1XrTrue,1)/2;
R2PosNum = size(R2XrTrue,1)/2;

Config;



%% Robot Orientation Generation
R1XphiT = bearingGeneration(R1XrTrue,R1bearingRange);
R2XphiT = bearingGeneration(R2XrTrue,R2bearingRange);

%% Initial position Generation
R1Xp0 = pose0Generation(R1XrTrue,R1XphiT,R1addPose0Noise,R1O);
R2Xp0 = pose0Generation(R2XrTrue,R2XphiT,R2addPose0Noise,R2O);

%% Odometry measurements Generation
R1Odo = odometryGeneration(R1XrTrue,R1XphiT,R1PosNum,R1addOdoNoise,R1Q);
R2Odo = odometryGeneration(R2XrTrue,R2XphiT,R2PosNum,R2addOdoNoise,R2Q);

%% Observation measurements Generation
R1Obs = observationGeneration(R1XrTrue,R1XphiT,R1PosNum,XfTrueAll,R1addObsNoise,R1R,R1sensorRange);
R2Obs = observationGeneration(R2XrTrue,R2XphiT,R2PosNum,XfTrueAll,R2addObsNoise,R2R,R2sensorRange);

%% Shared feature Observation at step 0
sharedObs0Num = numel(intersect(R1Obs(R1Obs(:,1)==0,2),R2Obs(R2Obs(:,1)==0,2)));
disp(['Number of shared feature observations at step 0: ',num2str(sharedObs0Num)])
if sharedObs0Num < reqSharedObsNum
    disp('Shared feature observations at step 0 not enough, need to adjust sensor range and rerun')
end

save('Parameters.mat','R1XrTrue','R1XphiT','R2XrTrue','R2XphiT','XfTrueAll')
save('Measurements.mat','R1Xp0','R1Odo','R1Obs','R2Xp0','R2Odo','R2Obs')