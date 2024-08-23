clc
close all

load('TrueState\RobotsWaypoints.mat')
load('TrueState\CheckedTrueState.mat')

R1XrTrueAll = dataModify(fliplr(wp1));
R2XrTrueAll = dataModify(wp2);
XfTrueAll = dataModify(lm);
XfTrueAll(:,1)=XfTrueAll(:,1)+1;

R1PosNum = size(R1XrTrueAll,1)/2;
R2PosNum = size(R2XrTrueAll,1)/2;

PosNum = min(R1PosNum,R2PosNum);

R1XrTrue = R1XrTrueAll(1:(2*PosNum),:);
R2XrTrue = R2XrTrueAll(1:(2*PosNum),:);

Config;


%% Robot Orientation Generation
R1XphiT = bearingGeneration(R1XrTrue,R1bearingRange);
R2XphiT = bearingGeneration(R2XrTrue,R2bearingRange);

R1Xp0Set = [];
R2Xp0Set = [];
R1OdoSet = [];
R2OdoSet = [];
R1ObsSet = [];
R2ObsSet = [];

for mc = 1:mcNum
%% Initial position Generation
R1Xp0 = pose0Generation(R1XrTrue,R1XphiT,R1addPose0Noise,R1O);
R2Xp0 = pose0Generation(R2XrTrue,R2XphiT,R2addPose0Noise,R2O);
R1Xp0Set = [R1Xp0Set,R1Xp0];
R2Xp0Set = [R2Xp0Set,R2Xp0];

%% Odometry measurements Generation
R1Odo = odometryGeneration(R1XrTrue,R1XphiT,PosNum,R1addOdoNoise,R1Q);
R2Odo = odometryGeneration(R2XrTrue,R2XphiT,PosNum,R2addOdoNoise,R2Q);
R1OdoSet = [R1OdoSet,R1Odo];
R2OdoSet = [R2OdoSet,R2Odo];

%% Observation measurements Generation
R1Obs = observationGeneration(R1XrTrue,R1XphiT,PosNum,XfTrueAll,R1addObsNoise,R1R,R1sensorRange);
R2Obs = observationGeneration(R2XrTrue,R2XphiT,PosNum,XfTrueAll,R2addObsNoise,R2R,R2sensorRange);
R1ObsSet = [R1ObsSet,R1Obs];
R2ObsSet = [R2ObsSet,R2Obs];
end

%% Shared feature Observation at step 0
sharedObs0Num = numel(intersect(R1Obs(R1Obs(:,1)==0,2),R2Obs(R2Obs(:,1)==0,2)));
disp(['Number of shared feature observations at step 0: ',num2str(sharedObs0Num)])
if sharedObs0Num < reqSharedObsNum
    disp('Shared feature observations at step 0 not enough, need to adjust sensor range and rerun')
end

save('Parameters.mat','R1XrTrue','R1XphiT','R2XrTrue','R2XphiT','XfTrueAll')
save('Measurements.mat','R1Xp0Set','R1OdoSet','R1ObsSet','R2Xp0Set','R2OdoSet','R2ObsSet')