% Monte Carlo Measurements Generation

clc
close all

Config;

load('TrueState\RobotsWaypoints.mat')

wp1 = repmat(wp1,1,cy);
wp2 = repmat(wp2,1,cy);

R1XrTrueAll = dataModify(wp1);
R2XrTrueAll = dataModify(wp2);

R1PosNum = size(R1XrTrueAll,1)/2;
R2PosNum = size(R2XrTrueAll,1)/2;

PosNum = min(R1PosNum,R2PosNum);

R1XrTrue = R1XrTrueAll(1:(2*PosNum),:);
R2XrTrue = R2XrTrueAll(1:(2*PosNum),:);

%% Robot Orientation Generation
R1XphiT = bearingGeneration(R1XrTrue,R1bearingRange);
R2XphiT = bearingGeneration(R2XrTrue,R2bearingRange);

R1Xp0Set = [];
R2Xp0Set = [];
R1OdoSet = [];
R2OdoSet = [];

% True odometry
R1OdoT = odometryGeneration(R1XrTrue,R1XphiT,PosNum,0,R1Q);
R2OdoT = odometryGeneration(R2XrTrue,R2XphiT,PosNum,0,R2Q);

for mc = 1:mcNum
    %% Initial position Generation
    R1Xp0 = pose0Generation(R1XrTrue,R1XphiT,R1addPose0Noise,R1O);
    R2Xp0 = pose0Generation(R2XrTrue,R2XphiT,R2addPose0Noise,R2O);
    R1Xp0Set = [R1Xp0Set,R1Xp0];
    R2Xp0Set = [R2Xp0Set,R2Xp0];

    %% Odometry measurements Generation
    R1Odo = odometryGeneration(R1XrTrue,R1XphiT,PosNum,R1addOdoNoise,R1Q);
    R2Odo = odometryGeneration(R2XrTrue,R2XphiT,PosNum,R2addOdoNoise,R2Q);
    R1OdoSet = [R1OdoSet,R1Odo(:,3)];
    R2OdoSet = [R2OdoSet,R2Odo(:,3)];
end

R1OdoSet = [R1Odo(:,1:2),R1OdoSet];
R2OdoSet = [R2Odo(:,1:2),R2OdoSet];

for i = 1:3
    R1ObsSet = [];
    R2ObsSet = [];
    XfTrueAll = [];
    if i == 1
        load('TrueState\CheckedTrueState_20features.mat')
    elseif i == 2
        load('TrueState\CheckedTrueState_60features.mat')
    else
        load('TrueState\CheckedTrueState_100features.mat')
    end
    XfTrueAll = dataModify(lm);
    XfTrueAll(:,1)=XfTrueAll(:,1)+1;
    %% True observation
    R1ObsT = observationGeneration(R1XrTrue,R1XphiT,PosNum,XfTrueAll,0,R1R,R1sensorRange);
    R2ObsT = observationGeneration(R2XrTrue,R2XphiT,PosNum,XfTrueAll,0,R2R,R2sensorRange);
    for mc = 1:mcNum
        %% Observation measurements Generation
        R1Obs = observationGeneration(R1XrTrue,R1XphiT,PosNum,XfTrueAll,R1addObsNoise,R1R,R1sensorRange);
        R2Obs = observationGeneration(R2XrTrue,R2XphiT,PosNum,XfTrueAll,R2addObsNoise,R2R,R2sensorRange);
        R1ObsSet = [R1ObsSet,R1Obs(:,3)];
        R2ObsSet = [R2ObsSet,R2Obs(:,3)];
    end

    R1ObsSet = [R1Obs(:,1:2),R1ObsSet];
    R2ObsSet = [R2Obs(:,1:2),R2ObsSet];
        
    feaNum = numel(unique([R1ObsSet(:,2);R2ObsSet(:,2)]));



    %% Shared feature Observation at step 0
    if i == 1
    sharedObs0Num = numel(intersect(R1Obs(R1Obs(:,1)==0,2),R2Obs(R2Obs(:,1)==0,2)));
    disp(['Number of shared feature observations at step 0: ',num2str(sharedObs0Num)])
    if sharedObs0Num < reqSharedObsNum
        error('Shared feature observations at step 0 not enough, need to adjust sensor range and rerun')
    end
    end

    if i == 1
        disp(['Number of features: ',num2str(feaNum)])
        save('MT_Parameters_20fea.mat', ...
            'R1XrTrue','R1XphiT','R2XrTrue','R2XphiT','XfTrueAll','R1OdoT','R2OdoT','R1ObsT','R2ObsT')
        save('MT_Measurements_20fea.mat','R1Xp0Set','R1OdoSet','R1ObsSet','R2Xp0Set','R2OdoSet','R2ObsSet')
    elseif i == 2
        disp(['Number of features: ',num2str(feaNum)])
        save('MT_Parameters_60fea.mat', ...
            'R1XrTrue','R1XphiT','R2XrTrue','R2XphiT','XfTrueAll','R1OdoT','R2OdoT','R1ObsT','R2ObsT')
        save('MT_Measurements_60fea.mat','R1Xp0Set','R1OdoSet','R1ObsSet','R2Xp0Set','R2OdoSet','R2ObsSet')
    else
        disp(['Number of features: ',num2str(feaNum)])
        save('MT_Parameters_100fea.mat', ...
            'R1XrTrue','R1XphiT','R2XrTrue','R2XphiT','XfTrueAll','R1OdoT','R2OdoT','R1ObsT','R2ObsT')
        save('MT_Measurements_100fea.mat','R1Xp0Set','R1OdoSet','R1ObsSet','R2Xp0Set','R2OdoSet','R2ObsSet')
    end
end