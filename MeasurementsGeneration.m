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

%% Initial position Generation
R1Xp0 = pose0Generation(R1XrTrue,R1XphiT,R1addPose0Noise,R1O);
R2Xp0 = pose0Generation(R2XrTrue,R2XphiT,R2addPose0Noise,R2O);

%% Odometry measurements Generation
R1Odo = odometryGeneration(R1XrTrue,R1XphiT,PosNum,R1addOdoNoise,R1Q);
R2Odo = odometryGeneration(R2XrTrue,R2XphiT,PosNum,R2addOdoNoise,R2Q);

%% Observation measurements Generation
R1Obs = observationGeneration(R1XrTrue,R1XphiT,PosNum,XfTrueAll,R1addObsNoise,R1R,R1sensorRange);
R2Obs = observationGeneration(R2XrTrue,R2XphiT,PosNum,XfTrueAll,R2addObsNoise,R2R,R2sensorRange);

%% Shared feature Observation at step 0
sharedObs0Num = numel(intersect(R1Obs(R1Obs(:,1)==0,2),R2Obs(R2Obs(:,1)==0,2)));
disp(['Number of shared feature observations at step 0: ',num2str(sharedObs0Num)])
if sharedObs0Num < reqSharedObsNum
    disp('Shared feature observations at step 0 not enough, need to adjust sensor range and rerun')
end

save('Parameters.mat','R1XrTrue','R1XphiT','R2XrTrue','R2XphiT','XfTrueAll')
save('Measurements.mat','R1Xp0','R1Odo','R1Obs','R2Xp0','R2Odo','R2Obs')

% Measurements Check
if measurementsCheck == 1
hold on
plot(R1XrTrue(1:2:(end-1),2),R1XrTrue(2:2:end,2),'-bo')
plot(R2XrTrue(1:2:(end-1),2),R2XrTrue(2:2:end,2),'-co')
plot(XfTrueAll(1:2:(end-1),2),XfTrueAll(2:2:end,2),'*','Color',darkGreen)
text(XfTrueAll(1:2:(end-1),2),XfTrueAll(2:2:end,2),num2str(XfTrueAll(2:2:end,1)),'Color',darkGreen)



R1Xf = [];
R2Xf = [];
for k = 0:(PosNum-1)
    if k == 0
        R1Xpk = R1Xp0;
        R1Xp = R1Xpk;
        R2Xpk = R2Xp0;
        R2Xp = R2Xpk;
    else
        R1Odok = R1Odo(R1Odo(:,2)==k,3);
        R1Xpk(1:2,1) = R1Xpk(1:2,1) + rotationMatrix(R1Xpk(3,1))*R1Odok(1:2,1);
        R1Xpk(3,1) = R1Xpk(3,1) + R1Odok(3,1);
        R1Xp = [R1Xp;R1Xpk];

        R2Odok = R2Odo(R2Odo(:,2)==k,3);
        R2Xpk(1:2,1) = R2Xpk(1:2,1) + rotationMatrix(R2Xpk(3,1))*R2Odok(1:2,1);
        R2Xpk(3,1) = R2Xpk(3,1) + R2Odok(3,1);
        R2Xp = [R2Xp;R2Xpk];
    end

    R1Zk = R1Obs(R1Obs(:,1)==k,2:3);
    for R1j = 1:(size(R1Zk,1)/2)
        R1Xfk = R1Xp(k*3+(1:2),1) + rotationMatrix(R1Xp(k*3+3,1)) * R1Zk((R1j-1)*2+(1:2),2);
        R1Xf = [R1Xf;R1Zk((R1j-1)*2+(1:2),1),R1Xfk];
    end

    R2Zk = R2Obs(R2Obs(:,1)==k,2:3);
    for R2j = 1:(size(R2Zk,1)/2)
        R2Xfk = R2Xp(k*3+(1:2),1) + rotationMatrix(R2Xp(k*3+3,1)) * R2Zk((R2j-1)*2+(1:2),2);
        R2Xf = [R2Xf;R2Zk((R2j-1)*2+(1:2),1),R2Xfk];
    end

end
plot(R1Xp(1:3:(end-2),1),R1Xp(2:3:(end-1),1),'--ro')
plot(R2Xp(1:3:(end-2),1),R2Xp(2:3:(end-1),1),'--mo')

% plot(R1Xf(1:2:(end-1),2),R1Xf(2:2:end,2),'*','Color',brightGreen)
% text(R1Xf(1:2:(end-1),2),R1Xf(2:2:end,2),num2str(R1Xf(2:2:end,1)),"Color",brightGreen)
% 
% plot(R2Xf(1:2:(end-1),2),R2Xf(2:2:end,2),'*','Color',paleGreen)
% text(R2Xf(1:2:(end-1),2),R2Xf(2:2:end,2),num2str(R2Xf(2:2:end,1)),"Color",paleGreen)
xlim([fea_xlb,fea_xub])
ylim([fea_ylb,fea_yub])
hold off
end