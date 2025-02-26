function obs = observationGeneration(XrTrue,XphiTrue,PosNum,XfTrueAll,addObsNoise,R,sensorRange)
if addObsNoise == 0
    R(1,1) = 0;
    R(2,2) = 0;
end

obs = [];
for k = 0:PosNum
    XrkT = XrTrue(XrTrue(:,1)==k,2);
    XphikT = XphiTrue(XphiTrue(:,1)==k,2);
    for j = 1:size(XfTrueAll,1)/2
        XfjT = XfTrueAll((j-1)*2+(1:2),2);
        % Euclidean distance from the feature to the robot
        DeltaDist = XfjT - XrkT;
        % FeaRobDis: distance from the feature to the robot
        FeaRobDist = sqrt(sum(DeltaDist.^2));
        if FeaRobDist <= sensorRange
            ROT = Rot(XphikT);
            obs((end+1):(end+2),1:3) = [[k;k],XfTrueAll((j-1)*2+(1:2),1), ...
                ROT'*DeltaDist+sqrt(R)*Randn(2,1)]; 
        end
    end
end