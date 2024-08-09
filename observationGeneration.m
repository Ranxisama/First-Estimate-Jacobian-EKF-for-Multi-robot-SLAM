function obs = observationGeneration(XrTrue,odo_phiT,PosNum,XfTrueAll,addObsNoise,R,sensorRange)
if addObsNoise == 0
    R = 0;
end

obs = [];
for k = 0:PosNum-1
    for j = 1:size(XfTrueAll,1)/2
        % Euclidean distance from the feature to the robot
        DeltaDist = XfTrueAll((j-1)*2+(1:2),2)-XrTrue(XrTrue(:,1)==k);
        % FeaRobDis: distance from the feature to the robot
        FeaRobDist = sqrt(sum(DeltaDist.^2));
        if FeaRobDist <= sensorRange
            Rot = rotationMatrix(odo_phiT(k+1));
            obs(end+1:end+2,1:3) = [[k;k],XfTrueAll((j-1)*2+(1:2),1),Rot'*DeltaDist+R*randn(2,1)]; 
        end
    end
end