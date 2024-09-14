clc
close all

load Zstate_VicPark_6898_loops
Odo = Zstate(Zstate(:,2)==1,[4,3,1]);
poseNum = size(Odo,1)/3;
Obs = Zstate(Zstate(:,2)==2,[4,3,1]);
K = [];

%% find pose with more than 8 shared features
for k = 0:(poseNum-2)
    Obs1 = Obs(Obs(:,1)==k,:);
    Obs2 = Obs(Obs(:,1)==k+1,:);

    Sharedfeatures = unique(intersect(Obs1(:,2),Obs2(:,2)));
    SharedfeaNum = numel(Sharedfeatures);

    if SharedfeaNum >= 8
        K = [K,k];
    end
end

%% robot pose and feature position computation
Waypoints = zeros(3,2);
Phi = zeros(1,2);
Landmarks = [];
for k = 1:poseNum
    Phi_k = Phi(k,2);
    Odo_k = Odo(Odo(:,2) == k,:);

    Xp_k = Waypoints((k-1)*3+(1:2),:);
    DPhi_k = Waypoints((k-1)*3+3,:);

    Waypoints = [Waypoints; ...
        Odo_k(1:2,2),Xp_k(:,2)+Rot(DPhi_k(1,2))*Odo_k(1:2,3); ...
        Odo_k(3,2),DPhi_k+Odo_k(3,3)];
end