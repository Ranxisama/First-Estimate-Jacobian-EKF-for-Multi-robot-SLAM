function [XrRMSE,XrRMSE_mean] = XrRMSE(poseNumber,MonteCarloExpNumber,DeltaXrFullSet)
XrRMSE = [(0:poseNumber)',zeros(poseNumber+1,1)];
XrRMSE(:,2) =  sqrt((sum(DeltaXrFullSet(1:2:(end-1),:).^2,2)+sum(DeltaXrFullSet(2:2:end,:).^2,2))/MonteCarloExpNumber);
XrRMSE_mean = mean(XrRMSE(:,2));
end