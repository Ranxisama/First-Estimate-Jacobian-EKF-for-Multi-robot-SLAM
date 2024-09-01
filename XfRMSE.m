function [XrRMSE,XrRMSE_mean] = XfRMSE(featureNumber,MonteCarloExpNumber,DeltaXfFullSet)
XrRMSE = [DeltaXfFullSet(1:2:(end-1),1),zeros(featureNumber,1)];
XrRMSE(:,2) =  sqrt((sum(DeltaXfFullSet(1:2:(end-1),:).^2,2)+sum(DeltaXfFullSet(2:2:end,:).^2,2))/MonteCarloExpNumber);
XrRMSE_mean = mean(XrRMSE(:,2));
end