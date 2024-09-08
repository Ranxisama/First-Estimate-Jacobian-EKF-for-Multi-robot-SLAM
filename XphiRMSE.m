function [XphiRMSE,XphiRMSE_mean] = XphiRMSE(poseNumber,MonteCarloExpNumber,DeltaXphiFullSet)
XphiRMSE = [(0:poseNumber)',zeros(poseNumber+1,1)];
XphiRMSE(:,2) = sqrt(sum(DeltaXphiFullSet(:,2:end).^2,2)/MonteCarloExpNumber);
XphiRMSE_mean = mean(XphiRMSE(:,2));
end