function [XfRMSE,XfRMSE_mean] = XFRMSE(featureNumber,MonteCarloExpNumber,DeltaXfFullSet)
XfRMSE = [DeltaXfFullSet(1:2:(end-1),1),zeros(featureNumber,1)];
XfRMSE(:,2) =  sqrt((sum(DeltaXfFullSet(1:2:(end-1),2:end).^2,2)+sum(DeltaXfFullSet(2:2:end,2:end).^2,2))/MonteCarloExpNumber);
XfRMSE_mean = mean(XfRMSE(:,2));
end