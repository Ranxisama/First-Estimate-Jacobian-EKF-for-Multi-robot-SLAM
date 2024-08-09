function modData = dataModify(orgData)
%% modify data form
modData = [];
modData(:,1) = repelem(0:size(orgData,2)-1, 2);
modData(1:2:end,2) = orgData(1,:)';
modData(2:2:end,2) = orgData(2,:)';