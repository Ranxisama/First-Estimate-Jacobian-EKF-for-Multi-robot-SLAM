function Fx = F(Xs,Zks)
Fx = Zks;
feaNum = (size(Xs,1)-6)/2;
% For R1 shared feature observation
Fx(6+(1:2:(2*feaNum-1)),1) = (Xs(7:2:(end-1),1) - Xs(1,1))*cos(Xs(3,1)) + (Xs(8:2:end,1) - Xs(2,1))*sin(Xs(3,1));
Fx(6+(2:2:(2*feaNum)),1) = (Xs(7:2:(end-1),1) - Xs(1,1))*-sin(Xs(3,1)) + (Xs(8:2:end,1) - Xs(2,1))*cos(Xs(3,1));

% For R2 shared feature observation
Fx((2*feaNum+7):2:(end-1),1) = (Xs(7:2:(end-1),1) - Xs(4,1))*cos(Xs(6,1)) + (Xs(8:2:end,1) - Xs(5,1))*sin(Xs(6,1));
Fx((2*feaNum+8):2:end,1) = (Xs(7:2:(end-1),1) - Xs(4,1))*-sin(Xs(6,1)) + (Xs(8:2:end,1) - Xs(5,1))*cos(Xs(6,1));