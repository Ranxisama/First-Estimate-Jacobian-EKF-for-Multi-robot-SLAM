function JFx = JacobiF(R1Xp0,Xs,Zks)
JFx = sparse(size(Zks,1),size(Xs,1));
sharedFeaNum = size(Zks,1)/4;
JFx(sharedFeaNum*2+(1:2:(sharedFeaNum*2-1)),1:3) = [repmat([-cos(Xs(3,1)),-sin(Xs(3,1))],sharedFeaNum, 1), ...
    -sin(Xs(3,1))*(Xs(4:2:(end-1),1)-Xs(1,1)) + cos(Xs(3,1))*(Xs(5:2:end,1)-Xs(2,1))];

JFx(sharedFeaNum*2+(2:2:(sharedFeaNum*2)),1:3) = [repmat([sin(Xs(3,1)),-cos(Xs(3,1))],sharedFeaNum, 1), ...
    -cos(Xs(3,1))*(Xs(4:2:(end-1),1)-Xs(1,1)) - sin(Xs(3,1))*(Xs(5:2:end,1)-Xs(2,1))];

R1Rot = Rot(R1Xp0(3,1));
R2Rot = Rot(Xs(3,1));
for Rjs = 1:sharedFeaNum
    JFx((Rjs-1)*2+(1:2),3+(Rjs-1)*2+(1:2))=R1Rot';
    JFx(sharedFeaNum*2+(Rjs-1)*2+(1:2),3+(Rjs-1)*2+(1:2))=R2Rot';
end


