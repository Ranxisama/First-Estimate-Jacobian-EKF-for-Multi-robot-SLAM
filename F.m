function Fx = F(R1Xp0,Xs,Zks)
Fx = zeros(size(Zks,1),1);
sharedFeaNum = size(Zks,1)/4;
for zksNum = 1:sharedFeaNum
    Fx((zksNum-1)*2+(1:2),1) = rotationMatrix(R1Xp0(3,1))'*(Xs(3+(zksNum-1)*2+(1:2),1)-R1Xp0(1:2,1));
    Fx(sharedFeaNum*2+(zksNum-1)*2+(1:2),1) = rotationMatrix(Xs(3,1))'*(Xs(3+(zksNum-1)*2+(1:2),1)-Xs(1:2,1));
end