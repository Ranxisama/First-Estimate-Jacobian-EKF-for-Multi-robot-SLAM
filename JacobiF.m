function JFx = JacobiF(Xs,Zks)

feaNum = (size(Xs,1)-6)/2;

JFx = sparse(size(Zks,1),size(Xs,1));

JFx(1:6,1:6) = eye(6);

JFx(6+(1:2:(2*feaNum-1)),1:3) = [repmat([-cos(Xs(3,1)),-sin(Xs(3,1))],feaNum, 1), ...
    -sin(Xs(3,1))*(Xs(7:2:(end-1),1)-Xs(1,1)) + cos(Xs(3,1))*(Xs(8:2:end,1)-Xs(2,1))];

JFx(6+(2:2:(2*feaNum)),1:3) = [repmat([sin(Xs(3,1)),-cos(Xs(3,1))],feaNum, 1), ...
    -cos(Xs(3,1))*(Xs(7:2:(end-1),1)-Xs(1,1)) - sin(Xs(3,1))*(Xs(8:2:end,1)-Xs(2,1))];

JFx((2*feaNum+7):2:(end-1),4:6) = [repmat([-cos(Xs(6,1)),-sin(Xs(6,1))],feaNum, 1), ...
    -sin(Xs(6,1))*(Xs(7:2:(end-1),1)-Xs(4,1)) + cos(Xs(6,1))*(Xs(8:2:end,1)-Xs(5,1))];

JFx((2*feaNum+8):2:end,4:6) = [repmat([sin(Xs(6,1)),-cos(Xs(6,1))],feaNum, 1), ...
    -cos(Xs(6,1))*(Xs(7:2:(end-1),1)-Xs(4,1)) - sin(Xs(6,1))*(Xs(8:2:end,1)-Xs(5,1))];

R1Rot = rotationMatrix(Xs(3,1));
R2Rot = rotationMatrix(Xs(6,1));
for Rjs = 1:feaNum
    JFx(6+(Rjs-1)*2+(1:2),6+(Rjs-1)*2+(1:2))=R1Rot';
    JFx(6+2*feaNum+(Rjs-1)*2+(1:2),6+(Rjs-1)*2+(1:2))=R2Rot';
end


