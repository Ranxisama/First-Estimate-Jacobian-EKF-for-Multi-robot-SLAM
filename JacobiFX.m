function JFx = JacobiFX(Xp0,Xfk)
JFx = sparse(size(Xfk,1),size(Xp0,1)+size(Xfk,1));
JFx(1:2:(end-1),1:3) = [repmat([-cos(Xp0(3,1)),-sin(Xp0(3,1))],size(Xfk,1)/2, 1), ...
    -sin(Xp0(3,1))*(Xfk(1:2:(end-1),1)-Xp0(1,1)) + cos(Xp0(3,1))*(Xfk(2:2:end,1)-Xp0(2,1))];

JFx(2:2:end,1:3) = [repmat([sin(Xp0(3,1)),-cos(Xp0(3,1))],size(Xfk,1)/2, 1), ...
    -cos(Xp0(3,1))*(Xfk(1:2:(end-1),1)-Xp0(1,1)) - sin(Xp0(3,1))*(Xfk(2:2:end,1)-Xp0(2,1))];

Rot = rotationMatrix(Xp0(3,1));
for R1js = 1:size(Xfk,1)/2
    JFx((R1js-1)*2+(1:2),size(Xp0,1)+(R1js-1)*2+(1:2))=Rot';
end