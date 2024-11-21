function Xp0 = pose0Generation(XrTrue,XphiTrue,addR1Pose0Noise,R1O)
if addR1Pose0Noise == 0
    R1O(1,1) = 0;
    R1O(2,2) = 0;
    R1O(3,3) = 0;
end
Xp0 = zeros(3,1);
Xr0True = XrTrue(XrTrue(:,1)==0,2);

Xp0(1,1) = Xr0True(1,1) + sqrt(R1O(1,1))*Randn(1,1);
Xp0(2,1) = Xr0True(2,1) + sqrt(R1O(2,2))*Randn(1,1);
Xp0(3,1) = XphiTrue(1,2) + sqrt(R1O(3,3))*Randn(1,1);
