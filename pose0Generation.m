function Xp0 = pose0Generation(XrTrue,addR1Pose0Noise,R1O)
Xp0 = zeros(3,1);
Xr0True = XrTrue(XrTrue(:,1)==0,2);
Xr1True = XrTrue(XrTrue(:,1)==1,2);
if addR1Pose0Noise == 1
    Xp0(1,1) = Xr0True(1,2) + sqrt(R1O(1,1))*randn;
    Xp0(2,1) = Xr0True(2,2) + sqrt(R1O(2,2))*randn;
    Xp0(3,1) = atan2(Xr1True(2,2)-Xr0True(2,2),Xr1True(1,2)-Xr0True(1,2)) + sqrt(R1O(3,3))*randn;
end