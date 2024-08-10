function Xphi = bearingGeneration(XrTrue,bearingRange)
Deltay = XrTrue(4:2:end,2)-XrTrue(2:2:end-2,2);
Deltax = XrTrue(3:2:end-1,2)-XrTrue(1:2:end-3,2);
Deltaphi = atan2(Deltay,Deltax);
Xphi = zeros(size(Deltaphi,1),1);
Xphi(:,1) = Deltaphi(:,1) + bearingRange*(2*rand(size(Deltaphi,1),1)-1);
Xphi(end+1,1) = Xphi(end,1) + bearingRange*(2*rand-1);
Xphi = wrap(Xphi);
