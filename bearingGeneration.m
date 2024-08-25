function Xphi = bearingGeneration(XrTrue,bearingRange)
Deltay = XrTrue(4:2:end,2)-XrTrue(2:2:end-2,2);
Deltax = XrTrue(3:2:end-1,2)-XrTrue(1:2:end-3,2);
Deltaphi = wrap(atan2(Deltay,Deltax));
Xphi = zeros(size(Deltaphi,1)+1,2);
Xphi(:,1) = unique(XrTrue(:,1));
for i = 1:(size(Deltaphi,1))
    if i == 1
        Xphi(i,2) = Deltaphi(i,1);
        continue
    end

    if Deltaphi(i,1)-Deltaphi(i-1,1) > 0
        Xphi(i,2) = Deltaphi(i,1) + bearingRange;
    elseif Deltaphi(i,1)-Deltaphi(i-1,1) < 0
        Xphi(i,2) = Deltaphi(i,1) - bearingRange;
    else
        Xphi(i,2) = Deltaphi(i,1);
    end
end
Xphi(end,2) = Xphi(end-1,2);
Xphi(:,2) = wrap(Xphi(:,2));
