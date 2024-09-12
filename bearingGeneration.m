function Xphi = bearingGeneration(XrTrue,bearingRange)
Deltay = XrTrue(4:2:end,2)-XrTrue(2:2:end-2,2);
Deltax = XrTrue(3:2:end-1,2)-XrTrue(1:2:end-3,2);
Phi = wrap(atan2(Deltay,Deltax));
Xphi = zeros(size(Phi,1)+1,2);
Xphi(:,1) = unique(XrTrue(:,1));
for i = 1:(size(Phi,1))
    if i == 1
        Xphi(i,2) = Phi(i,1);
        continue
    end
    DPhi = Phi(i,1)-Phi(i-1,1);
    if abs(DPhi) > bearingRange
        if DPhi > 0
            Xphi(i,2) = Phi(i,1) - bearingRange;
        else
            Xphi(i,2) = Phi(i,1) + bearingRange;
        end
    else
        Xphi(i,2) = Phi(i,1);
    end
end
Xphi(end,2) = Xphi(end-1,2);
Xphi(:,2) = wrap(Xphi(:,2));
