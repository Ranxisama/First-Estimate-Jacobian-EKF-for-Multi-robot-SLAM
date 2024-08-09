function Xphi = bearingGeneration(XrTrue,Xphi0True,bearingRange)
Deltay = XrTrue(4:2:end)-XrTrue(2:2:end-2);
Deltax = XrTrue(3:2:end-1)-XrTrue(1:2:end-3);
Deltaphi = atan2(Deltay,Deltax);
Xphi = zeros(size(Deltaphi,1)+1,1);
for i = 1:size(Deltaphi,1)+1
    if i == 1
        Xphik = Xphi0True;
    else
        if Deltaphi(i-1,1) > 0
            Xphik = Xphik + Deltaphi(i-1,1) - bearingRange*rand;
        elseif Deltaphi(i-1,1) < 0
            Xphik = Xphik + Deltaphi(i-1,1) + bearingRange*rand;
        else
            Xphik = Xphik + Deltaphi(i-1,1) + 0.5*bearingRange*(2*rand-1);
        end
    end
    Xphi(i,1) = Xphik;
end

Xphi = wrap(Xphi);
% Uphi(end+1,1) = Uphi(end,1) + bearingRange*(2*rand-1);
