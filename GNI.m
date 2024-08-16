function [XsGni,PsGni] = GNI(R1Xp0,Xs,Ps,Zks,ConvergenceCondition)

XsGni = Xs;

for gni_num = 1:100

    XsGni(3,1) = wrap(XsGni(3,1));

    Fx = F(R1Xp0,XsGni,Zks);

    JFx = JacobiF(R1Xp0,XsGni,Zks);

    % R2Zks(:,2)-Fx
    % JFx*X

    Xold = XsGni;
    
    XsGni = Xold + (JFx'/Ps*JFx)\JFx'/Ps*(Zks-Fx);

    D = XsGni - Xold;
    DD = D'*D;

    if DD < ConvergenceCondition
        break
    end
end

PsGni = inv(JFx'/Ps*JFx); % R2Pp0Gni: Cov of R2 posture