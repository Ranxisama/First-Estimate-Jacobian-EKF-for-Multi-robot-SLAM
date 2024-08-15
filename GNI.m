function [XsGni,PsGni] = GNI(Xs,Ps,Zks,ConvergenceCondition)

XsGni = Xs;

for gni_num = 1:100

    XsGni([3,6],1) = wrap(XsGni([3,6],1));

    Fx = F(XsGni,Zks);

    JFx = JacobiF(XsGni,Zks);

    % R2Zks(:,2)-Fx
    % JFx*X

    Xold = XsGni;
    
    X_b = JFx'/Ps*(Zks-Fx+JFx*XsGni);
    XsGni = (JFx'/Ps*JFx)\X_b;

    D = XsGni - Xold;
    DD = D'*D;

    if DD < ConvergenceCondition
        break
    end
end

PsGni = inv(JFx'/Ps*JFx); % R2Pp0Gni: Cov of R2 posture