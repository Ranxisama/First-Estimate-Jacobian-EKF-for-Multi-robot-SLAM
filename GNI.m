function [XsGni,PsGni] = GNI(R1Xp0,Xs,Ps,Zks,ConvergenceCondition)

XsGni = Xs;
DD = 1;

k = 0;
while DD >= ConvergenceCondition
    
    k = k+1;

    % if k == 6
    %     pause
    % end

    XsGni(3,1) = wrap(XsGni(3,1));

    Fx = F(R1Xp0,XsGni,Zks);

    JFx = JacobiF(R1Xp0,XsGni,Zks);

    % R2Zks(:,2)-Fx
    % JFx*X

    Xold = XsGni;
    
    XsGni = Xold + (JFx'/Ps*JFx)\JFx'/Ps*(Zks-Fx);

    D = XsGni - Xold;
    DD = D'*D;
end

PsGni = inv(JFx'/Ps*JFx); % R2Pp0Gni: Cov of R2 posture