function [XsGni,PsGni] = GNI_Ide(R1Xp0,R1Xp0T,Xs,XsT,Ps,Zks,ZksT,ConvergenceCondition)

XsGni = Xs;
DD = 1;

while DD >= ConvergenceCondition
    
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

JFx_Ide = JacobiF(R1Xp0T,XsT,ZksT);

PsGni = inv(JFx_Ide'/Ps*JFx_Ide);