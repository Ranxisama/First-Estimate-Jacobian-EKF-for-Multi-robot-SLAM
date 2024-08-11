function [R2Xp0Gni,XfksGni,R2P0Gni] = GNI(R2Zks,R2Xp0,R1Xfks,R1Pfks,ConvergenceCondition)
% R2Zks: shared feature's observation of R2 at step k 
% R2Xp0: initial pose estimate of R2 at step 0
% R1Xfks: shared feature's state of R1 at step k 
% R1Pfks: shared feature's cov of R1 at step k

R2Xp0Gni = R2Xp0;
XfksGni = R1Xfks(:,2);

for gni_num = 1:100
    X = [R2Xp0Gni;XfksGni];

    Fx = FX(R2Xp0Gni,XfksGni);

    JFx = JacobiFX(R2Xp0Gni,XfksGni);

    % R2Zks(:,2)-Fx
    % JFx*X

    Xold = X;

    % X_b = JFx'*(R2Zks(:,2)-Fx+JFx*X);
    % X = (JFx'*JFx)\X_b;
    X_b = JFx'/R1Pfks*(R2Zks(:,2)-Fx+JFx*X);
    X = (JFx'/R1Pfks*JFx)\X_b;

    R2Xp0Gni = X(1:3,1);
    XfksGni = X(4:end,1);

    D = X - Xold;
    DD = D'*D;

    if DD < ConvergenceCondition
        break
    end
end


R2P0Gni = inv(JFx'/R1Pfks*JFx); % R2Pp0Gni: Cov of R2 posture