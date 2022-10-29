function [e, Jaci, Jacj] = poseErrorSE3Numerical(Toi, Toj, Tij) %
%This function is for internal use only. It may be removed in the future.

%poseErrorSE3Numerical Estimate SE(3) pose error and Jacobians w.r.t. Tio
%   and Tjo. Jacobians are estimated through numerical finite difference.
%   This function is MEX'ed to accelerate MATLAB execution.

%   Copyright 2020 The MathWorks, Inc.

%#codegen

    Tio = robotics.core.internal.SEHelpers.tforminvSE3(Toi);
    Tji = robotics.core.internal.SEHelpers.tforminvSE3(Tij);

    e = robotics.core.internal.SEHelpers.veelogmSE3(Tji*Tio*Toj);
            
    Tjo = robotics.core.internal.SEHelpers.tforminvSE3(Toj);

    n = 6;
    N = eye(n);
    delta = 1e-5; % cannot be too small (to avoid numerical precision issues in veelogmSE3)
    Jaci = zeros(n);
    Jacj = zeros(n); 
    for i = 1:n

        %Jaci numeric finite diff
        Tp = robotics.core.internal.SEHelpers.expSE3hat(N(:,i)*delta);
        Tiop = Tp * Tio;

        Tm = robotics.core.internal.SEHelpers.expSE3hat(-N(:,i)*delta);
        Tiom = Tm * Tio;

        poseErrPlus = robotics.core.internal.SEHelpers.veelogmSE3(Tji * Tiop * Toj);
        poseErrMinus = robotics.core.internal.SEHelpers.veelogmSE3(Tji * Tiom *Toj);
        Jaci(:,i) = (poseErrPlus - poseErrMinus)/(2*delta);

        %Jac2 numeric finite diff
        Tjop = Tp * Tjo;
        Tjom = Tm * Tjo;
        poseErrPlus = robotics.core.internal.SEHelpers.veelogmSE3(Tji * Tio * robotics.core.internal.SEHelpers.tforminvSE3(Tjop));
        poseErrMinus = robotics.core.internal.SEHelpers.veelogmSE3(Tji * Tio * robotics.core.internal.SEHelpers.tforminvSE3(Tjom));
        Jacj(:,i) = (poseErrPlus - poseErrMinus)/(2*delta);
    end

end