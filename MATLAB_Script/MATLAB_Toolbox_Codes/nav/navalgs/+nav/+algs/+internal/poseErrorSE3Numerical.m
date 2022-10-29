function [e, Jaci, Jacj] = poseErrorSE3Numerical(Toi, Toj, Tij)
%This function is for internal use only. It may be removed in the future.

%POSEERRORSE3NUMERICAL Compute pose error, and Jacobians w.r.t. Tio, Tjo numerically 

%   Copyright 2020 The MathWorks, Inc.

%#codegen

if coder.target('MATLAB')
    % When running in MATLAB, use MEX file for improved performance
    [e, Jaci, Jacj] = nav.algs.internal.mex.poseErrorSE3Numerical(Toi, Toj, Tij);
else
    % When generating code, use MATLAB implementation directly
    [e, Jaci, Jacj] = nav.algs.internal.impl.poseErrorSE3Numerical(Toi, Toj, Tij);
end

end

