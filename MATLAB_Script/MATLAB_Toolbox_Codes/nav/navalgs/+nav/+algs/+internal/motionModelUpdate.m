function Xn = motionModelUpdate(Xn_1, oldOdometry, newOdometry, alpha)
%This function is for internal use only. It may be removed in the future.

%motionModelUpdate Implementation of sampling based odometry motion model
%
%   XN = MOTIONMODELUPDATE(XN_1, OLDODOMETRY, NEWODOMETRY, ALPHA)
%   returns the current poses XN, given the previous sample poses XN_1,
%   previous odometry OLDODOMETY, current odometry NEWODOMETRY, and
%   then noise parameters ALPHA for the sampling-based odometry motion
%   model.

%   Copyright 2016-2018 The MathWorks, Inc.

%   References:
%
%   [1] S. Thrun, W. Burgard and D. Fox, Probabilistic Robotics.
%   Cambridge, MA: MIT Press, 2005.

%#codegen

    if coder.target('MATLAB')
        % Use MEX file for improved performance
        Xn = nav.algs.internal.mex.motionModelUpdate(...
            Xn_1, oldOdometry, newOdometry, alpha);
    else % Use MATLAB implementation
        Xn = nav.algs.internal.impl.motionModelUpdate(...
            Xn_1, oldOdometry, newOdometry, alpha);
    end
end
