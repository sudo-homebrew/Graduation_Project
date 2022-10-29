function [hdop, vdop] = calculateDOP(dopMatrix, refFrame, lla0)
%CALCULATEDOP Get dilution of precision (DOP) values in local frame from 
%   Earth-Center-Earth-Fixed (ECEF) cofactor matrix
%
%   This function is for internal use only. It may be removed in the
%   future.

%   Copyright 2020 The MathWorks, Inc.

%#codegen

T = eye(4, 'like', dopMatrix);
T(1:3,1:3) = refFrame.ecef2framerotmat(lla0(1), lla0(2));

% Convert from ECEF to local frame.
dopMatrix = T * dopMatrix * T.';

diagDOPMat = diag(dopMatrix);

hdop = sqrt(sum(diagDOPMat(1:2)));
vdop = sqrt(diagDOPMat(3));
end
