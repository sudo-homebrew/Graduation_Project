function x = constvel2constacc(x1, x2)
% This function is for internal use and may be removed or modified later.
%constvel2constacc function to convert state or stateCovariance.
%   x = constvel2constacc(x1, x2) converts the state or stateCovariance.
%   x1 - specifies state or stateCovariance corresponding to constvel
%   x2 - specifies state or stateCovariance corresponding to constacc
%   x1 - a L-by-1 real vector or L-by-L real matrix
%   x2 – a M-by-1 real vector or M-by-M real matrix.
%   x  - a M-by-1 real vector or M-by-M real matrix
%
%   Example 1 - Converting state from constant velocity to constant
%   acceleration 
%   --------------------------------------------------------------------
%   %The input parameters for constvel2constacc are
%   x1 = [1;2;3;4;5;6];
%   x2 = [0;0;0;0;0;0;0;0;0];
%   x  = constvel2constacc(x1, x2)
%

%   Copyright 2018 The MathWorks, Inc.

%#codegen
dims1 = fusion.internal.constveldims(x1);
dims2 = fusion.internal.constaccdims(x2);
dims  = min(dims1, dims2);
doCov = ~isvector(x1);
classToUse = class(x2);
conversionMatrix = zeros(dims2*3,dims1*2,classToUse);
oneDConvMat = cast([1 0;0 1;0 0],classToUse);
switch dims
    case 1
        a = oneDConvMat;
    case 2
        a = blkdiag(oneDConvMat,oneDConvMat);
    otherwise % Validation of dimensions is done in switchimm
        a = blkdiag(oneDConvMat,oneDConvMat,oneDConvMat);
end
conversionMatrix(1:dims*3,1:dims*2) = a;
if doCov
    x = conversionMatrix * cast(x1,classToUse) * conversionMatrix';
    x = fusion.internal.fixPZeros(x);
else
    x = conversionMatrix * cast(x1,classToUse);
end