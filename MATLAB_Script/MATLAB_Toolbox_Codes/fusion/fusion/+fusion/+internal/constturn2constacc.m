function x = constturn2constacc(x1, x2)
% This function is for internal use and may be removed or modified later.
%constturn2constacc function to convert state or stateCovariance.
%   x = constturn2constacc(x1, x2) converts the state or stateCovariance.
%   x1 - specifies state or stateCovariance corresponding to constturn
%   x2 - specifies state or stateCovariance corresponding to constacc
%   x1 - a L-by-1 real vector or L-by-L real matrix
%   x2 – a M-by-1 real vector or M-by-M real matrix.
%   x  - a M-by-1 real vector or M-by-M real matrix
%
%   Example 1 - Converting state from constant turn to constant
%   acceleration 
%   --------------------------------------------------------------------
%   %The input parameters for constturn2constacc are
%   x1 = [1;2;3;4;5;6;7];
%   x2 = [0;0;0;0;0;0;0;0;0];
%   x  = constturn2constacc(x1, x2)
%

%   Copyright 2018 The MathWorks, Inc.

%#codegen
x2 = zeros(size(x2),'like',x2);
dims1 = fusion.internal.constturndims(x1);
if dims1 < 3
    inds = 1:2*dims1;
else
    inds = [1:4,6:7];
end

doCov = ~isvector(x1);
if doCov
    x = fusion.internal.constvel2constacc(x1(inds,inds), x2);
else
    x = fusion.internal.constvel2constacc(x1(inds), x2);
end
end
