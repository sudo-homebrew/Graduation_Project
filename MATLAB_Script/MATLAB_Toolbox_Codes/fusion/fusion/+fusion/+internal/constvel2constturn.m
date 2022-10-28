function x = constvel2constturn(x1, x2)
% This function is for internal use and may be removed or modified later.
%constvel2constturn function to convert state or stateCovariance.
%   x = constvel2constturn(x1, x2) converts the state or stateCovariance.
%   x1 - specifies state or stateCovariance corresponding to constvel
%   x2 - specifies state or stateCovariance corresponding to constturn
%   x1 - a L-by-1 real vector or L-by-L real matrix
%   x2 – a M-by-1 real vector or M-by-M real matrix.
%   x  - a M-by-1 real vector or M-by-M real matrix
%
%   Example 1 - Converting state from constant velocity to constant
%   turn
%   --------------------------------------------------------------------
%   %The input parameters for constvel2constturn are
%   x1 = [1;2;3;4;5;6];
%   x2 = [0;0;0;0;0;0;0];
%   x  = constvel2constturn(x1, x2)
%

%   Copyright 2018 The MathWorks, Inc.

%#codegen
dims1= fusion.internal.constveldims(x1);
dims2= fusion.internal.constturndims(x2);
dims = min(dims1, dims2);
x2   = zeros(size(x2),'like',x2);

doCov = ~isvector(x1);
if doCov
    x = zeros(size(x2),'like',x2);
else
    x = x2;
end

if dims<3 % One of the models is not 3D
    ind = true(1,2*dims);
    if doCov
        x(ind,ind) = x1(ind,ind);
    else
        x(ind) = x1(ind);
    end
else % Both x1 and x2 are 3D
    if doCov
        x([1:4,6:7],[1:4,6:7]) = x1;
    else
        x([1:4,6:7]) = x1;
    end
end

% Augment with diagonal block from higher dimension state
if doCov
    x = fusion.internal.fixPZeros(x);
end
end

