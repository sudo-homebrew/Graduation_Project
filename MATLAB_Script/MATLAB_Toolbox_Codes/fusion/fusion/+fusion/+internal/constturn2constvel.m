function x = constturn2constvel(x1, x2)
% This function is for internal use and may be removed or modified later.
%constturn2constvel function to convert state or stateCovariance.
%   x = constturn2constvel(x1, x2) converts the state or stateCovariance.
%   x1 - specifies state or stateCovariance corresponding to constturn
%   x2 - specifies state or stateCovariance corresponding to constvel
%   x1 - a L-by-1 real vector or L-by-L real matrix
%   x2 – a M-by-1 real vector or M-by-M real matrix.
%   x  - a M-by-1 real vector or M-by-M real matrix
%
%   Example 1 - Converting state from constant turn to constant
%   velocity 
%   --------------------------------------------------------------------
%   %The input parameters for constturn2constvel are
%   x1 = [1;2;3;4;5;6;7];
%   x2 = [0;0;0;0;0;0];
%   x  = constturn2constvel(x1, x2)
%

%   Copyright 2018 The MathWorks, Inc.

%#codegen
dims1 = fusion.internal.constturndims(x1);
dims2 = fusion.internal.constveldims(x2);
dims  = min(dims1, dims2);
x2    = zeros(size(x2),'like',x2);

doCov = ~isvector(x1);
if doCov
    x = zeros(size(x2),'like',x2);
else
    x = x2;
end

ind = 1:2*dims;
if dims < 3
    if doCov
        x(ind,ind) = x1(ind,ind);
    else
        x(ind) = x1(ind);
    end
else % Both models are 3D
    ind1 = [1:4,6:7];
    if doCov
        x(ind,ind) = x1(ind1,ind1);
    else
        x(ind) = x1(ind1);
    end
end

% Augment with diagonal blocks from higher dimension state
if doCov
    iAug = 2*dims+1:2*dims2;
    x(iAug,iAug) = x2(iAug,iAug);
    x = fusion.internal.fixPZeros(x);
end
end
