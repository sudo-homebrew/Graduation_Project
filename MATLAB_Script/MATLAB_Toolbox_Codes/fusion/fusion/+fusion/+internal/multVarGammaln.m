function v = multVarGammaln(a,d)
% This is an internal function and may be modified or removed in a future
% release.
% This function outputs the logarithm of multi-variate gamma function.
% The function outputs -realmax on negative values of input,a.
% v = multVarGammaLn(a,d) returns the logarithm of d-variate gamma
% function evaluated at a.

% Copyright 2018 The MathWorks, Inc.

%#codegen
dataType = class(a);

if d == 1
    if a < 0
        v = -realmax(dataType);
    else
        v = gammaln(a);
    end
else
    % log(pi) = 1.1447298858494;
    logPi = cast(1.1447298858494,dataType);
    if a + (1-d)/2  < 0
        v = -realmax(dataType);
        return;
    end
    % Number of recursions are d-1. d is typically 2-3.
    v = (d-1)/2*logPi + fusion.internal.multVarGammaln(a,d-1) + gammaln(a + (1-d)/2);
end
end