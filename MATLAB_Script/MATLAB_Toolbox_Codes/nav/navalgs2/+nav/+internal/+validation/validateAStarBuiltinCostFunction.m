function [strVal,idx,validStringsDist] = validateAStarBuiltinCostFunction(costFuncStr)
%This function is for internal use only. It may be removed in the future.

%validateNavPath Validate a navPath object

%   Copyright 2020 The MathWorks, Inc.

%#codegen

    coder.varsize('strVal');
    validStringsDist = {'Euclidean','Manhattan','Chebyshev','EuclideanSquared'};
    strVal = validatestring(costFuncStr,validStringsDist,'');
    idx = find(strcmp(validStringsDist,strVal));
end