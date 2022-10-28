function maxcount = maxuniquecounts(A)
% This is an internal function and may be removed or modified in a future
% release.

% Copyright 2021 The MathWorks, Inc.

% This function calculates the maximum count of unique elements in a vector
% A. 

%#codegen

if isempty(A)
    maxcount = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntZero();
else
    count = fusion.internal.uniquecounts(A);
    maxcount = max(count);
end
