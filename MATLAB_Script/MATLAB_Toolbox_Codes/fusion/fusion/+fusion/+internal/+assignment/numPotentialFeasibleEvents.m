function nPotentials = numPotentialFeasibleEvents(validationMatrix, numMeas, numTracks, classToUse)
% This is an internal function and may be removed or modified in a future
% release.

% Copyright 2020-2021 The MathWorks, Inc.

%#codegen

n1 = cast(1,classToUse);
nT = cast(min([numMeas,numTracks]),classToUse);
nM = cast(max([numMeas,numTracks]),classToUse);

nPotentials = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntZero();

for k = 0:nT-1
    n1 = n1 + nchoosek(nT,k)*nchoosek(nM,nT-k)*factorial(nT-k);
end

n2 = prod(sum(cast(validationMatrix,classToUse),2),1);
nPotentials(1) = min(n1,n2);

end