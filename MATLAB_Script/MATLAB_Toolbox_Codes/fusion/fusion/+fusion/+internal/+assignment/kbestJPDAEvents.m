function [FJE, FJEProbs] = kbestJPDAEvents(likelihoodMatrix, k)
% This is an internal function and may be removed or modified in a future
% release.

% Copyright 2020 The MathWorks, Inc.

% [FJE, FJEProbs] = kbestJPDAEvents(likelihoodMatrix,k) returns the k-best
% events from the likelihood matrix  and their associated normalized
% probabilities.

%#codegen

% Process input
validateattributes(likelihoodMatrix,{'single','double'},...
    {'real','finite','nonempty','nonsparse','nonnegative'},'jpdaEvents','likelihoodMatrix');

validateattributes(k,{'numeric'},...
    {'scalar','nonsparse','integer','positive'},'jpdaEvents','k');

% There must be at least 2 rows and 2 columns for k-best JPDA events
coder.internal.assert(size(likelihoodMatrix,1) > 1,'fusion:jpdaEvents:expectedLikelihoodMoreRows');
coder.internal.assert(size(likelihoodMatrix,2) > 1,'fusion:jpdaEvents:expectedLikelihoodMoreColumns');

% The unassignment likelihoods must be positive

% Detection unassignment
coder.internal.assert(all(likelihoodMatrix(1,2:end) > 0),'fusion:jpdaEvents:expectedPositiveTrackUnassignment');
% Track unassignment
coder.internal.assert(all(likelihoodMatrix(2:end,1) > 0),'fusion:jpdaEvents:expectedPositiveDetectionUnassignment');

% The rest of the matrix must have at least one non-zero value per column
coder.internal.assert(all(sum(likelihoodMatrix(2:end,2:end),1)>0), 'fusion:jpdaEvents:expectedNonzeroLikelihoodColumn');



% The Murty's algorithm is heavy for small problems and if k is comparable
% to maximum possible feasible events. However, the feasible events
% requires a huge amount of memory for large problems. A trade-off criteria
% is defined like this:
isStaticMemory = matlabshared.tracking.internal.fusion.isCodegenWithStaticMemory;
if ~isStaticMemory
    validationMatrix = likelihoodMatrix(2:end,:) > 0;
    [nRow, nCol] = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntSize2D(likelihoodMatrix);
    ONE = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntOne();
    nMeas = nRow - ONE;
    nTracks = nCol - ONE;
    TEN = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntIndex(10);
    nPotentials = fusion.internal.assignment.numPotentialFeasibleEvents(validationMatrix, nMeas, nTracks, class(likelihoodMatrix));
    toUseMurty = k < nPotentials/TEN || (nTracks > 8 && nMeas > 8);
else
    % When code is generated with static memory allocation, always use
    % Murty's algorithm. 
    toUseMurty = true;
end

% If Murty's use k-best algorithm, otherwise, use all feasible events and
% sort to get the first k events.
if toUseMurty
    [FJE, FJEProbs] = fusion.internal.assignment.murtyKBestEvents(likelihoodMatrix, k);
else
    [FJE, FJEProbs] = fusion.internal.assignment.feasibleAndSortKBestEvents(likelihoodMatrix, validationMatrix, k);
end

end
