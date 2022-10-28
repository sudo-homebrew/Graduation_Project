function [FJE, FJEProbs] = feasibleAndSortKBestEvents(likelihoodMatrix, validationMatrix, k)
% This is an internal function and may be removed or modified in a future
% release.

% Copyright 2020 The MathWorks, Inc.

% [FJE, FJEProbs] = feasibleAndSortKBestEvents(likelihoodMatrix,
% validationMatrix, k) returns the k-best solutions using the input
% likelihood matrix and validation matrix. It first computes all feasible
% events and then selects the top k events by sorting their posterior
% likelihood. This function is useful when the problem size is small and
% top-k events are desired.

%#codegen

% Compute the top-k events using the depth-search creation of feasible
% events and then sorting it by their posterior likelihoods.
ONE = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntOne();
kFound = ONE;
kInt = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntIndex(k);

skipValidation = true;
classToUse = class(likelihoodMatrix);

allFJE = fusion.internal.assignment.feasibleJPDAEvents(validationMatrix, skipValidation, classToUse);

posterior = calcPosteriorLikelihood(likelihoodMatrix, allFJE);

% Sort posterior by their values
[posterior, idx] = sort(posterior,'descend');

kFound(1) = min(kInt, numel(posterior));

% First k-posterior and normalized probabilities
kposterior = posterior(ONE:kFound);
FJEProbs = kposterior/sum(kposterior);

% First k-events
FJE = allFJE(:,:,idx(ONE:kFound));

end

function posterior = calcPosteriorLikelihood(likelihoodMatrix, FJE)
% Calculate the posterior likelihood of an event
nFJE = size(FJE,3);
posterior = zeros(nFJE,1,'like',likelihoodMatrix);
lhood = likelihoodMatrix(:);

for i = 1:nFJE
    FJEi = FJE(:,:,i);
    isTrackUnassigned = [false sum(FJEi(:,2:end),1) == 0];
    FJEPaddedi = [isTrackUnassigned;FJEi];
    posterior(i) = prod(lhood(FJEPaddedi(:)));
end

end