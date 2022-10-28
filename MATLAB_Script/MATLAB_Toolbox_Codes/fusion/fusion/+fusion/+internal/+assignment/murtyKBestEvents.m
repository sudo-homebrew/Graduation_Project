function [FJE, FJEProbs] = murtyKBestEvents(likelihoodMatrix, k)
% This is an internal function and may be removed or modified in a future
% release.

% Copyright 2020 The MathWorks, Inc.

% [FJE, FJEProbs] = fusion.internal.assignment.murtyKBestEvents(likelihoodMatrix, k)
% generates maximum k feasible events and their associated relative
% probabilities. The function uses murty's algorithm to compute the top k
% events.

%#codegen

% Convert likelihood matrix to cost matrix and cost of non assignments
[costMatrix, unassignedRowCost, unassignedColCost] = convertLikelihoodToCost(likelihoodMatrix);

% Use assignkbest to generate k-best solutions
costNonAssignment = {unassignedRowCost;unassignedColCost};

% Use jv algorithm for computing k-best solutions
[assignments,unassignedRows,~,cost] = assignkbest(costMatrix, costNonAssignment, k, 'jv');

% Using the cost output, compute the posterior and associated
% probabilities.

% log(posterior)
logpost = -cost;

% log(sum(posterior));
logsumpost = fusion.internal.logsumexp(logpost);

% log(probs) = log(posterior/sum(posterior)) = log(posterior) - log(sum(posterior))
logprob = logpost - logsumpost;

% probs = exp(log(probs));
FJEProbs = exp(logprob);

% Convert assignment output to event format
[nRows, nCols] = size(likelihoodMatrix);
nDets = nRows - 1;
nTracks = nCols - 1;

% Allocate memory for FJEs
FJE = false(nDets, nCols, numel(cost));

for i = 1:numel(assignments)
    FJE(:,:,i) = convertAssignmentToEvent(assignments{i},unassignedRows{i},nTracks,nDets);
end

end

function [costMatrix, unassignedRowCost, unassignedColCost] = convertLikelihoodToCost(likelihoodMatrix)
% Convert likelihood matrix to cost matrix and associated cost of
% unassignments for assignkbest function.
cost = -log(likelihoodMatrix);
costMatrix = cost(2:end,2:end);
unassignedRowCost = cost(2:end,1);
unassignedColCost = cost(1,2:end);
end

function event = convertAssignmentToEvent(assignments, unassignedDets, nTracks, nDets)
% Convert an assignment to an event format.
event = false(nDets, nTracks + 1);
assignedDets = assignments(:,1);
assignedTracks = assignments(:,2);
event(unassignedDets,1) = true;
ind = sub2ind([nDets nTracks + 1],assignedDets,assignedTracks + 1);
event(ind) = true;
end