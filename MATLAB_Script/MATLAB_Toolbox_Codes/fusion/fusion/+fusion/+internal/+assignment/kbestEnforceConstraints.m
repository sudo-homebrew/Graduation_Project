function constrainedCost = kbestEnforceConstraints(costMatrix,tuplesToRemove,tuplesToEnforce)
% kbestEnforceConstraints - Enforce the constraints on a costMatrix.
% This function enforces the constraints on a costMatrix. tuplesToRemove is
% a N-by-2 list, which defines the assignments which should not be a part of
% the assignment. tuplesToEnforce is a N-by-2 list, which defines tuples
% which should be a part of the assignment.
%
% This is an internal function and may be removed in a future release.

% Copyright 2018 The MathWorks, Inc.

%#codegen
constrainedCost = removeTuple(costMatrix,tuplesToRemove);
constrainedCost = enforceTuple(constrainedCost,tuplesToEnforce);

end

function costMatrix = removeTuple(costMatrix,tuples)
% This function modifies the cost matrix such that the input assignment,
% tuples, does not appear in the solution to costMatrix.
% To remove the tuple from the list, it's corresponding assignment value is
% set to Inf.
dimSizes = size(costMatrix);
numDims  = numel(dimSizes);
indices = cell(numDims,1);

for i = 1:numel(indices)
    indices{i} = tuples(:,i);
end
costMatrix(indices{:}) = Inf(1,'like',costMatrix);
end

function costMatrix = enforceTuple(costMatrix,tuples)
% This function modifies the costMatrix so that the input assignment,
% tuples, is forced in the solution of the assignment problem. To enforce a
% tuple, all feasible assignments are set to Inf except the enforced one.
% For example, to enforce assignment (2,3) set costMatrix(:,3) = Inf and
% costMatrix(2,:) = Inf and retain costMatrix(2,3) to it's original value.

dimSizes = size(costMatrix);
numDims  = numel(dimSizes);
allArgs = cell(numDims,1);
for i = 1:numDims
    allArgs{i} = uint32(1:dimSizes(i));
end

numTuples = size(tuples,1);
for i = 1:numTuples
    if coder.target('MATLAB')
        tuplesInCell = num2cell(tuples(i,:));
    else
        tuplesInCell = cell(numDims,1);
        for k = 1:numel(tuplesInCell)
            tuplesInCell{k} = tuples(i,k);
        end
    end
    tempCost = costMatrix(tuplesInCell{:});
    for j = 1:size(tuples,2)
        % Assignments of the type [1 4 6] where 1 is a dummy should be
        % enforced as follows:
        % costMatrix(:,4,:) = inf;
        % costMatrix(:,:,6) = inf;
        % costMatrix(1,4,6) = inf; 
        % invalid command: costMatrix(1,:,:) = inf
        indices = allArgs;
        if ~(tuples(i,j) == 1 && numDims > 2)
            indices{j} = tuples(i,j);
            costMatrix(indices{:}) = inf(1,'like',costMatrix);
        end
    end
    % Store back the only permissive cost.
    costMatrix(tuplesInCell{:}) = tempCost;
end

end
