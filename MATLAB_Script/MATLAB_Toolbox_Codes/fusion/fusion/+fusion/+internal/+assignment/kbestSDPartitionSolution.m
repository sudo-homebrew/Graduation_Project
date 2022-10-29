function partitionedSolution = kbestSDPartitionSolution(solution, desiredGap, maxIterations, algorithm)
% partitionSolution - partition an SD solution into multiple lists based on
% Murty's algorithm.
%
% Inputs:
% solution - a cell array of cells organized as follows:
% 1st cell element: constrained cost matrix
% 2nd cell element: solution to constrained cost matrix
% 3rd cell element: cost and gap of assignment
% 4th cell element: number of assignments in solutions which were enforced.
%
% desiredGap, maxIterations, algorithm are the same as the inputs to assignkbestsd
%
% Outputs:
% An array of solutions partitioned from the input solution organized in
% the same format.
%
% This is an internal function and may be removed in a future release.

% Copyright 2018, The MathWorks, Inc.
%#codegen

% Get current assignment, costMatrix and number of enforced assignments
assignment = solution{1}{2};
costMatrix = solution{1}{1};
enforcedConstraintIndex = 0;%solution{1}{4};

% The partition of assignment only takes places on "chosen" assignments and
% not forced assignments
toPartitionAssignment = assignment(1+enforcedConstraintIndex:end,:);
numPartitions = size(toPartitionAssignment,1);

% Create memory for partitioned solution.
partitionedSolution = repmat(solution,[1 0]);
coder.varsize('partitionedSolution',[1 inf],[0 1]);

for k = 1:numPartitions
    % Remove kth tuple from toPartitionAssignment
    tuplesToRemove = toPartitionAssignment(k,:);
    
    % Enforce first k-1 tuples from toPartitionAssignment
    tuplesToEnforce = toPartitionAssignment(1:k-1,:);
    
    % Enforce constraints
    constrainedCost = fusion.internal.assignment.kbestEnforceConstraints(costMatrix,tuplesToRemove,tuplesToEnforce);
    
    % Solve the new problem
    [newSolution, newCost, newGap, isValidSoln] = fusion.internal.assignment.kbestGetSDSolution(constrainedCost, desiredGap, maxIterations, algorithm);
    
    % Store the solution in list
    if isValidSoln
        % Initialize the solution
        % In codegen is the partitionedSolution is not defined, it can be
        % defined from the first solution. The subsequent elements can be
        % added using the end+1 index.
        if isempty(partitionedSolution)
            partitionedSolution = {solution{1}};
        else
            partitionedSolution{end+1} = solution{1};
        end
        % Set values for the new solution
        partitionedSolution{end}{1} = constrainedCost;
        partitionedSolution{end}{2} = newSolution;
        partitionedSolution{end}{3} = [newCost, newGap];
        partitionedSolution{end}{4} = enforcedConstraintIndex + k - 1;
    end
end

end