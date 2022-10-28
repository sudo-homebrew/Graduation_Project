function [solution,cost,isValidSoln] = kbestGet2DSolution(costMatrix,costOfNonAssignment,algFcn)
% kbestGet2DSolution - Calculates solution for 2-D assignment problem with 
% a cost matrix padded with dummy rows and columns for unassignment.
% 
% Inputs:
% costMatrix is a padded cost matrix with dummies for unassigned rows and
% columns. 
% costOfNonAssignment is a large number greater than the costMatrix. This
% is to ensure that the assignment functions are called with the required
% syntax.
% As cost of non-assignment is included in the matrix, any solution which
% picks up this cost is declared as invalid. Invalid solutions occur when
% more constraints are enforced on the costMatrix and hence no feasible
% solutions can occur.
%
% Outputs:
% solution - A P-by-2 list of assignments 
% cost - cost of assignment of solution.
% isValidSoln - A flag indicating if the solution is valid.
%
% This is an internal function and may be removed in a future release.

% Copyright 2018 The MathWorks, Inc.

%#codegen

[solution,unassignedRows,unassignedCols] = algFcn(costMatrix,costOfNonAssignment);
isValidSoln = true;

% Because cost of non-assignment is included in the cost matrix, every
% assignment which could not assign all rows and columns is not a valid
% solution.
if ~isempty(unassignedRows) || ~isempty(unassignedCols)
    isValidSoln = false;
end

cost = calcCost(costMatrix,solution,unassignedRows,unassignedCols,costOfNonAssignment);
solution = sortrows(solution);

end

function cost = calcCost(costMatrix,assignment,unassignedRows,unassignedCols,costOfNonAssignment)

row = assignment(:,1);
col = assignment(:,2);

assignedIndices = sub2ind(size(costMatrix),row,col);
cost = sum(costMatrix(assignedIndices)) + (numel(unassignedRows) + numel(unassignedCols))*costOfNonAssignment;

end