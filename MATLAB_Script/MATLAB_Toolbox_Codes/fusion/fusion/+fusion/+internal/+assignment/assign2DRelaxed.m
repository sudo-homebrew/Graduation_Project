function [assign,cost,price] = assign2DRelaxed(costMatrix,algFcn)
% assign2DRelaxed - Solve a 2-D assignment problem with 1 index assigned to
% the dummy. 
% [assignments,cost] = assign2DRelaxed(costMatrix,algorithm) solves a
% generalized assignment problem where the index 1 is reserved for dummies.
% Algorithm can be specified as a function handle from the following list:
% @assignauction, @assignjv, @assignmunkres.
% 
% [assignments,cost,price] = assign2DRelaxed(costMatrix,@assignauction)
% allows to retrieve the price or dual variables from the auction
% algorithm. If algorithm specified is other than @assignauction, the price
% is returned as zeros.
%
% The first assignment is always a dummy solution i.e. [1 1]. 
% In the optimal solution, [1 1] is included if costMatrix(1,1) < 0
% 
% This is an internal function and may be removed or modified in a future
% release.

% Copyright 2018 The MathWorks, Inc.

%#codegen

% No validation is performed in this function as it is done in the caller
% functions.
classToUse = class(costMatrix);
% The dummy-dummy assignment does not take part in the solution.
temp = costMatrix(1,1);
% Set it's cost to 0 for now.
costMatrix(1,1) = 0;
% Assess the size of cost Matrix.
[nRow,nCol] = size(costMatrix);
% Pad the cost matrix with unassigned rows and columns in a different
% manner than fusion.internal.assignment.lapPadForUnassignedRowsAndColumns.
% In this case the padding is done using the (1,2:end) and (2:end,1) as the
% cost of unassignment for rows and columns respectively.
relaxedMatrix = padCostMatrix(costMatrix);

% If assignauction is used from the top function, divert it to an internal
% auction algorithm which can return the price variables from the auction
% algorithm.
if isequal(algFcn,@assignauction)
    [assignments,unassignedRows,unassignedCols,priceWithDummy] = fusion.internal.assignment.assignAuctionWithPrice(relaxedMatrix);
else
    % For all other assignment algorithms, use their respective functions witha
    % big number for cost of non assignment. This is done because cost of non
    % assignment is already padded in the relaxedMatrix.
    [assignments,unassignedRows,unassignedCols] = algFcn(relaxedMatrix,cast(1e10,classToUse));
    % priceWithDummy is assiged to 0 as price has no meaning for non-auction
    % algorithms.
    priceWithDummy = zeros(1,nCol,classToUse);
end
% Assign unassigned rows and columns to 0.
assignments = [assignments;unassignedRows(:) zeros(numel(unassignedRows),1);zeros(numel(unassignedCols),1) unassignedCols(:)];
assignments = sortrows(assignments);
% Restore original indices from assignment. The assignment takes place from
% (2:end,2:end) as (1,1) does not take part in the assignment problem.
assignments = assignments + 1;
% Modify the priceWithDummy to update the index.
price = [0 priceWithDummy(1:(nCol-1))];

% Remove the padded rows and columns from the solution and restore the
% actual solution.
colSoln = assignments(:,1);
validRows = colSoln <= nRow;
rowSoln = assignments(validRows,2);
rowColSoln = assignments(validRows,1);
rowSoln(rowSoln > nCol) = 1;
dummyRowSoln = assignments(~validRows,2);
validCols = dummyRowSoln <= nCol;
dummyRowSoln = dummyRowSoln(validCols);
assignments = [rowColSoln rowSoln; ones(numel(dummyRowSoln),1) dummyRowSoln];
assign = assignments;
% sortrows as assignmunkres does not return sorted rows.
assign = sortrows(assign);
% Keep only 1 dummy in the solution. 
dummyAssignment = assign(:,1) == 1 & assign(:,2) == 1;
numDummies = sum(dummyAssignment);
if numDummies >= 1
    assign = assign(numDummies+1:end,:);
end
% Pad the assignment with [1 1] as this is required for future enforcements
% of constraints. For example, the third dimension assignment should have a
% dummy variable to be associated with it n3 > n1 or n3 > n2. 
if temp <= 0 || true
    assign = [1 1;assign];
end
% Compute the cost of assignment.
row = assign(:,1);
col = assign(:,2);
costMatrix(1,1) = temp;
ind = sub2ind(size(costMatrix),row,col);
cost = sum(costMatrix(ind));
end

function paddedCost = padCostMatrix(costMatrix)
% This function calculates the padded cost for allowing multiple
% dimensions on index 1 on each dimension.
    innerCost = costMatrix(2:end,2:end);
    dummyFirst = costMatrix(1,2:end);
    dummySecond = costMatrix(2:end,1);
    nFirst = numel(dummyFirst);
    nSecond = numel(dummySecond);
    dummyCostFirst = inf(numel(dummyFirst));
    dummyCostSecond = inf(numel(dummySecond));
    indFirst = sub2ind([nFirst nFirst],1:nFirst,1:nFirst);
    dummyCostFirst(indFirst) = (dummyFirst);
    indSecond = sub2ind([nSecond nSecond],1:nSecond,1:nSecond);
    dummyCostSecond(indSecond) = (dummySecond);
    dummyDummyAssignment = 0*ones(nFirst,nSecond);
    paddedCost = [innerCost dummyCostSecond;dummyCostFirst dummyDummyAssignment];
end


