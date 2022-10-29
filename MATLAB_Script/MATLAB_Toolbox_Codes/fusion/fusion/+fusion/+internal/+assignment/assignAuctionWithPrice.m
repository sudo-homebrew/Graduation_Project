function [assignments,unassignedRows,unassignedColumns,price] = assignAuctionWithPrice(costMatrix)
% assignAuctionWithPrice Linear Assignment Problem using Auction Algorithm
% using price returned.
% [assignments,price] = assignAuctioWithPrice(costMatrix) solves the 2-D
% assignment problem using Forward reverse auction algorithm. price are the
% values of dual variables when the solution was found.
%
% The price from auction algorithm can be used to update the Lagrangian
% Multipliers for the S-D assignment problem more efficiently.
%
% Note that this function does not solve the Generalized Assignment problem 
% where costMatrix can be rectangular. Therefore, costMatrix input must be 
% padded with rows and columns for unassignment before passing to this 
% function. 
%
% This is an internal function and may be removed or modified in a future
% release.

% Copyright 2018 The MathWorks, Inc.

%#codegen
if isempty(costMatrix)
    assignments = zeros(0,2,'uint32');
    unassignedRows = zeros(0,1,'uint32');
    unassignedColumns = zeros(0,1,'uint32');
    price = zeros(1,0,'like',costMatrix);
    return;
end

% Scale the cost matrix so that padding is not required. We are only
% concerned about the ratio of prices.

maxValue = max(eps,max(abs(costMatrix(isfinite(costMatrix(:))))));
costMatrix = costMatrix/maxValue;
% Cost matrix is now less than 1. 2 is maximum cost of non-assignment for
% all feasible assignments.
smallNumber = 0.1;
bigNumber = cast(2 + smallNumber,class(costMatrix));

% Small eps differences can also cause bad scaling. Auction is sub-optimal
% and this will not affect the quality of solution that we are looking for.
costMatrix = round(costMatrix*1e10)/1e10;

costMatrix(~isfinite(costMatrix)) = sign(costMatrix(~isfinite(costMatrix)))*bigNumber;

% Assign price before reducing matrix
price = zeros(1,size(costMatrix,2),class(costMatrix));

[checkMatrix, rowIdx, colIdx, unassignedRows, unassignedColumns] = ...
    fusion.internal.assignment.lapRemoveImpossibles(costMatrix, bigNumber);

if ~all(size(checkMatrix) == size(costMatrix))
    assignments = zeros(0,2,'like',rowIdx);
    unassignedRows = uint32(1:size(costMatrix,1));
    unassignedColumns = uint32(1:size(costMatrix,2));
    return;
end
% Compute linear assignment
[nRow, nCol] = size(costMatrix);

% The prices are not guaranteed to be negative for Forward-Reverse auction, 
% which is necessary for S-D assignment.
% Use the standard auction to compute prices.
[rowSoln, colSoln, priceColn] = fusion.internal.assignment.lapAuction(costMatrix);
priceColn = priceColn*maxValue;

% Remove assignments to padded rows and columns
rowSoln = rowSoln(1:nRow);
colSoln = colSoln(1:nCol);
price = priceColn(1:nCol);
rowSoln(rowSoln > nCol) = NaN;
colSoln(colSoln > nRow) = NaN;

% Place assignments into the expected return format
isRowAssigned = ~isnan(rowSoln);
isColAssigned = ~isnan(colSoln);
assignments = zeros(0,2,'like',rowIdx);
assignments = [assignments; rowIdx(isRowAssigned), colIdx(rowSoln(isRowAssigned))'];
unassignedRows = [unassignedRows; rowIdx(~isRowAssigned)];
unassignedColumns = [unassignedColumns'; colIdx(~isColAssigned)'];

assignments = reshape(assignments,[],2);
unassignedRows = reshape(unassignedRows,[],1);
unassignedColumns = reshape(unassignedColumns,[],1);

end