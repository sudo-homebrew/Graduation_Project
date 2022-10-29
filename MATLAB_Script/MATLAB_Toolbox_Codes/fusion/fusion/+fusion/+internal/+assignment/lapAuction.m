function [rowSoln, colSoln, colRedux] = ...
    lapAuction(costMatrix, rowSoln, colSoln, colRedux, maxAuctions, hasEpsilonScaling)
% This function is for internal use only. It may be removed in the future.

%LAPAUCTION Solution to the Linear Assignment Problem (LAP) using forward Auction
%   [ROWSOLN, COLSOLN, COLREDUX] = ...
%     LAPAUCTION(COSTMATRIX, ROWSOLN, COLSOLN, COLREDUX, MAXAUCTIONS, ...
%       HASEPSILONSCALING)
%   Assigns rows to columns based on the COSTMATRIX using the forward
%   Auction assignment algorithm, where each column is assigned to a row in
%   a way that minimizes the total cost.
%
%   COSTMATRIX is an M-by-N matrix, where each element defines the cost of
%   assigning column n to row m is represented as COSTMATRIX(m,n). Larger
%   assignment costs mean that the assignment is less likely to selected by
%   the algorithm as it seeks to minimize the overall cost of assignment of
%   all columns to rows.
%
%   ROWSOLN is an M element column vector, with each element set to the
%   column assigned to the corresponding row. When no column is assigned,
%   the element is set to NaN.
%
%   COLSOLN is an N element row vector, with each element set to the row
%   assigned to the corresponding column. When no row is assigned, the
%   element is set to NaN.
%
%   COLREDUX is an N element row vector of the column reduction values
%   corresponding to the returned assignment solution.
%
%   Algorithm can use a partial solution to the LAP by passing in the
%   ROWSOLN, COLSOLN, and COLREDUX from a previous partial solution. If not
%   provided, the algorithm assumes no partial solution has been provided.
%
%   The number of auction cycles can be limited by setting MAXAUCTIONS to a
%   positive integer. If not provided, the algorithm iterates until the
%   optimal solution which minimizes the total assignment cost is found. By
%   setting MAXAUCTIONS, an optimal solution is not guaranteed, and a
%   partial solution may be returned.
%
%   Enabling epsilon-scaling by setting HASEPSILONSCALING to true can speed
%   up the algorithm's convergence for some linear assignment problems.

% Copyright 2017 The MathWorks, Inc.
%
%   References:
%   -----------
%   [1] D. Bertsekas, Auction algorithms, Encyclopedia of Optimization,
%   Kluwer, 2001
%   [2] D. Bertsekas, Linear network optimization: algorithms and codes,
%   MIT Press, 1991

%#codegen

[nRow, nCol] = size(costMatrix);

if nargin < 2 || isempty(rowSoln)
    rowSoln = NaN(nRow, 1, 'like', costMatrix);
end

if nargin < 3 || isempty(colSoln)
    colSoln = NaN(1, nCol, 'like', costMatrix);
end

if nargin < 4 || isempty(colRedux)
    colRedux = zeros(1, nCol, 'like', costMatrix);
end

if nargin < 5 || isempty(maxAuctions)
    maxAuctions = inf;
end

if nargin < 6 || isempty(hasEpsilonScaling)
    hasEpsilonScaling = true;
end

% Nothing to do?
if ~any(isnan(rowSoln))
    return;
end

% Define constant
ZERO = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntZero();

% Bertsekas' recommendations for e-scaling:
%
%   bidFactor in the bounds [2 5]
%   epsFactor in the bounds [4, 10] (forward-reverse auction should use
%                                    larger values)
%   epsilon in the bounds [minBid, maxBid] (note that setting to
%                                           minBid disables e-scaling)
%
% Auction parameters below selected to give best performance across broad
% range of assignment problems

bidFactor = cast(4,'like',costMatrix);
epsFactor = cast(7,'like',costMatrix);

maxRes = max(costMatrix(isfinite(costMatrix(:))))-min(costMatrix(isfinite(costMatrix(:))));
minRes = findMinResolution(costMatrix);

nPlus1 = cast(nRow,'like',costMatrix) + 1;
maxBid = maxRes/bidFactor/nPlus1; % [maxRes/5, maxRes/2]
minBid = minRes/nPlus1; % Set to satisfy the optimality condition

epsilon = maxBid;

if ~hasEpsilonScaling
    epsilon = minBid;
end

% Loop over epsilon scaling phases
% Each phase improves the estimate of the column reduction values, which
% leads to faster convergence for the next auction phase using smaller bid
% values. Optimal assignment is guaranteed when epsilon < 1/n

lastPhase = false;
while ~lastPhase
    
    % Reset assignment to begin next scaling phase
    auctionCount = ZERO;
    rowSoln = NaN(nRow, 1, 'like', costMatrix);
    colSoln = NaN(1, nCol, 'like', costMatrix);
    
    lastPhase = epsilon <= minBid;
    
    % Perform auction until all rows are assigned or max iteration count is reached
    while any(isnan(rowSoln)) && ~(lastPhase && auctionCount >= maxAuctions)
        [rowSoln, colSoln, colRedux] = ...
            auction(costMatrix, epsilon, rowSoln, colSoln, colRedux);
        
        auctionCount = auctionCount+1;
    end
    
    % Update scaling
    epsilon = max(epsilon/epsFactor, minBid);
end
end

function [rowSoln, colSoln, colRedux] = ...
    auction(costMatrix, epsilon, rowSoln, colSoln, colRedux)
% Performs one iteration of the forward Auction linear assignment algorithm

rowPrevIdx = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntOne();

% Each Auction iteration cycles over all free rows
rowList = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntFind(isnan(rowSoln));
for thisRow = 1:numel(rowList)
    
    % Find the next row to assign
    rowFree = rowList(thisRow);
    
    % Bidding phase: find the best and 2nd best column
    costs = costMatrix(rowFree, :) - colRedux;
    
    % Update the min-valued column's reduction value according to
    % the bid price
    [bid, bestCol] = computeBid(costs, isnan(colSoln));
    bid = bid + epsilon;
    colRedux(bestCol) = colRedux(bestCol) - bid;

    % Assign this row to the min-valued column
    rowPrev = colSoln(bestCol);
    colSoln(bestCol) = rowFree;
    rowSoln(rowFree) = bestCol;
    
    % If the minimum column was previously assigned to a row, then
    % unassign it
    if ~isnan(rowPrev)
        rowPrevIdx(1) = rowPrev;
        rowSoln(rowPrevIdx) = NaN;
    end
    
end
end

function [bid, bestInd] = computeBid(costs, isUnassigned)
% Returns the 2 smallest values and their corresponding indices in costs.
% When multiple values at the same minimum are found, will return a value
% corresponding to an unassigned index as the first minimum

[min1, minInds] = findAllMin(costs);
bestInd = minInds(1);
if numel(minInds) > 1
    minInds = minInds(2:end);
    isMinUnassigned = isUnassigned(minInds);
    if any(isMinUnassigned)
        minInds = minInds(isMinUnassigned);
        bestInd = minInds(1);
    end
    bid = zeros(1,'like',costs);
else
    % Find second smallest minimum
    costs(minInds) = NaN;
    min2 = min(costs);
    bid = min2-min1;
end
end

function [minValue, minIndicies] = findAllMin(values)
% Returns the minimum value and all indices in 'values' that correspond to
% the minimum value in 'values'

minValue = min(values);
minIndicies = find(values == minValue);

end


function minRes = findMinResolution(costMatrix)
% Returns the minimum difference between any two finite values in the cost
% matrix
if coder.target('MATLAB')
    minRes = findMinResolution_sim(costMatrix);
else
    minRes = findMinResolution_cg(costMatrix);
end

end

function minRes = findMinResolution_sim(costMatrix)
costs = costMatrix(isfinite(costMatrix(:)));
costs = unique(costs);
% Only 1 finite unique element in the list makes the resolution as 0.
if isscalar(costs)
    minRes = cast(0,'like',costMatrix);
else
    minRes = min(diff(costs));
end
end

function minRes = findMinResolution_cg(costMatrix)
idx = coder.internal.sortIdx(costMatrix(:),'a');
minRes = cast(inf,'like',costMatrix);
n = numel(costMatrix);
for i = 1:n-1
    thisDiff = abs(costMatrix(idx(i+1)) - costMatrix(idx(i)));
    if thisDiff > 0 && thisDiff < minRes
        minRes = thisDiff;
    end
end
end
