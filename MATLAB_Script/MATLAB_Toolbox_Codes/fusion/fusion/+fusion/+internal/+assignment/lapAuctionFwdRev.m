function [rowSoln, colSoln, colRedux] = ...
    lapAuctionFwdRev(costMatrix, varargin)
% This function is for internal use only. It may be removed in the future.

%LAPAUCTIONFWDREV Solution to the Linear Assignment Problem (LAP) using forward/reverse Auction
%   [ROWSOLN, COLSOLN, COLREDUX] = ...
%     LAPAUCTIONFWDREV(COSTMATRIX, ROWSOLN, COLSOLN, COLREDUX, MAXAUCTIONS, ...
%       HASEPSILONSCALING)
%   Assigns rows to columns based on the COSTMATRIX using the
%   forward/reverse Auction assignment algorithm, where each column is
%   assigned to a row in a way that minimizes the total cost.
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
%   [1] D. Bertsekas and D. Castanon, A forward/reverse auction algorithm
%   for asymmetric assignment problems, 1993
%   [2] D. Bertsekas, Linear network optimization: algorithms and codes,
%   MIT Press, 1991

%#codegen

narginchk(1,6);

[nRow, nCol] = size(costMatrix);

[rowSoln, colSoln, colRedux, maxAuctions, hasEpsilonScaling] = ...
    parseInputs(nRow, nCol, class(costMatrix), varargin{:});

% Nothing to do?
if ~any(isnan(rowSoln))
    return;
end

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

bidFactor = 4;
epsFactor = 50;

maxRes = max(costMatrix(isfinite(costMatrix(:))))-min(costMatrix(isfinite(costMatrix(:))));
minRes = findMinResolution(costMatrix);

maxBid = maxRes/bidFactor/(nRow+1); % [maxRes/5, maxRes/2]
minBid = minRes/(nRow+1); % Set to satisfy the optimality condition

epsilon = maxBid;

if ~hasEpsilonScaling
    epsilon = minBid;
end

% Loop over epsilon scaling phases
% Each phase improves the estimate of the column reduction values, which
% leads to faster convergence for the next auction phase using smaller bid
% values. Optimal assignment is guaranteed when epsilon < 1/n

% Init row reduction values for reverse auction
rowRedux = zeros(nRow, 1, 'like', costMatrix);

lastPhase = false;
while ~lastPhase
    
    % Reset assignment to begin next scaling phase
    auctionCount = 0;
    rowSoln = NaN(nRow, 1);
    colSoln = NaN(1, nCol);

    lastPhase = epsilon <= minBid;
    
    while any(isnan(rowSoln)) && ~(lastPhase && auctionCount >= maxAuctions)
        
        % Forward auction
        [rowSoln, colSoln, rowRedux, colRedux] = ...
            auction(costMatrix, epsilon, rowSoln, colSoln, rowRedux, colRedux);

        auctionCount = auctionCount+1;
        if ~any(isnan(rowSoln)) || (lastPhase && auctionCount >= maxAuctions)
            break;
        end
        
        % Reverse auction
        [colSoln, rowSoln, colRedux, rowRedux] = ...
            auction(costMatrix', epsilon, colSoln, rowSoln, colRedux, rowRedux);        

        auctionCount = auctionCount+1;
    end
    
    if epsilon <= minBid
        break;
    end
    
    % Update scaling
    epsilon = max(epsilon/epsFactor, minBid);
end
end

function [rowSoln, colSoln, rowRedux, colRedux] = ...
    auction(costMatrix, epsilon, rowSoln, colSoln, rowRedux, colRedux)
% Performs the forward Auction linear assignment algorithm until the number
% of free rows is reduced by at least one

isUnassigned = isnan(rowSoln);

% Nothing to be done?
if ~any(isUnassigned)
    return
end

nAssigned = sum(~isUnassigned);
% This auction phase may get stuck in "price wars" when epsilon is low.
% Start checking for price war iterations when number of iterations are
% high.
numIterations = uint32(1);
priceWarIter = uint32(1);
initPriceWarCheck = false;

while sum(~isUnassigned) < (nAssigned+1) && priceWarIter < 1000
    numIterations = numIterations + 1;
    initPrices = colRedux(:);
    rowList = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntFind(isUnassigned);
    for thisRow = 1:numel(rowList)
        % Find the next row to assign
        freeRow = rowList(thisRow);
        
        % Bidding phase: find the best and 2nd best column
        costs = costMatrix(freeRow, :) - colRedux(:)';
        
        % Update the min-valued column's reduction value according to
        % the bid price
        [bid, bestCol] = computeBid(costs, isnan(colSoln));
        bid = bid + epsilon;
        colRedux(bestCol) = colRedux(bestCol) - bid;
        rowRedux(freeRow) = costMatrix(freeRow, bestCol) - colRedux(bestCol);
        
        % Assign this row to the min-valued column
        prevRow = colSoln(bestCol);
        colSoln(bestCol) = freeRow;
        rowSoln(freeRow) = bestCol;
        
        % If the minimum column was previously assigned to a row, then
        % unassign it
        if ~isnan(prevRow)
            rowSoln(prevRow) = NaN;
        end
    end
    finalPrices = colRedux(:);
    % Price wars - Protracted sequences of small price rises (or profit
    % reduction) resulting from group of objects competing for a smaller
    % number of roughly equal desirable objects [1].
    if initPriceWarCheck || numIterations > 100000
        initPriceWarCheck = true;
        if all(abs(finalPrices - initPrices) <= epsilon + eps(epsilon))
            priceWarIter = priceWarIter + 1;
        else
            % No price war in last bidding phase.
            priceWarIter = uint32(1);
        end
    end
    
    % Check the number of unassigned rows
    isUnassigned = isnan(rowSoln);
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

costs = costMatrix(isfinite(costMatrix(:)));
costs = unique(costs);
minRes = min(diff(costs));

end

function [rowSoln, colSoln, colRedux, maxAuctions, hasEpsilonScaling] = parseInputs(nRow,nCol,classToUse,varargin)

% Define default input arguments.
inputArgs = {NaN(nRow,1); NaN(1,nCol); zeros(1, nCol, classToUse); inf; true};

% Replace them if provided with input args
coder.unroll();
for i = 1:5
    if nargin - 2 > i && ~isempty(varargin{i})
        inputArgs{i} = varargin{i};
    end
end

[rowSoln, colSoln, colRedux, maxAuctions, hasEpsilonScaling] = deal(inputArgs{:});

end