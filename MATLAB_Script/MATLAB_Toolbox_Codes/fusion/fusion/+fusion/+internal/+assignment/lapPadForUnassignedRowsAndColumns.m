function costMatrix = lapPadForUnassignedRowsAndColumns(costMatrix, costOfNonAssignment)
% This function is for internal use only. It may be removed in the future.

%LAPPADFORUNASSIGNEDROWSANDCOLUMNS Pads cost matrix to allow for non-assignment solutions
%   COSTMATRIXOUT = ...
%     LAPPADFORUNASSIGNEDROWSANDCOLUMNS(COSTMATRIX, COSTOFNONASSIGNMENT)
%   Pads COSTMATRIX to allow for solutions where rows and columns may not
%   be assigned.
%
%   COSTMATRIX is an M-by-N matrix, where each element defines the cost of
%   assigning column n to row m is represented as COSTMATRIX(m,n). Larger
%   assignment costs mean that the assignment is less likely to selected by
%   an algorithm as it seeks to minimize the overall cost of assignment of
%   all columns to rows.
%
%   COSTOFNONASSIGNMENT is either:
%   A scalar value representing the cost of not
%   assigning a row or a column. 
%   A 2-element cell array of vectors. The first element defines the cost
%   of non assignment for each row. The second element defines the cost of
%   non assignment for each column. 
%   A 2-element vector. The first element defines the cost of non
%   assignment for all rows. The second element defines the cost of non
%   assignment for all columns. 
%
%   COSTMATRIXOUT is an L-by-L matrix, where L = M+N. The returned cost
%   matrix is of the form:
%
%                                     | x      Inf     Inf     ...     Inf
%                                     | Inf    x       Inf     ...     Inf
%           COSTMATRIX                | Inf    Inf      x      ...      :
%                                     |  :      :       :       x       :
%                                     | Inf    Inf     Inf     ...     x
% ------------------------------------+-----------------------------------
% x      Inf     Inf     ...     Inf  |
% Inf    x       Inf     ...     Inf  |
% Inf    Inf      x      ...      :   |           ZEROS(N,M)
% Inf    Inf     Inf     ...     x    |

% Copyright 2017 The MathWorks, Inc.

%#codegen

% Pad matrix with dummy rows and columns to include the costOfNonAssignment
% in the solution
[nRow, nCol] = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntSize2D(costMatrix); 
[costRowUnassignment, costColUnassignment] = parseCostOfNonAssignment(costMatrix, costOfNonAssignment, nRow, nCol);
classToUse = class(costMatrix);
    
if coder.target('MATLAB')
    dummyCols = Inf(nRow, classToUse);
    dummyCols(sub2ind([nRow, nRow], 1:nRow, 1:nRow)) = costRowUnassignment;
    dummyRows = Inf(nCol, classToUse);
    dummyRows(sub2ind([nCol, nCol], 1:nCol, 1:nCol)) = costColUnassignment;
    costMatrix = [...
        costMatrix, dummyCols;...
        dummyRows, zeros(nCol, nRow, classToUse)];
else
    costMatrix = blkdiag(costMatrix,zeros(nCol,nRow,classToUse));
    for i = 1:nRow
        costMatrix(i,(nCol+1):end) = inf;
        costMatrix(i,nCol+i) = costRowUnassignment(i);
    end
    for i = 1:nCol
        costMatrix((nRow+1):end,i) = inf;
        costMatrix(nRow+i,i) = costColUnassignment(i);
    end
end

end

function [costRowUnassignment, costColUnassignment] = parseCostOfNonAssignment(costMatrix, costOfNonAssignment, nRow, nCol)
    if isscalar(costOfNonAssignment)
        costRowUnassignment = bsxfun(@times,costOfNonAssignment,ones(nRow,1,'like',costMatrix));
        costColUnassignment = bsxfun(@times,costOfNonAssignment,ones(1,nCol,'like',costMatrix));
    elseif iscell(costOfNonAssignment)
        costRowUnassignment = reshape(costOfNonAssignment{1},nRow,1);
        costColUnassignment = reshape(costOfNonAssignment{2},1,nCol);
    else
        costRowUnassignment = bsxfun(@times,costOfNonAssignment(1),ones(nRow,1,'like',costMatrix));
        costColUnassignment = bsxfun(@times,costOfNonAssignment(2),ones(1,nCol,'like',costMatrix));
    end
end