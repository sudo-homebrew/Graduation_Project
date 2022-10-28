function [rowSoln, colSoln, colRedux, rowAssignedCnt] = ...
    lapColumnReduction(costMatrix, rowSoln, colSoln, colRedux)
% This function is for internal use only. It may be removed in the future.

%LAPCOLUMNREDUCTION Performs column reduction on the current assignment solution
%   [ROWSOLN, COLSOLN, COLREDUX, ROWASSIGNEDCNT] = ...
%     LAPCOLUMNREDUCTION(COSTMATRIX, ROWSOLN, COLSOLN, COLREDUX)
%   Performs column reduction on the current partial assignment solution.
%   Column reduction is a common operation in solutions to the Linear
%   Assignment Problem (LAP) and is the first step in the Jonker-Volgenant
%   assignment.
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
%   corresponding to the assignment solution.
%
%   ROWASSIGNEDCNT is an M element vector indicating the number of times a
%   row was assigned to a column in the current step of the assignment
%   solution.

% Copyright 2017 The MathWorks, Inc.
%
%   References:
%   -----------
%   [1] R. Jonker and A. Volgenant, A shortest augmenting path algorithm
%   for dense and sparse linear assignment problems, 1987, Computing 38, 4
%   (November 1987), 325-340.
%   [2] A. Volgenant, Linear and semi-assignment problems: a core oriented
%   approach, 1996, Computers & Operations Research 23, 10 (October 1996),
%   917-932.

%#codegen

% For square cost matrices (nRows == nColumns), the first step in JV
% Assignment assigns columns to unassigned rows with the least cost

[nRow, nCol] = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntSize2D(costMatrix);
colAssignedIdx = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntOne();

if nargin < 2 || isempty(rowSoln)
    rowSoln = NaN(nRow, 1, 'like', costMatrix);
end

if nargin < 3 || isempty(colSoln)
    colSoln = NaN(1, nCol, 'like', costMatrix);
end

if nargin < 4 || isempty(colRedux)
    colRedux = zeros(1, nCol, 'like', costMatrix);
end

rowAssignedCnt = zeros(nRow, 1, 'like', costMatrix);

% Starting with the last column, find row in each column with the minimum
% cost. If the min-valued row is not already assigned, assign that row to
% the column
for col = nCol:-1:1
    
    % Find min-valued row
    [costMin, rowMin] = min(costMatrix(:, col));
    
    % Update column reduction
    colRedux(col) = costMin;
    
    % Update the number of times this row was the minimum
    rowAssignedCnt(rowMin) = rowAssignedCnt(rowMin)+1;
    
    % Assign row to column, if not already assigned
    colAssigned = rowSoln(rowMin);
    if isnan(colAssigned)
        rowSoln(rowMin) = col;
        colSoln(col) = rowMin;
    else
        % Cast to integer before indexing
        colAssignedIdx(1) = colAssigned;
        if costMin < colRedux(colAssignedIdx)
            % If the column this row was previously assigned to had a higher
            % cost than this new column, then reassign it
            rowSoln(rowMin) = col;
            colSoln(col) = rowMin;
            colSoln(colAssignedIdx) = NaN;
        end
    end
end
end
