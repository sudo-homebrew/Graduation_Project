function colRedux = ...
    lapReductionTransfer(costMatrix, rowSoln, colRedux, rowAssignedCnt)
% This function is for internal use only. It may be removed in the future.

%LAPREDUCTIONTRANSFER Exchanges the reduction value between columns and rows
%   COLREDUX = ...
%     LAPREDUCTIONTRANSFER(COSTMATRIX, ROWSOLN, COLREDUX, ROWASSIGNEDCNT)
%   Updates the column reduction values, COLREDUX, in the current step of
%   the assignment solution by transferring the row reduction values to
%   column reduction values. Reduction transfer  is a common operation in
%   solutions to the Linear Assignment Problem (LAP) and typically follows
%   column reduction to update column reduction values based on the number
%   of row assignments that occurred in the column reduction.
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

% For square cost matrices (nRows == nColumns), the second step in JV
% Assignment enables the next step (Augmenting Row Reduction) by adjusting
% the column reduction values to ensure that the costs across each row will
% not include negative values

% Update column reduction values for rows that were assigned only once
assignedOnceRows = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntFind(~isnan(rowSoln) & rowAssignedCnt == 1);

colAssigned = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntOne();

for thisRow = 1:numel(assignedOnceRows)
    
    row = assignedOnceRows(thisRow);
    
    % Find minimum value for the column not assigned to this row
    rowCost = costMatrix(row, :) - colRedux;
    colAssigned(1) = rowSoln(row);
    rowCost(colAssigned) = NaN;
    costMin = min(rowCost);
    
    % Update the column reduction by this minimum value
    colRedux(colAssigned) = colRedux(colAssigned) - costMin;
end
end
