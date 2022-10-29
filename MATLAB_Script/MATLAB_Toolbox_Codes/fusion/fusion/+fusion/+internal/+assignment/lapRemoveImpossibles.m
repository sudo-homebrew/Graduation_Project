function [costMatrix, rowIdx, colIdx, unassignedRows, unassignedColumns] = ...
    lapRemoveImpossibles(costMatrix,costOfNonAssignment)
% This function is for internal use only. It may be removed in the future.

%LAPREMOVEIMPOSSIBLES Removes impossible assignments from COSTMATRIX
%   [COSTMATRIXOUT, ROWIDX, COLIDX, UNASSIGNEDROWS, UNASSIGNEDCOLUMNS] = ...
%     LAPREMOVEIMPOSSIBLES(COSTMATRIX, COSTOFNONASSIGNMENT)
%   Removes rows and columns from COSTMATRIX where all values in the row or
%   column are set to Inf. No assignment is possible for these rows or
%   columns and they will be unassigned in the final assignment solution.
%
%   COSTMATRIX is an M-by-N matrix, where each element defines the cost of
%   assigning column n to row m is represented as COSTMATRIX(m,n). Larger
%   assignment costs mean that the assignment is less likely to selected by
%   the algorithm as it seeks to minimize the overall cost of assignment of
%   all columns to rows.
%
%   COSTOFNONASSIGNMENT is a scalar, which represents the cost of leaving
%   unassigned objects. When a value in the cost matrix is greater-than or
%   equal-to twice the cost of non-assignment, that row and column in the
%   cost matrix will not be assigned.
%
%   COSTMATRIXOUT is a P-by-Q matrix, where rows and columns in COSTMATRIX
%   where assignment was not possible have been removed.
%
%   ROWIDX is a row vector of the row indices in COSTMATRIX which are
%   returned in COSTMATRIXOUT.
%
%   COLIDX is a row vector of column indices in COSTMATRIX which are
%   returned in COSTMATRIXOUT.
%
%   UNASSIGNEDROWS is a column vector of the indices of rows in COSTMATRIX
%   which were removed because assignment was not possible.
%
%   UNASSIGNEDCOLS is a row vector of the indices of columns in COSTMATRIX
%   which were removed because assignment was not possible.

% Copyright 2017 The MathWorks, Inc.

%#codegen

% Gate values in the cost matrix which are greater-than or equal-to twice
% the cost of non-assignment
costMatrix(costMatrix>=costOfNonAssignment*2) = Inf;

% Remove gated (set to inf) rows or columns in the costMatrix where no
% assignment is possible
[nRow, nCol] = size(costMatrix);
rowIdx = cast(1:nRow, 'uint32')';
colIdx = cast(1:nCol, 'uint32');

isUnassignedRows = all(isinf(costMatrix), 2);
isUnassignedColumns = all(isinf(costMatrix), 1);
costMatrix = costMatrix(~isUnassignedRows, ~isUnassignedColumns);
rowIdx = rowIdx(~isUnassignedRows);
colIdx = colIdx(~isUnassignedColumns);
unassignedRows = cast(find(isUnassignedRows), 'uint32');
unassignedColumns = cast(find(isUnassignedColumns), 'uint32');
end
