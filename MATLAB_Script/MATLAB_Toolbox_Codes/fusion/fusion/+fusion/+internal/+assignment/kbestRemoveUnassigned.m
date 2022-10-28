function [assignment,unassignedRows,unassignedCols] = kbestRemoveUnassigned(assignment,costSize)
% kbestRemoveUnassigned - remove unassigned rows and columns from the assignment
% solution of a padded cost matrix.
%
% Inputs:
% assignments is a P-by-2 list of solutions to the padded cost matrix.
% costSize is a 2-element vector defining the size of original cost matrix.
% 
% Outputs:
% assignments - a N-by-2 list of solutions to the original cost matrix.
% unassignedRows - a list of unassigned rows 
% unassignedCols - a list of unassigned columns.
% This is an internal function and may be removed in a future release.

% Copyright 2018 The MathWorks, Inc.

%#codegen

nRow = costSize(1);
nCol = costSize(2);
rowIdx = uint32(1:nRow)';
rowSoln = assignment(:,2);
rowSoln = rowSoln(1:nRow);
rowSoln(rowSoln > nCol) = 0;
isRowAssigned = rowSoln ~= 0;
if nargout > 2
    colIdx = uint32(1:nCol)';
    [~,I] = sortrows(assignment(:,2));
    colSoln = assignment(I,:);
    colSoln = colSoln(1:nCol);
    colSoln(colSoln > nRow) = 0;
    isColAssigned = colSoln ~= 0;
    unassignedRows = rowIdx(~isRowAssigned);
    unassignedCols = colIdx(~isColAssigned);
end
assignment = assignment(1:nRow,:);
assignment = assignment(isRowAssigned,:);
end