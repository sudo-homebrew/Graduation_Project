function [assignments, unassignedRows, unassignedColumns] = ...
    assignjv(costMatrix, costOfNonAssignment)
%ASSIGNJV Assignment using Jonker-Volgenant global nearest neighbor
%   [ASSIGNMENTS, UNASSIGNEDROWS, UNASSIGNEDCOLUMNS] = ...
%     ASSIGNJV(COSTMATRIX, COSTOFNONASSIGNMENT) assigns rows to columns
%   based on the COSTMATRIX using the Jonker-Volgenant assignment algorithm
%   which finds an optimal solution to the global nearest neighbor (GNN)
%   assignment problem, where each column is assigned to a row in a way
%   that minimizes the total cost, assuming their cost is less than the
%   COSTOFNONASSIGNMENT parameter, which serves as a gating parameter.
%
%   The Jonker-Volgenant algorithm solves the GNN assignment in two phases:
%   beginning with the Auction algorithm and finishing with Dijkstra's
%   shortest path algorithm.
%
%   ASSIGNJV is faster than ASSIGNMUNKRES for larger GNN assignments, such
%   as cases where there are more than 50 rows and columns in the
%   COSTMATRIX.
%
%   This assignment algorithm is useful, for example, in assigning tracks
%   to detections. This can be done by setting COSTMATRIX(i, j) to the cost
%   of assigning j-th detection to i-th track. The lower the cost, the more
%   likely the assignment is to be made.
% 
%   COSTMATRIX is an M-by-N matrix, where M is the number of objects to
%   assign, e.g., tracks, and N is the number of objects to be assigned,
%   e.g., detections. COSTOFNONASSIGNMENT is a scalar, which represents the
%   cost of leaving unassigned objects. Higher COSTOFNONASSIGNMENT
%   corresponds to the higher likelihood that every existing object will be
%   assigned.
%
%   ASSIGNMENTS is an L-by-2 matrix of index pairs of rows and
%   corresponding columns, where L is the number of pairs. The first column
%   in the ASSIGNMENTS matrix contains the row indices and the second
%   column contains the corresponding column indices. For example, in
%   assigning tracks to detections, the first column will represent the
%   tracks while the second is the corresponding detections.
% 
%   UNASSIGNEDROWS is a P-element column vector, where P is the number of
%   unassigned rows. Each element is an index of a row to which no columns
%   were assigned. For example, in the aforementioned assignment of tracks
%   to detections, this column will be of unassigned tracks.
%
%   UNASSIGNEDCOLUMNS is a Q-element column vector, where Q is the number
%   of unassigned columns. Each element is an index of a column that was
%   not assigned to any rows. For example, in the aforementioned assignment
%   of tracks to detections. this column will be of unassigned detections.
%
% Class Support:
% --------------
% All inputs must be of the same class, which can be single or double, and
% they must be real and nonsparse. COSTMATRIX may contain Inf entries to
% indicate that no assignment is possible. COSTOFNONASSIGNMENT must be
% finite.
%
% All outputs are of class uint32.
%
% % Example:
% % Given the following predicted track locations, tracks:
% tracks = [1,1; 2,2];
% 
% % A list of 3 detections is received. There are currently 2 tracks and 3
% % detections, so at least one detection will not be assigned.
% dets = [1.1, 1.1; 2.1, 2.1; 1.5, 3];
% 
% % Compute the cost of matching each detection to a track using the
% % Euclidean distance. You can also use the tracking filter distance method.
% for i = size(tracks, 1):-1:1
%     delta = dets - tracks(i, :);
%     costMatrix(i, :) = sqrt(sum(delta .^ 2, 2));
% end
% 
% % Assign detections to tracks
% %   Detection 1 should match track 1, detection 2 to track 2, and detection
% %   3 should be unassigned.
% [assignments, unassignedTracks, unassignedDetections] = ...
%     assignjv(costMatrix, 0.2)
% 
% % Plot detection to track assignments
% figure;
% plot(tracks(:, 1), tracks(:, 2), '*', dets(:, 1), dets(:, 2), 'o');
% hold on; xlim([0, 4]); ylim([0, 4]); legend('tracks', 'detections');
% title('Jonker-Volgenant Assignment of Detections to Tracks');
% assignStr = strsplit(num2str(1:size(assignments, 1)));
% text(tracks(assignments(:, 1), 1)+0.1, ...
%     tracks(assignments(:, 1), 2)-0.1, assignStr);
% text(dets(assignments(:, 2), 1)+0.1, ...
%     dets(assignments(:, 2), 2)-0.1, assignStr);
% text(dets(unassignedDetections(:), 1)+0.1, ...
%         dets(unassignedDetections(:), 2)+0.1, 'unassigned');
%
% See also: assignmunkres, assignauction.
 
%   Copyright 2017-2018 The MathWorks, Inc.
% 
%   References:
%   -----------
%   [1] R. Jonker and A. Volgenant, A shortest augmenting path algorithm
%   for dense and sparse linear assignment problems, 1987, Computing 38, 4
%   (November 1987), 325-340.
%   [2] A. Volgenant, Linear and semi-assignment problems: a core oriented
%   approach, 1996, Computers & Operations Research 23, 10 (October 1996),
%   917-932.
%   [3] Samuel Blackman and Robert Popoli, Design and Analysis of Modern
%   Tracking Systems, 1999, Artech House.

%#codegen

% Parse and check inputs
fusion.internal.assignment.lapCheckCostMatrix(costMatrix, mfilename);
fusion.internal.assignment.lapCheckUnassignedCost(costOfNonAssignment, costMatrix, mfilename);

[costMatrix, rowIdx, colIdx, unassignedRows, unassignedColumns] = ...
    fusion.internal.assignment.lapRemoveImpossibles(costMatrix, costOfNonAssignment);

[nRow, nCol] = size(costMatrix);
costMatrix = fusion.internal.assignment.lapPadForUnassignedRowsAndColumns(costMatrix, costOfNonAssignment);

% Compute linear assignment
[rowSoln, colSoln, colRedux, rowAssignedCnt] = fusion.internal.assignment.lapColumnReduction(costMatrix);
colRedux = ...
    fusion.internal.assignment.lapReductionTransfer(costMatrix, rowSoln, colRedux, rowAssignedCnt);
[rowSoln, colSoln, colRedux] = ...
    fusion.internal.assignment.lapAuction(costMatrix, rowSoln, colSoln, colRedux, 2, false);
[rowSoln, colSoln] = ...
    fusion.internal.assignment.lapDijkstra(costMatrix, rowSoln, colSoln, colRedux);

% Remove assignments to padded rows and columns
rowSoln = rowSoln(1:nRow);
colSoln = colSoln(1:nCol);
rowSoln(rowSoln > cast(nCol,'like',rowSoln)) = NaN;
colSoln(colSoln > cast(nRow,'like',colSoln)) = NaN;

% Place assignments into the expected return format
isRowAssigned = ~isnan(rowSoln);
isColAssigned = ~isnan(colSoln);
assignments = zeros(0,2,'like',rowIdx);
rowSolnIdx = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntIndex(rowSoln(isRowAssigned));
assignments = [assignments; rowIdx(isRowAssigned), colIdx(rowSolnIdx)'];
unassignedRows = [unassignedRows; rowIdx(~isRowAssigned)];
unassignedColumns = [unassignedColumns'; colIdx(~isColAssigned)'];

assignments = reshape(assignments,[],2);
unassignedRows = reshape(unassignedRows,[],1);
unassignedColumns = reshape(unassignedColumns,[],1);
end