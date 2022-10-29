function [assignments, unassignedRows, unassignedColumns] = assignmunkres(costMatrix, costOfNonAssignment)    
%ASSIGNMUNKRES  Assignment using Munkres global nearest neighbor
%   [ASSIGNMENTS, UNASSIGNEDROWS, UNASSIGNEDCOLUMNS] = ...
%     ASSIGNMUNKRES(COSTMATRIX, COSTOFNONASSIGNMENT) assigns rows to
%   columns based on the COSTMATRIX using the Munkres algorithm. The
%   Munkres algorithm finds an optimal solution to the global nearest
%   neighbor (GNN) assignment problem, where each column is assigned to a
%   row in a way that minimizes the total cost, assuming their cost is less
%   than the COSTOFNONASSIGNMENT parameter, which serves as a gating
%   parameter.
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
%     assignmunkres(costMatrix, 0.2)
% 
% % Plot detection to track assignments
% figure;
% plot(tracks(:, 1), tracks(:, 2), '*', dets(:, 1), dets(:, 2), 'o');
% hold on; xlim([0, 4]); ylim([0, 4]); legend('tracks', 'detections');
% title('Munkres Assignment of Detections to Tracks');
% assignStr = strsplit(num2str(1:size(assignments, 1)));
% text(tracks(assignments(:, 1), 1)+0.1, ...
%     tracks(assignments(:, 1), 2)-0.1, assignStr);
% text(dets(assignments(:, 2), 1)+0.1, ...
%     dets(assignments(:, 2), 2)-0.1, assignStr);
% text(dets(unassignedDetections(:), 1)+0.1, ...
%         dets(unassignedDetections(:), 2)+0.1, 'unassigned');
%
% See also: assignjv, assignauction.
 
%   Copyright 2017-2018 The MathWorks, Inc.
% 
%   References:
%   [1] Matt L. Miller, Harold S. Stone, and Ingemar J. Cox. Optimizing
%   Murty's Ranked Assignment Method.  IEEE Transactions on Aerospace and
%   Electronic Systems, 33(3), 1997
%
%   [2] James Munkres, Algorithms for Assignment and Transportation
%   Problems, Journal of the Society for Industrial and Applied Mathematics
%   Volume 5, Number 1, March, 1957
%
%   [3] R. A. Pilgrim. Munkres' Assignment Algorithm Modified for
%   Rectangular Matrices
%   http://csclab.murraystate.edu/bob.pilgrim/445/munkres.html

%#codegen

% Parse and check inputs
fusion.internal.assignment.lapCheckCostMatrix(costMatrix, mfilename);
fusion.internal.assignment.lapCheckUnassignedCost(costOfNonAssignment, costMatrix, mfilename);

numberOfRows = size(costMatrix, 1);
numberOfColumns = size(costMatrix, 2);

if numberOfRows == 0 || numberOfColumns == 0 % Nothing to assign
    assignments = zeros(0, 2, 'uint32');
    unassignedRows = cast((1:numberOfRows)', 'uint32');
    unassignedColumns = cast((1:numberOfColumns)', 'uint32');
    return
end

theClass = class(costMatrix);
costUnmatchedTracksVector = ones(1, size(costMatrix, 1), theClass) .* ...
    costOfNonAssignment;
costUnmatchedDetectionsVector = ones(1, size(costMatrix, 2), theClass) .*...
    costOfNonAssignment;

[assignments, unassignedRows, unassignedColumns] = ...
    cvalgAssignDetectionsToTracks(costMatrix, costUnmatchedTracksVector, ...
    costUnmatchedDetectionsVector);

%--------------------------------------------------------------------------
function [matches, unmatchedTracks, unmatchedDetections] = ...
    cvalgAssignDetectionsToTracks(cost, costUnmatchedTracks, ...
    costUnmatchedDetections)

% add dummy rows and columns to account for the possibility of 
% unassigned tracks and observations
paddedCost = getPaddedCost(cost, costUnmatchedTracks, ...
    costUnmatchedDetections);

% solve the assignment problem
[rowInds, colInds] = find(fusion.internal.assignment.hungarianAssignment(paddedCost));

rows = size(cost, 1);
cols = size(cost, 2);
unmatchedTracks = uint32(rowInds(rowInds <= rows & colInds > cols));
unmatchedDetections = uint32(colInds(colInds <= cols & rowInds > rows));

matches = uint32([rowInds, colInds]);
matches = matches(rowInds <= rows & colInds <= cols, :);
if isempty(matches)
    matches = zeros(0, 2, 'uint32');
end

%-------------------------------------------------------------------------
function paddedCost = getPaddedCost(cost, costUnmatchedTracks,...
    costUnmatchedDetections)
% replace infinities with the biggest possible number
bigNumber = getTheHighestPossibleCost(cost);
cost(isinf(cost)) = bigNumber;

% create a "padded" cost matrix, with dummy rows and columns
% to account for the possibility of not matching
rows = size(cost, 1);
cols = size(cost, 2);
paddedSize = rows + cols;
paddedCost = ones(paddedSize, class(cost)) * bigNumber;

paddedCost(1:rows, 1:cols) = cost;

for i = 1:rows
    paddedCost(i, cols+i) = costUnmatchedTracks(i);
end
for i = 1:cols
    paddedCost(rows+i, i) = costUnmatchedDetections(i);
end
paddedCost(rows+1:end, cols+1:end) = 0;

%-------------------------------------------------------------------------
function bigNumber = getTheHighestPossibleCost(cost)
    bigNumber = realmax(class(cost));

%-------------------------------------------------------------------------
