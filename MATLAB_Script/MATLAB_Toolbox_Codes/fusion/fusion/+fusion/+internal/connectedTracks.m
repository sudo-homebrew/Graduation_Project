function [clustRows,clustCols,numClusters] = connectedTracks(A)
% This is an internal function and may be modified or removed in a future
% release.

% Copyright 2021 The MathWorks, Inc.

% This function forms smaller clusters of detections and tracks that can be
% assigned to each other.
% 
% A is the validation matrix, where A(i,j) defines if ith row can be
% assigned to jth column.
%
% clustRows is the cluster index of rows
% clustCols is the cluster index of columns
% numClusters is the total number of clusters
% 
% Note: unassigned rows and unassigned columns will be added to their own
% cluster

%#codegen

[m,n] = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntSize2D(A);
ZERO = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntZero();
ONE = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntOne();

clustRows = repmat(ZERO,ONE,m);
clustCols = repmat(ZERO,ONE,n);
numClusters = ZERO;
colStack = repmat(ONE,ONE,n);
for track = ONE:n
    if clustCols(track) == 0
        % This column doesn't have a cluster yet
        numClusters = numClusters + ONE;
        clustCols(track) = numClusters;
        colStackSize = ONE;
        colStack(colStackSize) = track;

        while colStackSize > ZERO
            currentCol = colStack(colStackSize);
            colStackSize = colStackSize - ONE;
            colStack(end)=[];
            for row = ONE:m
                if A(row,currentCol) && clustRows(row) == ZERO
                    clustRows(row) = numClusters;
                    for nextCol = ONE:n
                        if A(row,nextCol) && clustCols(nextCol) == ZERO
                            clustCols(nextCol) = numClusters;
                            colStackSize = colStackSize + ONE;
                            colStack(colStackSize) = nextCol;
                        end
                    end
                end
            end
        end
    end
end

% Assign each unassigned row its own cluster similar to columns
unassignedDetCluster = clustRows == 0;
numUnassigned = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntIndex(sum(unassignedDetCluster));
assert(numUnassigned <= matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntNumel(clustRows));
clustRows(unassignedDetCluster) = numClusters + (ONE:numUnassigned);
numClusters = numClusters + numUnassigned;

end