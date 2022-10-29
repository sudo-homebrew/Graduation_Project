function [clustRows, clustCols, nClusters, currentMaxRowsPerCluster, currentMaxColumnsPerCluster, connMatrix] = constrainedConnectedTracks(costMatrix, connMatrix,...
    maxRowsPerCluster, maxColsPerCluster, violationHandling, violationMsg)
% This is an internal function and may be removed or modified in a future
% release. 

% Copyright 2021 The MathWorks, Inc.

% This function forms smaller clusters of detections and tracks that can be
% assigned to each other. It allows you to specify bounds on maximum number
% of detections and tracks per cluster. Upon violation, you can trigger to
% error or split the cluster.
%
% [clustRows, clustCols, nClusters, connMatrix] = fusion.internal.constrainedConnectedTracks(costMatrix, connMatrix, maxRowsPerCluster, maxColsPerCluster, violationHandling, violationMsg) 
% 
% costMatrix is a M-by-N cost matrix
%
% connMatrix is a logical M-by-N matrix. connMatrix(i,j) = true means ith row can be assigned to jth column.
%
% maxRowsPerCluster is the maximum number of row elements per cluster
%
% maxColsPerCluster is the maximum number of column elements per cluster
%
% violationHandling is a fusion.internal.ClusterViolationHandlingType enum
% to define what to do if cluster bounds are violated. 
%
% violationMsg is a 2-by-1 msg identifier for error or warning. The first
% element is when rows violate and second element is when columns violate.
% 
% clustRows is a 1-by-M matrix describing the cluster index of each row
% clustCols is a 1-by-N matrix describing the cluster index of each column
% nClusters is the total number of clusters
% maxClusterSize is the size of the largest cluster [numRows numCols]
% connMatrix is the updated connMatrix matrix (if there was splitting).

%#codegen

% Perform clustering using DFS
[clustRows, clustCols, nClusters] = fusion.internal.connectedTracks(connMatrix);

% Calculate max number per cluster
currentMaxRowsPerCluster = fusion.internal.maxuniquecounts(clustRows);
currentMaxColumnsPerCluster = fusion.internal.maxuniquecounts(clustCols);

hasRowViolation = currentMaxRowsPerCluster > maxRowsPerCluster;
hasColumnViolation = currentMaxColumnsPerCluster > maxColsPerCluster;

switch violationHandling
    case 1 % Terminate
         throwError(hasRowViolation, hasColumnViolation, violationMsg);
    case 2 % Split and warn
        [clustRows, clustCols, nClusters, connMatrix] = fusion.internal.splitClustersUsingWeakestLink...
            (costMatrix, connMatrix, maxRowsPerCluster, maxColsPerCluster, clustRows, clustCols, nClusters);
        throwWarning(hasRowViolation, hasColumnViolation, violationMsg);
    case 3 % Split don't warn
        [clustRows, clustCols, nClusters, connMatrix] = fusion.internal.splitClustersUsingWeakestLink...
            (costMatrix, connMatrix, maxRowsPerCluster, maxColsPerCluster, clustRows, clustCols, nClusters);
end

end

function throwError(hasRowViolation,hasColumnViolation, violationMsg)
    coder.internal.assert(~hasRowViolation, violationMsg{1});
    coder.internal.assert(~hasColumnViolation, violationMsg{2});
end

function throwWarning(hasRowViolation,hasColumnViolation,violationMsg)
if hasRowViolation
    coder.internal.warning(violationMsg{1});
end
if hasColumnViolation
    coder.internal.warning(violationMsg{2});
end
end


