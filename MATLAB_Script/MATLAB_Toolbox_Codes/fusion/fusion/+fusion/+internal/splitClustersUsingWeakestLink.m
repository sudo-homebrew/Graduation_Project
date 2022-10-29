function [clustRows, clustCols, nClusters, valMatrix] = splitClustersUsingWeakestLink(costMatrix, valMatrix, ...
    maxRowsPerCluster, maxColsPerCluster,...
    clustRows, clustCols, nClusters)
% This is an internal function and may be removed or modified in a future
% release

% Copyright 2021 The MathWorks, Inc.

% This function processes the pre-calculated clusters of a cost matrix to
% ensure it satisfies maximum number of rows and columns allowed per
% cluster.
%
% It uses recursive forbidding of weakest links per violating cluster until
% the constraints are met.
% 
% outputs costMatrix and validation matrix 

%#codegen

currentMaxRowsPerCluster = fusion.internal.maxuniquecounts(clustRows);
currentMaxColsPerCluster = fusion.internal.maxuniquecounts(clustCols);

while currentMaxRowsPerCluster > maxRowsPerCluster || currentMaxColsPerCluster > maxColsPerCluster
    % Break weakest link per violating cluster
    if coder.target('MATLAB')
        valMatrix = breakWeakestLinkPerCluster(costMatrix, valMatrix, ...
            clustRows,clustCols,nClusters,maxRowsPerCluster,maxColsPerCluster);
    else
        valMatrix = breakWeakestLinkPerCluster_cg(costMatrix, valMatrix, ...
            clustRows,clustCols,nClusters,maxRowsPerCluster,maxColsPerCluster);
    end

    % Use the DFS algorithm to reform clusters
    [clustRows, clustCols, nClusters] = fusion.internal.connectedTracks(valMatrix);

    % Find maximum rows and columns per cluster
    currentMaxRowsPerCluster = fusion.internal.maxuniquecounts(clustRows);
    currentMaxColsPerCluster = fusion.internal.maxuniquecounts(clustCols);
end

end

function valMatrix = breakWeakestLinkPerCluster_cg(costMatrix, valMatrix, clustRows, clustCols, numClusters, maxRowsPerCluster, maxColsPerCluster)
% This function allows a memory-efficient way in generated code to set the
% maximum element per cluster to inf.
ONE = matlabshared.tracking.internal.fusion.codegen.StrictSingleCoderUtilities.IntOne();
[nRows, nCols] = matlabshared.tracking.internal.fusion.codegen.StrictSingleCoderUtilities.IntSize2D(costMatrix);
bigNum = realmax(class(costMatrix));
for i = 1:numClusters
    thisDets = clustRows == i;
    thisTracks = clustCols == i;
    if sum(thisDets) > maxRowsPerCluster || sum(thisTracks) > maxColsPerCluster
        maxVal = -bigNum;
        maxIIdx = ONE;
        maxJIdx = ONE;
        for iIdx = ONE:nRows
            for jIdx = ONE:nCols
                if valMatrix(iIdx,jIdx) && thisDets(iIdx) && thisTracks(jIdx) && costMatrix(iIdx,jIdx) > maxVal
                    maxVal = costMatrix(iIdx,jIdx);
                    maxIIdx = iIdx;
                    maxJIdx = jIdx;
                end
            end
        end
        valMatrix(maxIIdx,maxJIdx) = false;
    end
end
end

function valMatrix = breakWeakestLinkPerCluster(costMatrix, valMatrix, ...
            clustRows,clustCols,numClusters,maxRowsPerCluster,maxColsPerCluster)
% This function breaks the weakest link of validation matrix for violating
% clusters. 
for i = 1:numClusters
    thisRows = find(clustRows == i);
    thisCols = find(clustCols == i);
    if numel(thisRows) > maxRowsPerCluster || numel(thisCols) > maxColsPerCluster
        thisCost = costMatrix(thisRows,thisCols);
        thisVal = valMatrix(thisRows,thisCols);
        validIdx = find(thisVal);
        validCost = thisCost(validIdx);
        [~,idx] = max(validCost);
        thisVal(validIdx(idx)) = false;
        valMatrix(thisRows,thisCols) = thisVal;
    end
end
end


