function cellNumbers = clusterUsingDistance(distanceMatrix,threshold)
% This is an internal function and may be modified or removed in future

% This function implements the Distance partitioning algorithm.
%
% cellNumbers = clusterUsingDistance(distanceMatrix,threshold) returns the
% partitions of the entities used in distanceMatrix according to the
% threshold.
% threshold is a scalar value defining the maximum distance allowed between
% two entities in the same cluster.
% distanceMatrix is a symmetric matrix with 0s on the diagonal.
% cellNumbers is the vector of n elements, where each element defines the
% cluster index of the entity.

% Copyright 2018-2019 The MathWorks, Inc.

%#codegen
n = size(distanceMatrix,1);
cellNumbers = zeros(n,1,'uint32');
    
cellId = cast(1,'uint32');

for i = 1:n
    if cellNumbers(i) == 0
        cellNumbers(i) = cellId;
        cellNumbers = findNeighbors(i,cellNumbers,cellId,distanceMatrix,threshold);
        cellId = cellId + 1;
    end
end

function cellNumbers = findNeighbors(i,cellNumbers,cellId,distanceMatrix,threshold)

for j = 1:numel(cellNumbers)
    if j ~= i && distanceMatrix(i,j) <= threshold && cellNumbers(j) == 0
        cellNumbers(j) = cellId;
        cellNumbers = findNeighbors(j,cellNumbers,cellId,distanceMatrix,threshold);
    end
end

end

end
