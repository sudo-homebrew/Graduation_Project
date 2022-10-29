function [minDist,xyObstacles] = clearance(points, map)
%This function is for internal use only. It may be removed in the future.

%CLEARANCE Compute the minimum distance between the query points and
%   obstacles. The min distance is precise upto sqrt(2) times grid map cell
%   size.
%
%   [MINDIST,XYOBSTACLES] = clearance(POINTS, MAP) returns minimum  
%   distances MINDIST, coordinates at minimum distances on obstacles
%   XYPTSOBS. MINDIST is 1-by-N vector, and XYOBSTACLES is a matrix of size
%   N-by-2 ([x y]) where is N number of  POINTS. POINTS is a matrix of size 
%   N-by-2 ([x y]) and MAP is either binaryOccupancyMap or occupancyMap.

% Copyright 2019-2020 The MathWorks, Inc.

%Validate points attribute
validateattributes(points, {'numeric'}, {'nonempty', 'ncols', 2},...
    'nav.algs.internal.clearance', 'poses',1);

%Validate map attribute
validateattributes(map,...
    {'binaryOccupancyMap', ...
    'occupancyMap'}, {'nonempty', 'scalar'}, ...
    'nav.algs.internal.clearance', 'map', 2);

[ObsCellDistance, obstCellIndices] = nearestObstacleDistance(map);

minDist = inf(1,size(points,1));
xyObstacles = nan(size(points,1),2);

% Return distances inf if environment is free.
if isinf(ObsCellDistance(1,1))
    return;
end

% Convert poses into grid cells.
gridCells = world2grid(map, points(:,1:2));

gridCellLinearInd = sub2ind(map.GridSize, gridCells(:,1), gridCells(:,2));

% Find nearest occupied grid cell index for gridCells(k)
[I, J] = ind2sub(size(obstCellIndices), obstCellIndices(gridCellLinearInd));

% Convert the occupied grid into world point.
xyObstacles = grid2world(map,[I,J]);

xyPts = grid2world(map,world2grid(map,points));

dis = xyObstacles - xyPts;

minDist = sqrt(sum(dis.*dis,2))';
end



function [nearestObstacleCellDistance, nearestObstacleCellIDs] = nearestObstacleDistance(map)

    outType = uint32(1);
    
    if isa(map, "binaryOccupancyMap")
        matrix = map.occupancyMatrix;
    else
        matrix = occupancyMatrix(map, 'ternary');
        matrix(matrix == -1) = 0;
        matrix = logical(matrix);
    end
    
    %Find the minimum distance for each grid cells and closest grid
    %cells indices. Size of nearestObstacleDistance and
    %nearestObstacleIDs are equal to size(matrix).
    [nearestObstacleCellDistance, nearestObstacleCellIDs] = ...
        imageslib.internal.bwdistComputeEDTFT(matrix, outType);
end
