function [occMatrix] = GetMap()
%GETMAP Summary of this function goes here
%   Detailed explanation goes here
    global slamAlg
    
    maxLidarRange = 8;
    mapResolution = 20;
    
    [scans, optimizedPoses]  = scansAndPoses(slamAlg);
    map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);
    occMatrix = getOccupancy(map);
end

