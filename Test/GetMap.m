function [map] = GetMap(slamAlg, scanMsge)
%GETMAP Summary of this function goes here
%   Detailed explanation goes here
    scan_temp = rosReadLidarScan(scanMsge);
    [scans, optimizedPoses]  = scansAndPoses(slamAlg);
    map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);
end

