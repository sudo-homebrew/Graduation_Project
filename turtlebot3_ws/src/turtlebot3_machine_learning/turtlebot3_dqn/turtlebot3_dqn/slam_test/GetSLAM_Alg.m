function [result] = GetSLAM_Alg()
%RETURNSLAM_ALG Summary of this function goes here
%   Detailed explanation goes here
    global slamAlg

    maxLidarRange = 8;
    mapResolution = 20;
    maxNumScans = 360;
    slamAlg = lidarSLAM(mapResolution, maxLidarRange, maxNumScans);
    
    
    slamAlg.LoopClosureThreshold = 210;  
    slamAlg.LoopClosureSearchRadius = 8;

    result = 1;
end

