function [slamAlg] = GetSLAM_Alg()
%RETURNSLAM_ALG Summary of this function goes here
%   Detailed explanation goes here
    maxLidarRange = 8;
    mapResolution = 20;
    maxNumScans = 360;
    slamAlg = lidarSLAM(mapResolution, maxLidarRange, maxNumScans);
    
    
    slamAlg.LoopClosureThreshold = 210;  
    slamAlg.LoopClosureSearchRadius = 8;
end
