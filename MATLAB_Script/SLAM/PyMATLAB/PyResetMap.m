function  PyResetMap()
%PYRESETMAP Summary of this function goes here
%   Detailed explanation goes here
    global slamAlg

    maxLidarRange = 8;
    mapResolution = 20;
    maxNumScans = 360;
    slamAlg = lidarSLAM(mapResolution, maxLidarRange, maxNumScans);
end
