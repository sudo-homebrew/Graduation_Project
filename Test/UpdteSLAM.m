function [slamAlg, isScanAccepted] = UpdteSLAM(slamAlg, scanMsge)
%UPDTESLAM Summary of this function goes here
%   Detailed explanation goes here
    scan_temp = rosReadLidarScan(scanMsge);
    isScanAccepted = addScan(slamAlg, scan_temp);
end

