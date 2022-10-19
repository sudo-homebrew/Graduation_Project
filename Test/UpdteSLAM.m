function [Neu_slamAlg, isScanAccepted] = UpdteSLAM(slamAlg, scanMsg)
%UPDTESLAM Summary of this function goes here
%   Detailed explanation goes here
    if ~isa(scanMsg,'struct')
        Neu_slamAlg = slamAlg;
    end


    scan_temp = rosReadLidarScan(scanMsg);
    isScanAccepted = addScan(slamAlg, scan_temp);
    Neu_slamAlg = slamAlg;
end

