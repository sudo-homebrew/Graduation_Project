function [isScanAccepted] = UpdateSLAM(scanMsg)
%UPDTESLAM Summary of this function goes here
%   Detailed explanation goes here
    global slamAlg
    
    if ~isa(scanMsg,'struct')
        isScanAccepted = 0;

        return
    end


    scan_temp = rosReadLidarScan(scanMsg);
    isScanAccepted = addScan(slamAlg, scan_temp);
end
