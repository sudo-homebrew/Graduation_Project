classdef TrackAssignment < fusion.internal.metrics.Assignment
%

%   Copyright 2018-2019 The MathWorks, Inc.

    properties
        TrackID
        TruthID
        TotalLength
        DeletionStatus
        DeletionLength
        DivergenceStatus
        DivergenceCount
        DivergenceLength
        RedundancyStatus
        RedundancyCount
        RedundancyLength
        FalseStatus
        FalseLength
        SwapCount
    end
    
    methods
        function obj = TrackAssignment(n, invalidTrackID, invalidTruthID)
            obj.TrackID          = repmat(invalidTrackID,n,1);
            obj.TruthID          = repmat(invalidTruthID,n,1);
            obj.TotalLength      = zeros(n,1);
            obj.DeletionStatus   = false(n,1);
            obj.DeletionLength   = zeros(n,1);
            obj.DivergenceStatus = false(n,1);
            obj.DivergenceCount  = zeros(n,1);
            obj.DivergenceLength = zeros(n,1);
            obj.RedundancyStatus = false(n,1);
            obj.RedundancyCount  = zeros(n,1);
            obj.RedundancyLength = zeros(n,1);
            obj.FalseStatus      = true(n,1);
            obj.FalseLength      = zeros(n,1);
            obj.SwapCount        = zeros(n,1);
        end
        
        function tm = metricsTable(obj, deleted)
            trackID = vertcat(obj.TrackID,deleted.TrackID);
            truthID = vertcat(obj.TruthID,deleted.TruthID);
            surviving = vertcat(true(size(obj.TotalLength)),false(size(deleted.TotalLength)));
            totalLength = vertcat(obj.TotalLength,deleted.TotalLength);
            deletionStatus = vertcat(obj.DeletionStatus,deleted.DeletionStatus);
            deletionLength = vertcat(obj.DeletionLength,deleted.DeletionLength);
            divergenceStatus = vertcat(obj.DivergenceStatus,deleted.DivergenceStatus);
            divergenceCount = vertcat(obj.DivergenceCount,deleted.DivergenceCount);
            divergenceLength = vertcat(obj.DivergenceLength,deleted.DivergenceLength);
            redundancyStatus = vertcat(obj.RedundancyStatus,deleted.RedundancyStatus);
            redundancyCount = vertcat(obj.RedundancyCount,deleted.RedundancyCount);
            redundancyLength = vertcat(obj.RedundancyLength,deleted.RedundancyLength);
            falseStatus = vertcat(obj.FalseStatus,deleted.FalseStatus);
            falseLength = vertcat(obj.FalseLength,deleted.FalseLength);
            swapCount = vertcat(obj.SwapCount,deleted.SwapCount);
            [~,iTracks] = sort(trackID);
            tm = struct2table(struct( ...
                'TrackID', trackID(iTracks), ...
                'AssignedTruthID', truthID(iTracks), ...
                'Surviving', surviving(iTracks), ...
                'TotalLength', totalLength(iTracks), ...
                'DeletionStatus', deletionStatus(iTracks), ...
                'DeletionLength', deletionLength(iTracks), ...
                'DivergenceStatus', divergenceStatus(iTracks), ...
                'DivergenceCount', divergenceCount(iTracks), ...
                'DivergenceLength', divergenceLength(iTracks), ...
                'RedundancyStatus', redundancyStatus(iTracks), ...
                'RedundancyCount', redundancyCount(iTracks), ...
                'RedundancyLength', redundancyLength(iTracks), ...
                'FalseTrackStatus', falseStatus(iTracks), ...
                'FalseTrackLength', falseLength(iTracks), ...
                'SwapCount', swapCount(iTracks)));
        end
        
        function tm = buildSummary(obj, deleted)
            tm = struct( ...
                'TotalNumTracks', numel(obj.TrackID)+numel(deleted.TrackID), ...
                'NumFalseTracks', sum(obj.FalseStatus)+sum(deleted.FalseStatus), ...
                'MaxSwapCount',  zeromax(obj.SwapCount,deleted.SwapCount), ...
                'TotalSwapCount', sum(obj.SwapCount)+sum(deleted.SwapCount), ...
                'MaxDivergenceCount',zeromax(obj.DivergenceCount,deleted.DivergenceCount), ...
                'TotalDivergenceCount',sum(obj.DivergenceCount)+sum(deleted.DivergenceCount), ...
                'MaxDivergenceLength',zeromax(obj.DivergenceLength,deleted.DivergenceLength), ...
                'TotalDivergenceLength',sum(obj.DivergenceLength)+sum(deleted.DivergenceLength), ...
                'MaxRedundancyCount',zeromax(obj.RedundancyCount,deleted.RedundancyCount), ...
                'TotalRedundancyCount',sum(obj.RedundancyCount)+sum(deleted.RedundancyCount), ...
                'MaxRedundancyLength',zeromax(obj.RedundancyLength,deleted.RedundancyLength), ...
                'TotalRedundancyLength',sum(obj.RedundancyLength)+sum(deleted.RedundancyLength));
        end
        
        function copyIndexedProperties(obj, iProps, src, iSrc)
            obj.TrackID(iProps)          = src.TrackID(iSrc) ;
            obj.TruthID(iProps)          = src.TruthID(iSrc);
            obj.TotalLength(iProps)      = src.TotalLength(iSrc);
            obj.DeletionStatus(iProps)   = src.DeletionStatus(iSrc);
            obj.DeletionLength(iProps)   = src.DeletionLength(iSrc);
            obj.DivergenceStatus(iProps) = src.DivergenceStatus(iSrc);
            obj.DivergenceCount(iProps)  = src.DivergenceCount(iSrc);
            obj.DivergenceLength(iProps) = src.DivergenceLength(iSrc);
            obj.RedundancyStatus(iProps) = src.RedundancyStatus(iSrc);  
            obj.RedundancyCount(iProps)  = src.RedundancyCount(iSrc);
            obj.RedundancyLength(iProps) = src.RedundancyLength(iSrc);
            obj.FalseStatus(iProps)      = src.FalseStatus(iSrc);
            obj.FalseLength(iProps)      = src.FalseLength(iSrc);
            obj.SwapCount(iProps)        = src.SwapCount(iSrc);
        end
        
        function iAppended = appendIndexedProperties(obj, src, iSrc)
            iAppended = (numel(obj.TrackID) + (1:numel(iSrc)))';
            obj.TrackID          = vertcat(obj.TrackID,          src.TrackID(iSrc));
            obj.TruthID          = vertcat(obj.TruthID,          src.TruthID(iSrc));
            obj.TotalLength      = vertcat(obj.TotalLength,      src.TotalLength(iSrc));
            obj.DeletionStatus   = vertcat(obj.DeletionStatus,   src.DeletionStatus(iSrc));
            obj.DeletionLength   = vertcat(obj.DeletionLength,   src.DeletionLength(iSrc));
            obj.DivergenceStatus = vertcat(obj.DivergenceStatus, src.DivergenceStatus(iSrc));
            obj.DivergenceCount  = vertcat(obj.DivergenceCount,  src.DivergenceCount(iSrc));
            obj.DivergenceLength = vertcat(obj.DivergenceLength, src.DivergenceLength(iSrc));
            obj.RedundancyStatus = vertcat(obj.RedundancyStatus, src.RedundancyStatus(iSrc));  
            obj.RedundancyCount  = vertcat(obj.RedundancyCount,  src.RedundancyCount(iSrc));
            obj.RedundancyLength = vertcat(obj.RedundancyLength, src.RedundancyLength(iSrc));
            obj.FalseStatus      = vertcat(obj.FalseStatus,      src.FalseStatus(iSrc));
            obj.FalseLength      = vertcat(obj.FalseLength,      src.FalseLength(iSrc));
            obj.SwapCount        = vertcat(obj.SwapCount,        src.SwapCount(iSrc));
        end
        
        function computeCumulativeMetrics(obj)
            obj.TotalLength = obj.TotalLength + 1;
            
            % redundancy length
            iRedundancy = find(obj.RedundancyStatus);
            obj.RedundancyLength(iRedundancy) = obj.RedundancyLength(iRedundancy) + 1;
            
            % deletion length
            iDeleted = find(obj.DeletionStatus);
            obj.DeletionLength(iDeleted) = obj.DeletionLength(iDeleted) + 1;
            
            % divergence length
            iDivergent = find(obj.DivergenceStatus);
            obj.DivergenceLength(iDivergent) = obj.DivergenceLength(iDivergent) + 1;
            
            % false length
            iFalse = find(obj.FalseStatus);
            obj.FalseLength(iFalse) = obj.FalseLength(iFalse) + 1;
        end
        
        function setIdentifier(obj, id)
            obj.TrackID = id;
        end
        
        function id = getIdentifier(obj)
            id = obj.TrackID;
        end
    end    
end
