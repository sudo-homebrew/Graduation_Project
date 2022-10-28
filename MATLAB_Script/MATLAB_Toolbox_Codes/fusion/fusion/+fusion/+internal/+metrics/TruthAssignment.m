classdef TruthAssignment < fusion.internal.metrics.Assignment
%

%   Copyright 2018 The MathWorks, Inc.

    properties
        TruthID
        TrackID
        TotalLength
        BreakStatus
        BreakCount
        BreakLength
        DetectableStatus
        EstablishmentStatus
        EstablishmentCount
        EstablishmentLength
    end
    
    methods
        function obj = TruthAssignment(n, invalidTrackID, invalidTruthID)
            obj.TruthID             = repmat(invalidTruthID,n,1);
            obj.TrackID             = repmat(invalidTrackID,n,1);
            obj.TotalLength         = zeros(n,1);
            obj.BreakStatus         = false(n,1);
            obj.BreakCount          = zeros(n,1);
            obj.BreakLength         = zeros(n,1);
            obj.DetectableStatus    = false(n,1);
            obj.EstablishmentStatus = false(n,1);
            obj.EstablishmentCount  = zeros(n,1);
            obj.EstablishmentLength = zeros(n,1);
        end
        
        function tm = metricsTable(obj, deleted)
            truthID = vertcat(obj.TruthID,deleted.TruthID);
            trackID = vertcat(obj.TrackID,deleted.TrackID);
            surviving = vertcat(true(size(obj.TotalLength)),false(size(deleted.TotalLength)));
            totalLength = vertcat(obj.TotalLength,deleted.TotalLength);
            breakStatus = vertcat(obj.BreakStatus,deleted.BreakStatus);
            breakCount = vertcat(obj.BreakCount,deleted.BreakCount);
            breakLength = vertcat(obj.BreakLength,deleted.BreakLength);
            inCoverageArea = vertcat(obj.DetectableStatus,deleted.DetectableStatus);
            establishmentStatus = vertcat(obj.EstablishmentStatus,deleted.EstablishmentStatus);
            establishmentLength = vertcat(obj.EstablishmentLength,deleted.EstablishmentLength);
            
            [~,iTruth] = sort(truthID);
            
            tm = struct2table(struct( ...
                'TruthID', truthID(iTruth), ...
                'AssociatedTrackID', trackID(iTruth), ...
                'DeletionStatus', ~surviving(iTruth), ...
                'TotalLength', totalLength(iTruth), ...
                'BreakStatus', breakStatus(iTruth), ...
                'BreakCount', breakCount(iTruth), ...
                'BreakLength', breakLength(iTruth), ...
                'InCoverageArea', inCoverageArea(iTruth), ...
                'EstablishmentStatus', establishmentStatus(iTruth), ...
                'EstablishmentLength', establishmentLength(iTruth)));
        end
        
        function tm = buildSummary(obj, deleted)
            
            % report establishment length only for established truths
            oES = obj.EstablishmentStatus;
            dES = deleted.EstablishmentStatus;
            
            tm = struct( ...
                'TotalNumTruths', numel(obj.TruthID)+numel(deleted.TruthID), ...
                'NumMissingTruths', sum(~oES)+sum(~dES), ...
                'MaxEstablishmentLength', zeromax(obj.EstablishmentLength(oES), deleted.EstablishmentLength(dES)), ...
                'TotalEstablishmentLength', sum(obj.EstablishmentLength(oES)) + sum(deleted.EstablishmentLength(dES)), ...
                'MaxBreakCount',zeromax(obj.BreakCount,deleted.BreakCount), ...
                'TotalBreakCount', sum(obj.BreakCount) + sum(deleted.BreakCount), ...
                'MaxBreakLength',zeromax(obj.BreakLength, deleted.BreakLength), ...
                'TotalBreakLength',sum(obj.BreakLength)+sum(deleted.BreakLength));
        end
        
        function copyIndexedProperties(obj, iProps, src, iSrc)
            obj.TruthID(iProps)             = src.TruthID(iSrc);
            obj.TrackID(iProps)             = src.TrackID(iSrc);
            obj.TotalLength(iProps)         = src.TotalLength(iSrc);
            obj.BreakStatus(iProps)         = src.BreakStatus(iSrc);
            obj.BreakCount(iProps)          = src.BreakCount(iSrc);
            obj.BreakLength(iProps)         = src.BreakLength(iSrc);
            obj.DetectableStatus(iProps)    = src.DetectableStatus(iSrc);
            obj.EstablishmentStatus(iProps) = src.EstablishmentStatus(iSrc);
            obj.EstablishmentCount(iProps)  = src.EstablishmentCount(iSrc);
            obj.EstablishmentLength(iProps) = src.EstablishmentLength(iSrc);
        end
        
        function iAppended = appendIndexedProperties(obj, src, iSrc)
            iAppended = (numel(obj.TruthID) + (1:numel(iSrc)))';
            obj.TruthID             = vertcat(obj.TruthID,             src.TruthID(iSrc));
            obj.TrackID             = vertcat(obj.TrackID,             src.TrackID(iSrc));
            obj.TotalLength         = vertcat(obj.TotalLength,         src.TotalLength(iSrc));
            obj.BreakStatus         = vertcat(obj.BreakStatus,         src.BreakStatus(iSrc));
            obj.BreakCount          = vertcat(obj.BreakCount,          src.BreakCount(iSrc));
            obj.BreakLength         = vertcat(obj.BreakLength,         src.BreakLength(iSrc));
            obj.DetectableStatus    = vertcat(obj.DetectableStatus,    src.DetectableStatus(iSrc));
            obj.EstablishmentStatus = vertcat(obj.EstablishmentStatus, src.EstablishmentStatus(iSrc));
            obj.EstablishmentCount  = vertcat(obj.EstablishmentCount,  src.EstablishmentCount(iSrc));
            obj.EstablishmentLength = vertcat(obj.EstablishmentLength, src.EstablishmentLength(iSrc));
        end
        
        function computeCumulativeMetrics(obj)
            obj.TotalLength = obj.TotalLength + 1;
            
            % break length
            iBreak = find(obj.BreakStatus);
            obj.BreakLength(iBreak) = obj.BreakLength(iBreak) + 1;
            
            % establishment length
            iUnestablished = find(~obj.EstablishmentStatus);
            obj.EstablishmentLength(iUnestablished) = obj.EstablishmentLength(iUnestablished) + obj.DetectableStatus(iUnestablished);
        end
        
        function setIdentifier(obj, id)
            obj.TruthID = id;
        end
        
        function id = getIdentifier(obj)
            id = obj.TruthID;
        end        

    end
end
