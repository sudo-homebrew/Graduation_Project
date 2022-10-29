classdef History < handle
%

%   Copyright 2018 The MathWorks, Inc.
    
    properties
        IdentifierFcn
    end
        
    properties
        Current
        Deleted
    end

    properties (Access = protected)
        Previous
        InvalidTrackID
        InvalidTruthID
    end
    
    methods (Abstract, Access = protected)
        metrics = metricsConstructor(obj, numMetrics, invalidTrackID, invalidTruthID)
        generateContinuityError(obj, resurrectedIDs)
        validateInvalidIdentifier(obj, id)
    end
    
    methods
        function obj = History(invalidTrackID,invalidTruthID)
            reset(obj, invalidTrackID, invalidTruthID)
        end
        
        function reset(obj, invalidTrackID, invalidTruthID)
            obj.InvalidTrackID = invalidTrackID;
            obj.InvalidTruthID = invalidTruthID;
            obj.Deleted = metricsConstructor(obj,0,invalidTrackID,invalidTruthID);
            obj.Previous = metricsConstructor(obj,0,invalidTrackID,invalidTruthID);
            obj.Current = metricsConstructor(obj,0,invalidTrackID,invalidTruthID);
        end
        
        function [sortedIdx, iDeleted, iExisting, iNew] = init(obj, array)
            % find supplied id's and arrange in sorted order
            arrayID = getArrayIdentifer(obj, array(:));
            [sortedID, sortedIdx] = sort(arrayID(:));
                            
            % create new metrics for current tracker update
            obj.Current = metricsConstructor(obj, numel(arrayID), obj.InvalidTrackID, obj.InvalidTruthID);
            setIdentifier(obj.Current, sortedID(:));
            
            % copy properties from pre-existing entries
            [~,iPrev,iExisting] = intersect(getIdentifier(obj.Previous), getIdentifier(obj.Current));
            copyIndexedProperties(obj.Current, iExisting, obj.Previous, iPrev);

            % append deleted tracks to deletion list
            [~,iRemoved,iNew] = setxor(getIdentifier(obj.Previous), getIdentifier(obj.Current));
            iDeleted = appendIndexedProperties(obj.Deleted, obj.Previous, iRemoved);
            
            % sanity-check against the recurrence of an expired entry
            validateNewEntries(obj, sortedID(iNew), getIdentifier(obj.Deleted));
        end
        
        function age(obj)
            computeCumulativeMetrics(obj.Current);
            obj.Previous = obj.Current;
        end
        
        function tm = summary(obj)
            tm = buildSummary(obj.Current, obj.Deleted);
        end
        
        function tm = metricsTable(obj)
           tm = metricsTable(obj.Current, obj.Deleted);
        end
        
        function validateNewEntries(obj, newIDs, deletedIDs)
            isResurrected = ismember(newIDs, deletedIDs);
            if any(isResurrected)
                generateContinuityError(obj, newIDs(isResurrected));
            end
        end
        
        function s = saveHistory(obj)
            s.Current = obj.Current;
            s.Previous = obj.Previous;
            s.Deleted = obj.Deleted;
        end
        
        function loadHistory(obj, s)
            obj.Current = s.Current;
            obj.Previous = s.Previous;
            obj.Deleted = s.Deleted;
        end
    end

    methods (Access = protected)
        function id = getArrayIdentifer(obj, array)
            id = obj.IdentifierFcn(array);
            if numel(id) ~= numel(array) || ~isstring(id) && ~isnumeric(id)
                error(message('fusion:trackAssignmentMetrics:InvalidIDResult',char(obj.IdentifierFcn)));
            end
            validateInvalidIdentifier(obj, id);
        end
    end
end
