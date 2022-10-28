classdef TrackHistory < fusion.internal.metrics.History
%

%   Copyright 2018 The MathWorks, Inc.

    methods
        function obj = TrackHistory(invalidTrackID,invalidTruthID)
            obj@fusion.internal.metrics.History(invalidTrackID,invalidTruthID);
        end
    end
        
    methods (Access = protected)
        function metrics = metricsConstructor(~, numMetrics,invalidTrackID,invalidTruthID)
            metrics = fusion.internal.metrics.TrackAssignment(numMetrics,invalidTrackID,invalidTruthID);
        end
        
        function generateContinuityError(~, resurrectedIDs) %#ok<INUSD>
            error(message('fusion:trackAssignmentMetrics:ResurrectedTrackIDs',evalc('disp(resurrectedIDs)')));
        end
        
        function validateInvalidIdentifier(obj, id)
            if ~(isnumeric(id) && isnumeric(obj.InvalidTrackID) || isstring(id) && isstring(obj.InvalidTrackID))
                error(message('fusion:trackAssignmentMetrics:MismatchedTrackIdentifier'));
            end
        end
    end
end
