classdef TruthHistory < fusion.internal.metrics.History
%

%   Copyright 2018 The MathWorks, Inc.

    methods
        function obj = TruthHistory(invalidTrackID,invalidTruthID)
            obj@fusion.internal.metrics.History(invalidTrackID,invalidTruthID);
        end
    end
    
    methods (Access = protected)
        function metrics = metricsConstructor(~, numMetrics,invalidTrackID,invalidTruthID)
            metrics = fusion.internal.metrics.TruthAssignment(numMetrics,invalidTrackID,invalidTruthID);
        end
        
        function generateContinuityError(~, resurrectedIDs) %#ok<INUSD>
            error(message('fusion:trackAssignmentMetrics:ResurrectedTruthIDs',evalc('disp(resurrectedIDs)')));
        end
        
        function validateInvalidIdentifier(obj, id)
            if ~(isnumeric(id) && isnumeric(obj.InvalidTruthID) || isstring(id) && isstring(obj.InvalidTruthID))
                error(message('fusion:trackAssignmentMetrics:MismatchedTruthIdentifier'));
            end
        end
    end

end
