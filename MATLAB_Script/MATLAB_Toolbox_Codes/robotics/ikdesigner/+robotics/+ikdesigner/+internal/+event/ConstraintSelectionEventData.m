classdef (ConstructOnLoad) ConstraintSelectionEventData < event.EventData
%This class is for internal use only and may be removed in a future release.

%ConstraintSelectionEventData Event data associated with constraint selection

%   Copyright 2021 The MathWorks, Inc.

    properties
        %ConstraintKey Constraints map key associated with a given constraint
        ConstraintKey
    end

    methods
        function data = ConstraintSelectionEventData(key)
            data.ConstraintKey = key;
        end
    end
end
