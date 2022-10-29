classdef (ConstructOnLoad) ConstraintStateData < event.EventData
%This class is for internal use only and may be removed in a future release.

%ConstraintStateData Event data for communicating constraint state

%   Copyright 2021-2022 The MathWorks, Inc.

    properties
        %UpdateType Enumeration that indicates how the constraint state is changing in this event
        UpdateType

        %ConstraintsMap Handle to the map containing all the constraints
        ConstraintsMap

        %ChangedKeys Keys of data that was changed
        ChangedKeys
    end

    methods
        function data = ConstraintStateData(updateType, constraintsMap, changedKeys)

            data.UpdateType = updateType;
            data.ConstraintsMap = constraintsMap;
            data.ChangedKeys = changedKeys;
        end
    end
end
