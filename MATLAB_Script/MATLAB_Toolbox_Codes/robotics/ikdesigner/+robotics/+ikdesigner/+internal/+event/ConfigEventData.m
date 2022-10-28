classdef (ConstructOnLoad) ConfigEventData < event.EventData
%This class is for internal use only and may be removed in a future release.

%ConfigEventData Event data for communicating stored configuration changes

%   Copyright 2021 The MathWorks, Inc.

    properties
        %Map Handle to the map containing all the configurations
        Map

        %UpdateType Enumeration indicating the type of change
        UpdateType

        %AffectedKeys If applicable, specific keys affected by the event
        AffectedKeys = string.empty;
    end

    methods
        function data = ConfigEventData(updateType, mapHandle)

            data.UpdateType = updateType;
            data.Map = mapHandle;
        end
    end
end