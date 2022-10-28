classdef (ConstructOnLoad) ConfigTableEventData < event.EventData
%This class is for internal use only and may be removed in a future release.

%ConfigTableEventData Event data for communicating stored changes on items in the configurations table
%   This configurations table is a user-visible display of ordered, stored
%   configuration data. This event is used for communicating changes
%   specifically about the table or its contents in the context of the
%   table view (and not just individual configurations within the table).

%   Copyright 2021 The MathWorks, Inc.

    properties
        %AffectedKeys
        AffectedKeys

        %NewRowIndices
        NewRowIndices

        %NewName
        NewName

        %NewConfig
        NewConfig

        ValueChanged
    end

    methods
        function data = ConfigTableEventData(affectedKeys, updatedRowIndices)

            data.AffectedKeys = affectedKeys;
            data.NewRowIndices = updatedRowIndices;
        end
    end
end