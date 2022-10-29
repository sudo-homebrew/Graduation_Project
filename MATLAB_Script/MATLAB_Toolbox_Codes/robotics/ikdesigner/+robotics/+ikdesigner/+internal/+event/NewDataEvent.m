classdef (ConstructOnLoad) NewDataEvent < event.EventData
    %This function is for internal use only and may be removed in a future release

    %NewDataEvent Event object for adding data to the app

    %   Copyright 2021 The MathWorks, Inc.

    properties
        %Data Cell array of variables
        Data

        %DataNames Cell array of character vectors indicating intended name for each variable
        DataNames
    end

    methods
        function evtData = NewDataEvent(data, dataNames)
            evtData.Data = data;
            evtData.DataNames = dataNames;
        end
    end
end