classdef (ConstructOnLoad) FilePathEvent < event.EventData
    %This function is for internal use only and may be removed in a future release

    %LoadSaveSessionEvent Data to communicate file paths for save/load events

    %   Copyright 2021 The MathWorks, Inc.

    properties (SetAccess = private)
        %FilePath File path string
        FilePath
    end

    methods
        function evtData = FilePathEvent(filepath)
            evtData.FilePath = filepath;
        end
    end
end