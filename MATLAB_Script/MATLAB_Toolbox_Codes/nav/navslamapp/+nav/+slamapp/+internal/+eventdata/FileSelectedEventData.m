classdef (ConstructOnLoad) FileSelectedEventData < event.EventData
%This class is for internal use only. It may be removed in the future.

%FileSelectedEventData

%   Copyright 2018 The MathWorks, Inc.

    properties
        FileName
    end

    methods
        function data = FileSelectedEventData(fileName)

            data.FileName = fileName;

        end
    end
end
