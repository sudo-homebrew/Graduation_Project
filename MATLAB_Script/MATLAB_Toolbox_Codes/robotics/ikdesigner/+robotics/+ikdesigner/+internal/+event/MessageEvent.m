classdef (ConstructOnLoad) MessageEvent < event.EventData
    %This function is for internal use only and may be removed in a future release

    %MessageEvent Data to communicate messages for UI alerts

    %   Copyright 2021 The MathWorks, Inc.

    properties (SetAccess = private)
        %String Message string
        String
    end

    methods
        function evtData = MessageEvent(msgString)
            evtData.String = msgString;
        end
    end
end