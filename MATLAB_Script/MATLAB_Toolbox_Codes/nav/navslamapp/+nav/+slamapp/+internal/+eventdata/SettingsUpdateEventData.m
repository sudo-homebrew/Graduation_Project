classdef (ConstructOnLoad) SettingsUpdateEventData < event.EventData
%This class is for internal use only. It may be removed in the future.

%SettingsUpdateEventData This class serves the listeners waiting for a
%    SettingsDialog_SendUserUpdate event

%   Copyright 2018 The MathWorks, Inc.

    properties
        Tag
        NumericStr
        Value
    end

    methods
        function data = SettingsUpdateEventData(tag, numericStr, val)
        %SettingsUpdateEventData

            data.Tag = tag;
            data.NumericStr = numericStr;
            data.Value = val;

        end
    end
end
