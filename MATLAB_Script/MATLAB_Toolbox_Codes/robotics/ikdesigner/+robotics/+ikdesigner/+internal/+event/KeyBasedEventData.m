classdef (ConstructOnLoad) KeyBasedEventData < event.EventData
%This class is for internal use only and may be removed in a future release.

%KeyBasedEventData Event data for communicating changes where the only necessary identifier is a key
%   Many events just use a key to access the data to be addressed (e.g. a
%   delete event just needs to know what is going to be removed). This
%   event is a generic template for use in those situations.

%   Copyright 2021 The MathWorks, Inc.

    properties
        %Key
        Key
    end

    methods
        function data = KeyBasedEventData(key)
            data.Key = key;
        end
    end
end