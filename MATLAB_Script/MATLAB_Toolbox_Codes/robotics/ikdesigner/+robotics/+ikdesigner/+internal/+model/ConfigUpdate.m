classdef ConfigUpdate < uint8
%This class is for internal use only and may be removed in a future release

%ConfigUpdate Enumeration class for the configurations model update action
%   The configurations update is used to describe how an event is changing
%   the set of stored configurations

%   Copyright 2021 The MathWorks, Inc.
    
    enumeration
        %MapValuesChange The values in the configurations map are being changed
        MapValuesChange  (0)

        %ValuesAdded Configurations are added to the map
        ValuesAdded     (1)

        %ValuesRemoved Configurations are removed from the map
        ValuesRemoved   (2)
    end
end

