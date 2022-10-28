classdef (Abstract) View < handle
    %This function is for internal use only. It may be removed in the future.
    
    %VIEW
    
    %   Copyright 2021 The MathWorks, Inc.
    
    properties (Constant)
        EventName = "ViewChanged"
    end

    events
        ViewChanged
    end

    methods (Abstract)
        %initialize Prepare view states to initial values
        initialize(obj)
    end
end
