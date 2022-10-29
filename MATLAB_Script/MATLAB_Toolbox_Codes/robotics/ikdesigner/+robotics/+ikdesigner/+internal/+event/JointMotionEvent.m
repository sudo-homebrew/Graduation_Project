classdef (ConstructOnLoad) JointMotionEvent < event.EventData
%This class is for internal use only and may be removed in a future release.

%JointMotionEvent Event for moving the robot joint configuration

%   Copyright 2021 The MathWorks, Inc.

    properties (SetAccess = private)
        %JointIndex
        JointIndex

        %Value
        Value
    end

    methods
        function data = JointMotionEvent(idx, value)
            data.JointIndex = idx;
            data.Value = value;
        end
    end
end