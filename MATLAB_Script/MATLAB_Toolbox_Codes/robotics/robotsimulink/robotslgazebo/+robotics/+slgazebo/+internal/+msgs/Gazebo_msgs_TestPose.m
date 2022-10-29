classdef Gazebo_msgs_TestPose < robotics.slgazebo.internal.msgs.Gazebo_Msgs
%This function is for internal use only. It may be removed in the future.

%Gazebo_msgs_TestPose defines the pose message
%Used to test the custom support

%   Copyright 2020 The MathWorks, Inc.

    properties

        MessageType = 'gazebo_msgs/TestPose';
        x = 0
        y = 0
        z = 0
        w = 1
    end

end
