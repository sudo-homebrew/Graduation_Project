classdef Gazebo_msgs_LaserScan < robotics.slgazebo.internal.msgs.Gazebo_Msgs
%This function is for internal use only. It may be removed in the future.

%Gazebo_msgs_LaserScan this is the template for Laser Scan message bus type

%   Copyright 2019 The MathWorks, Inc.

    properties

        MessageType = 'gazebo_msgs/LaserScan'
        angle_min = 0
        angle_max = 0
        angle_step = 0
        range_min = 0
        range_max = 0
        count = 0
        vertical_angle_min = 0
        vertical_angle_max = 0
        vertical_angle_step = 0
        range double
        intensities double
    end

end
