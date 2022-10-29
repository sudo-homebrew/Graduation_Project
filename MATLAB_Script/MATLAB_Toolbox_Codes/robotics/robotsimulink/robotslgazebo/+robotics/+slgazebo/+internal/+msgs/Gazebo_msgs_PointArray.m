classdef Gazebo_msgs_PointArray < robotics.slgazebo.internal.msgs.Gazebo_Msgs
%This function is for internal use only. It may be removed in the future.

%Gazebo_msgs_PointArray is the template for repeated point structure

%   Copyright 2020 The MathWorks, Inc.

    properties

        MessageType = 'gazebo_msgs/PointArray'
        % x with array of double
        x  double
        % y with array of double
        y  double
        % z with array of double
        z  double
    end

end
