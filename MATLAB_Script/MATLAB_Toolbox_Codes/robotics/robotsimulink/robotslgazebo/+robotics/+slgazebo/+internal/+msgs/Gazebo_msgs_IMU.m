classdef Gazebo_msgs_IMU < robotics.slgazebo.internal.msgs.Gazebo_Msgs
%This function is for internal use only. It may be removed in the future.

%Gazebo_msgs_IMU this is the template for IMU message bus type

%   Copyright 2019 The MathWorks, Inc.

    properties

        MessageType = 'gazebo_msgs/IMU'
        linear_acceleration robotics.slgazebo.internal.msgs.Gazebo_msgs_Point
        angular_velocity robotics.slgazebo.internal.msgs.Gazebo_msgs_Point
        orientation robotics.slgazebo.internal.msgs.Gazebo_msgs_Quaternion
    end

    methods

        function obj = Gazebo_msgs_IMU(~)

            acc = robotics.slgazebo.internal.msgs.Gazebo_msgs_Point;
            acc.x = 0;
            acc.y = 0;
            acc.z = 0;

            omega = robotics.slgazebo.internal.msgs.Gazebo_msgs_Point;
            omega.x = 0;
            omega.y = 0;
            omega.z = 0;

            orient = robotics.slgazebo.internal.msgs.Gazebo_msgs_Quaternion;

            orient.w = 1;
            orient.x = 0;
            orient.y = 0;
            orient.z = 0;

            obj.linear_acceleration = acc;
            obj.angular_velocity = omega;
            obj.orientation = orient;

        end

    end

end
