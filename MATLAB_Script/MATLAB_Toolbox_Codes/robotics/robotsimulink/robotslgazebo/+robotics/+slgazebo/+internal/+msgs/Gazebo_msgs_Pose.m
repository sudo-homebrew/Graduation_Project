classdef Gazebo_msgs_Pose < robotics.slgazebo.internal.msgs.Gazebo_Msgs
%This function is for internal use only. It may be removed in the future.

%Gazebo_msgs_Pose is the template for Pose in 3D space

%   Copyright 2019 The MathWorks, Inc.

    properties

        MessageType = 'gazebo_msgs/Pose';
        position robotics.slgazebo.internal.msgs.Gazebo_msgs_Point
        orientation robotics.slgazebo.internal.msgs.Gazebo_msgs_Quaternion

    end

    methods

        function obj = Gazebo_msgs_Pose(~)

            point = robotics.slgazebo.internal.msgs.Gazebo_msgs_Point;
            point.x = 0;
            point.y = 0;
            point.z = 0;

            orient = robotics.slgazebo.internal.msgs.Gazebo_msgs_Quaternion;

            orient.w = 1;
            orient.x = 0;
            orient.y = 0;
            orient.z = 0;

            obj.position = point;
            obj.orientation = orient;

        end

    end

end
