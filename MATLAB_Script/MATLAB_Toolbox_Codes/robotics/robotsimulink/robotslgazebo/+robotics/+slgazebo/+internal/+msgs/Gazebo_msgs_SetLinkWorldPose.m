classdef Gazebo_msgs_SetLinkWorldPose < robotics.slgazebo.internal.msgs.Gazebo_Msgs
%This function is for internal use only. It may be removed in the future.

%Gazebo_msgs_SetLinkWorldPose is the template for set link world pose message bus type
%It defines fields required to set link world pose

%   Copyright 2020 The MathWorks, Inc.

    properties

        MessageType = 'gazebo_msgs/SetLinkWorldPose';
        % name of model
        model_name = ''
        % name of link
        link_name = ''
        % world pose of link to be set in position and orientation format
        world_pose robotics.slgazebo.internal.msgs.Gazebo_msgs_Pose
        % time for set link world pose in sec and nsec
        duration robotics.slgazebo.internal.msgs.Gazebo_msgs_Time

    end

    methods

        function obj = Gazebo_msgs_SetLinkWorldPose(~)

            world_pose = robotics.slgazebo.internal.msgs.Gazebo_msgs_Pose;
            obj.world_pose = world_pose;

            durationTime = robotics.slgazebo.internal.msgs.Gazebo_msgs_Time;
            durationTime.seconds = 0;
            durationTime.nano_seconds = 0;

            obj.duration = durationTime;

        end

    end

end
