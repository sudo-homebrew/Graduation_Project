classdef Gazebo_msgs_SetLinkAngularVelocity < robotics.slgazebo.internal.msgs.Gazebo_Msgs
%This function is for internal use only. It may be removed in the future.

%Gazebo_msgs_SetLinkAngularVelocity is the template for set link angular velocity message bus type
%It defines fields required to set link angular velocity

%   Copyright 2019-2020 The MathWorks, Inc.

    properties

        MessageType = 'gazebo_msgs/SetLinkAngularVelocity';
        % name of model
        model_name = ''
        % name of link
        link_name = ''
        % angular velocity of link to be set in [x, y, z] directions
        velocity robotics.slgazebo.internal.msgs.Gazebo_msgs_Point
        % time for set angular velocity in sec and nsec
        duration robotics.slgazebo.internal.msgs.Gazebo_msgs_Time

    end

    methods

        function obj = Gazebo_msgs_SetLinkAngularVelocity(~)

            vel = robotics.slgazebo.internal.msgs.Gazebo_msgs_Point;
            obj.velocity = vel;

            durationTime = robotics.slgazebo.internal.msgs.Gazebo_msgs_Time;
            durationTime.seconds = 0;
            durationTime.nano_seconds = 0;

            obj.duration = durationTime;

        end

    end

end
