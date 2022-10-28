classdef Gazebo_msgs_SetLinkLinearVelocity < robotics.slgazebo.internal.msgs.Gazebo_Msgs
%This function is for internal use only. It may be removed in the future.

%Gazebo_msgs_SetLinkLinearVelocity is the template for set link linear velocity message bus type
%It defines fields required to set linear velocity of link

%   Copyright 2020 The MathWorks, Inc.

    properties

        MessageType = 'gazebo_msgs/SetLinkLinearVelocity';
        % name of model
        model_name = ''
        % name of link
        link_name = ''
        % linear velocity of link to be set in [x, y, z] directions
        velocity robotics.slgazebo.internal.msgs.Gazebo_msgs_Point
        % time for set linear velocity in sec and nsec
        duration robotics.slgazebo.internal.msgs.Gazebo_msgs_Time

    end

    methods

        function obj = Gazebo_msgs_SetLinkLinearVelocity(~)

            vel = robotics.slgazebo.internal.msgs.Gazebo_msgs_Point;
            obj.velocity = vel;

            durationTime = robotics.slgazebo.internal.msgs.Gazebo_msgs_Time;
            durationTime.seconds = 0;
            durationTime.nano_seconds = 0;

            obj.duration = durationTime;

        end

    end

end
