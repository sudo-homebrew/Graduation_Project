classdef Gazebo_msgs_Axis < robotics.slgazebo.internal.msgs.Gazebo_Msgs
% This function is for internal use only. It may be removed in the future.
% DO NOT EDIT!
% Gazebo_msgs_Axis is the template for custom Axis message bus type
%It defines fields required to read axis details for joint state

%   Copyright 2020 The MathWorks, Inc.

    properties

        MessageType = 'gazebo_msgs/Axis';
        % xyz components of axis unit vector
        xyz robotics.slgazebo.internal.msgs.Gazebo_msgs_PointArray
        % lower joint limit (in radians or meters)
        limit_lower double
        % upper joint limit (in radians or meters)
        limit_upper double
        % limit of maximum joint efforts
        limit_effort double
        % limit of maximum joint velocity
        limit_velocity double
        % joint damping coefficient
        damping double
        % joint friction value
        friction double
        % true if xyz elements are in joint frame
        use_parent_model_frame logical

    end

    methods

        function obj = Gazebo_msgs_Axis(~)

            xyz_ = robotics.slgazebo.internal.msgs.Gazebo_msgs_PointArray;
            obj.xyz  =  xyz_;

        end

    end
end
