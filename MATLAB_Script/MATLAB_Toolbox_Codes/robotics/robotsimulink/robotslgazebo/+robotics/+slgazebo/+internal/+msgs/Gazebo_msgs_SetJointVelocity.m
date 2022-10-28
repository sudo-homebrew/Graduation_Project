classdef Gazebo_msgs_SetJointVelocity < robotics.slgazebo.internal.msgs.Gazebo_Msgs
%This function is for internal use only. It may be removed in the future.

%Gazebo_msgs_SetJointVelocity is the template for set joint velocity message bus type
%It defines fields required to set joint velocity

%   Copyright 2020 The MathWorks, Inc.

    properties

        MessageType = 'gazebo_msgs/SetJointVelocity';
        % name of model
        model_name = ''
        % name of joint
        joint_name = ''
        % axis number(index) of joint
        index uint32 = 0
        % velocity of joint to be set ( in m/sec or rad/sec)
        velocity = 0
        % time for set joint velocity in sec and nsec
        duration robotics.slgazebo.internal.msgs.Gazebo_msgs_Time

    end

    methods

        function obj = Gazebo_msgs_SetJointVelocity(~)

            durationTime = robotics.slgazebo.internal.msgs.Gazebo_msgs_Time;
            durationTime.seconds = 0;
            durationTime.nano_seconds = 0;

            obj.duration = durationTime;

        end

    end

end
