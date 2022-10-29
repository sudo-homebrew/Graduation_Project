classdef Gazebo_msgs_SetJointPosition < robotics.slgazebo.internal.msgs.Gazebo_Msgs
%This function is for internal use only. It may be removed in the future.

%Gazebo_msgs_SetJointPosition is the template for set joint position message bus type
%It defines fields required to set joint position

%   Copyright 2020 The MathWorks, Inc.

    properties

        MessageType = 'gazebo_msgs/SetJointPosition';
        % name of model
        model_name = ''
        % name of joint
        joint_name = ''
        % axis number(index) of joint
        index uint32 = 0
        % position(angle) of joint to be set (in meter/radian)
        position = 0
        % time for set joint position in sec and nsec
        duration robotics.slgazebo.internal.msgs.Gazebo_msgs_Time

    end

    methods

        function obj = Gazebo_msgs_SetJointPosition(~)

            durationTime = robotics.slgazebo.internal.msgs.Gazebo_msgs_Time;
            durationTime.seconds = 0;
            durationTime.nano_seconds = 0;

            obj.duration = durationTime;

        end

    end

end
