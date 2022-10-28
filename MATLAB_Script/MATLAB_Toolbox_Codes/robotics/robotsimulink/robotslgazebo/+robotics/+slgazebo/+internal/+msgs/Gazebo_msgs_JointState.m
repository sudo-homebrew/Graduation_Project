classdef Gazebo_msgs_JointState < robotics.slgazebo.internal.msgs.Gazebo_Msgs
% This function is for internal use only. It may be removed in the future.
% DO NOT EDIT!
% Gazebo_msgs_JointState is the template for JointState message bus type
%It defines fields required to read joint state

%   Copyright 2020 The MathWorks, Inc.

    properties

        MessageType = 'gazebo_msgs/JointState';
        % name of model
        model_name uint8
        % name of joint
        joint_name uint8
        % joint id
        joint_id
        % positions of joint per axis
        joint_position double
        % velocities of joint per axis
        joint_velocity double
        % joint type
        joint_type
        % name of parent of joint
        parent_name uint8
        % id of parent of joint
        parent_id
        % name of child of joint
        child_name uint8
        % id of child of joint
        child_id
        % initial anchor pose of joint
        initial_anchor_pose robotics.slgazebo.internal.msgs.Gazebo_msgs_Pose
        % absolute world pose of joint
        world_pose robotics.slgazebo.internal.msgs.Gazebo_msgs_Pose
        % anchor pose on parent link of joint relative to world frame
        parent_world_pose robotics.slgazebo.internal.msgs.Gazebo_msgs_Pose
        % axis details
        axis robotics.slgazebo.internal.msgs.Gazebo_msgs_Axis

    end

    methods

        function obj = Gazebo_msgs_JointState(~)

            obj.joint_id = uint32(0);
            obj.joint_type = int32(0);
            obj.parent_id = uint32(0);
            obj.child_id = uint32(0);
            initial_anchor_pose_ = robotics.slgazebo.internal.msgs.Gazebo_msgs_Pose;
            obj.initial_anchor_pose = initial_anchor_pose_;
            world_pose_ = robotics.slgazebo.internal.msgs.Gazebo_msgs_Pose;
            obj.world_pose = world_pose_;
            parent_world_pose_ = robotics.slgazebo.internal.msgs.Gazebo_msgs_Pose;
            obj.parent_world_pose = parent_world_pose_;
            axis_ = robotics.slgazebo.internal.msgs.Gazebo_msgs_Axis;
            obj.axis  =  axis_;

        end

    end
end
