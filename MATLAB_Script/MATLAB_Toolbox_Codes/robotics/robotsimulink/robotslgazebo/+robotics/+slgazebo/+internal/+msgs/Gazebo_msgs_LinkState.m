classdef Gazebo_msgs_LinkState < robotics.slgazebo.internal.msgs.Gazebo_Msgs
% This function is for internal use only. It may be removed in the future.
% DO NOT EDIT!
% Gazebo_msgs_LinkState is the template for LinkState message bus type
%It defines fields required to read link state

%   Copyright 2020 The MathWorks, Inc.

    properties

        MessageType = 'gazebo_msgs/LinkState';
        % name of model
        model_name uint8
        % name of link
        link_name uint8
        % link id
        link_id
        % linear velocity of link in world frame [x, y, z] directions
        world_linear_velocity robotics.slgazebo.internal.msgs.Gazebo_msgs_Point
        % angular velocity of link in world frame [x, y, z] directions
        world_angular_velocity robotics.slgazebo.internal.msgs.Gazebo_msgs_Point
        % linear velocity of link relative to parent model [x, y, z] directions
        relative_linear_velocity robotics.slgazebo.internal.msgs.Gazebo_msgs_Point
        % angular velocity of link relative to parent model [x, y, z] directions
        relative_angular_velocity robotics.slgazebo.internal.msgs.Gazebo_msgs_Point
        % absolute world pose of link
        world_pose robotics.slgazebo.internal.msgs.Gazebo_msgs_Pose
        % pose of link relative to its parent
        relative_pose robotics.slgazebo.internal.msgs.Gazebo_msgs_Pose
        % true if self collision enabled for link
        self_collide
        % true if gravity is enabled for link
        gravity
        % true if link is kinematic
        kinematic
        % true if wind mode is enabled
        enable_wind
        % true if link is canonical
        canonical

    end

    methods

        function obj = Gazebo_msgs_LinkState(~)

            obj.link_id = uint32(0);
            world_linear_velocity_ = robotics.slgazebo.internal.msgs.Gazebo_msgs_Point;
            obj.world_linear_velocity = world_linear_velocity_;
            world_angular_velocity_ = robotics.slgazebo.internal.msgs.Gazebo_msgs_Point;
            obj.world_angular_velocity = world_angular_velocity_;
            relative_linear_velocity_ = robotics.slgazebo.internal.msgs.Gazebo_msgs_Point;
            obj.relative_linear_velocity = relative_linear_velocity_;
            relative_angular_velocity_ = robotics.slgazebo.internal.msgs.Gazebo_msgs_Point;
            obj.relative_angular_velocity = relative_angular_velocity_;
            world_pose_ = robotics.slgazebo.internal.msgs.Gazebo_msgs_Pose;
            obj.world_pose = world_pose_;
            relative_pose_ = robotics.slgazebo.internal.msgs.Gazebo_msgs_Pose;
            obj.relative_pose = relative_pose_;
            obj.self_collide = false;
            obj.gravity = false;
            obj.kinematic = false;
            obj.enable_wind = false;
            obj.canonical = false;

        end

    end
end
