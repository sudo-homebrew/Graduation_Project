classdef Category
%This function is for internal use only. It may be removed in the future.

%Category separate Gazebo read topics into different groups

%   Copyright 2019-2020 The MathWorks, Inc.
    enumeration
        %GroundTruthWorldPose Reads ground truth pose in world frame from
        %Gazebo core API
        GroundTruthWorldPose

        %TransportMessage Reads messages published by Gazebo through
        %ignition transport
        TransportMessage

        %JointStateMessage Reads joint state details
        JointStateMessage

        %LinkStateMessage Reads link state details
        LinkStateMessage
    end

    properties (Constant)
        PoseType = "gazebo_msgs/Pose"

        ImageType = "gazebo_msgs/Image"

        FusedIMUType = "gazebo_msgs/IMU"

        LidarType = "gazebo_msgs/LaserScan"

        JointStateType = "gazebo_msgs/JointState"

        LinkStateType = "gazebo_msgs/LinkState"

        CustomMessageType = "gazebo_msgs/custom/"

        CustomMessageName = "Gazebo_msgs_custom_"

        CoSimMsgsPrefix = "mw.internal.robotics.gazebotransport";
    end
end
