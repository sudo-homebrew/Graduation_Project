classdef GazeboReader < handle
%This function is for internal use only. It may be removed in the future.

%GazeboReader acts as a middle layer that maps the Gazebo topic names to
%actual API calls to internal builtin client.

%   Copyright 2019-2020 The MathWorks, Inc.

    properties (Constant)
        %GroundTruthWorldPosePrefix Prefix for the ground truth topic name
        %    Correct topic format would be [GroundTruthWorldPosePrefix,
        %    'modelname/linkname']
        GroundTruthWorldPosePrefix = '/gazebo/ground_truth/world_pose/';

        %JointStatePrefix Prefix for the joint state topic name
        %    Correct topic format would be [JointStatePrefix,
        %    'modelname/jointname']
        JointStatePrefix = '/gazebo/ground_truth/joint_state/';

        %LinkStatePrefix Prefix for the link state topic name
        %    Correct topic format would be [LinkStatePrefix,
        %    'modelname/linkname']
        LinkStatePrefix = '/gazebo/ground_truth/link_state/';
    end

    properties (Access = private)
        %GazeboClient Client interface to the Gazebo plugin
        GazeboClient
    end

    methods
        function obj = GazeboReader(client)
        %GazeboReader constructor
        %    Client should be connected already when passed to the
        %    reader
            obj.GazeboClient = client;
        end

        function success = subscribeIfNeeded(obj, topicType, topicName)
        %subscribeIfNeeded subscribe to Gazebo transport messages

        % dispatch into specific message categories
            topicCategory = obj.categorizeGazeboTopic(topicName);
            if topicCategory == robotics.gazebo.internal.topic.Category.TransportMessage
                success = obj.GazeboClient.subscribeMessageByTopic(topicType, topicName);
            else
                success = true;
            end
        end

        function success = subscribeCustomIfNeeded(obj, topicType, topicName)
        %subscribeCustomIfNeeded initialize to Gazebo custom subscriber
            success = obj.GazeboClient.initCustomSubscriber(topicName,topicType);
        end

        function [isNew, result] = read(obj, topicType, topicName)
        %read topic message from Gazebo client

        % dispatch into specific message categories
            topicCategory = obj.categorizeGazeboTopic(topicName);
            switch topicCategory
              case robotics.gazebo.internal.topic.Category.GroundTruthWorldPose
                % dispatch to ground truth reader
                [isNew, result] = obj.readGroundTruthWorldPose(topicName);
              case robotics.gazebo.internal.topic.Category.JointStateMessage
                % dispatch to joint state reader
                [isNew, result] = obj.readJointState(topicName);
              case robotics.gazebo.internal.topic.Category.LinkStateMessage
                % dispatch to link state reader
                [isNew, result] = obj.readLinkState(topicName);
              otherwise
                % dispatch to transport reader
                [isNew, result] = obj.readTransportMessage(topicType, topicName);
            end
        end

        function [isNew, result] = readCustom(obj, topicType, topicName)
        %read custom message from topic of Gazebo client

            [isNew, result] = obj.readCustomMessage(topicType, topicName);
        end

    end

    methods (Access = private)
        function [isNew, result] = readGroundTruthWorldPose(obj, topicname)
        %readGroundTruthWorldPose from Gazebo client

            linkname = strsplit(strrep(topicname, obj.GroundTruthWorldPosePrefix, ''), '/');
            poseInfo = obj.GazeboClient.getGroundTruthWorldPose(linkname{1}, linkname{2});
            isNew = poseInfo.is_new;
            result = poseInfo.message;
        end

        function [isNew, result] = readTransportMessage(obj, topicType, topicName)
        %readTransportMessage from Gazebo client

            imageInfo = obj.GazeboClient.getMessageByTopic(topicType, topicName);
            isNew = imageInfo.is_new;
            result = imageInfo.message;
        end

        function [isNew, result] = readCustomMessage(obj, topicType, topicName)
        %readCustomMessage from Gazebo client

            customMsg = obj.GazeboClient.getCustomMessage(topicName,topicType);
            isNew = customMsg.is_new;
            result = robotics.slgazebo.internal.util.getCustomMessageField(customMsg,topicType);

        end

        function [isNew, result] = readJointState(obj, topicname)
        %readJointState from Gazebo client

            jointname = strsplit(strrep(topicname, obj.JointStatePrefix, ''), '/');
            jointInfo = obj.GazeboClient.getJointState(jointname{1}, jointname{2});
            isNew = jointInfo.is_new;
            result = jointInfo.message;
        end

        function [isNew, result] = readLinkState(obj, topicname)
        %readLinkState from Gazebo client

            linkname = strsplit(strrep(topicname, obj.LinkStatePrefix, ''), '/');
            linkInfo = obj.GazeboClient.getLinkState(linkname{1}, linkname{2});
            isNew = linkInfo.is_new;
            result = linkInfo.message;
        end

    end

    methods (Static)
        function topicCategory = categorizeGazeboTopic(topicname)
        %categorizeGazeboTopic Categorize Gazebo topic based on topic name

            if startsWith(topicname, robotics.gazebo.internal.GazeboReader.GroundTruthWorldPosePrefix)
                % for ground truth world pose
                topicCategory = robotics.gazebo.internal.topic.Category.GroundTruthWorldPose;
            elseif startsWith(topicname, robotics.gazebo.internal.GazeboReader.JointStatePrefix)
                % for joint state
                topicCategory = robotics.gazebo.internal.topic.Category.JointStateMessage;
            elseif startsWith(topicname, robotics.gazebo.internal.GazeboReader.LinkStatePrefix)
                % for link state
                topicCategory = robotics.gazebo.internal.topic.Category.LinkStateMessage;
            else
                % for transport message
                topicCategory = robotics.gazebo.internal.topic.Category.TransportMessage;
            end

        end
    end

end
