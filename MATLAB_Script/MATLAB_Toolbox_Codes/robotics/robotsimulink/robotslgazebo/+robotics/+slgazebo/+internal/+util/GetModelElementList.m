classdef GetModelElementList < handle
%This function is for internal use only. It may be removed in the future.

%GetModelElementList Get the list of gazebo model links and joints

%   Copyright 2019-2020 The MathWorks, Inc.

    properties
        %Joints List of joints in the Gazebo simulator
        %   Format is 'modelName/jointName'
        Joints = {}
        %Links List of links in the Gazebo simulator
        %   Format is 'modelName/linkName'
        Links = {}
    end

    properties (Access = private)
        %GazeboClient used to connect to Gazebo and query model information
        GazeboClient
    end

    methods
        function obj = GetModelElementList()
        %GetModelElementList

        % initialize Gazebo client on construction
            obj.GazeboClient = robotics.internal.GazeboClient;
        end

        function updateEntityList(obj)
        %updateEntityList update model entity list

        % connect to Gazebo
            profileStore = robotics.slgazebo.internal.sim.GazeboPreferenceStore;
            profile = profileStore.getProfile();
            obj.GazeboClient.connect(profile.MasterHost, profile.MasterPort, profile.SimulationTimeout*1000);

            %refresh the model links and joints to the list
            obj.Links = {};
            obj.Joints = {};
            modelInfo = obj.GazeboClient.getModelEntityList();
            for modelIdx = 1:numel(modelInfo.model_data)
                for linkIdx = 1:numel(modelInfo.model_data(modelIdx).links.link_name)
                    obj.Links{end+1} = convertStringsToChars(...
                        strcat(modelInfo.model_data(modelIdx).model_name, ...
                               "/",...
                               modelInfo.model_data(modelIdx).links.link_name(linkIdx)));
                end

                for jointIdx = 1:numel(modelInfo.model_data(modelIdx).joints.joint_name)
                    obj.Joints{end+1} = convertStringsToChars(...
                        strcat(modelInfo.model_data(modelIdx).model_name, ...
                               "/",...
                               modelInfo.model_data(modelIdx).joints.joint_name(jointIdx)));
                end
            end

            % shutdown client
            obj.GazeboClient.shutdown();

            % add <JOINT0> or <LINK1> tag as suffix if a model has same link
            % and joint names
            for linkIndex = 1:numel(obj.Links)
                linkName = obj.Links{linkIndex};

                % find duplicate index for joint as well as link
                jointIdx = find(cellfun(@(x)strcmp(x,linkName),obj.Joints)==1);
                linkIdx = find(cellfun(@(x)strcmp(x,linkName),obj.Links)==1);

                % calculate number duplicate index
                duplicateCount = numel(jointIdx) + numel(linkIdx);

                % greater than 1 means duplicate names are present
                if( duplicateCount > 1)
                    for idx = 1: numel(jointIdx)
                        % replace older joint names with new names
                        newJointName = convertStringsToChars(strcat(obj.Joints{jointIdx(idx)},"<JOINT",num2str(idx-1),">"));
                        obj.Joints{jointIdx(idx)} = newJointName;
                    end

                    for idx =1:numel(linkIdx)
                        % replace older link names with new names
                        newLinkName = convertStringsToChars(strcat(obj.Links{linkIdx(idx)},"<LINK",num2str(idx-1),">"));
                        obj.Links{linkIdx(idx)} = newLinkName;
                    end
                end
            end

            % if there are only duplicate joints then tags are added
            % though, gazebo throw error and don't show two joints for
            % same model, this condition is handled here
            for jointIndex = 1:numel(obj.Joints)
                jointName = obj.Joints{jointIndex};

                % find duplicate index of joints
                jointIdx = find(cellfun(@(x)strcmp(x,jointName),obj.Joints)==1);
                if( numel(jointIdx) > 1)
                    for idx = 1: numel(jointIdx)
                        % replace older joint names with new names
                        newJointName = convertStringsToChars(strcat(obj.Joints{jointIdx(idx)},"<JOINT",num2str(idx-1),">"));
                        obj.Joints{jointIdx(idx)} = newJointName;
                    end
                end
            end
        end
    end
end
