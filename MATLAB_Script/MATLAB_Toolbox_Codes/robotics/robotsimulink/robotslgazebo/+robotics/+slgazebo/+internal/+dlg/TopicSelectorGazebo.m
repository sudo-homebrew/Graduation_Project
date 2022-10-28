classdef TopicSelectorGazebo < handle
%This class is for internal use only. It may be removed in the future.

%  TopicSelector opens a DDG dialog that lets the user select from a
%  list of available Gazebo topics. Once the user accepts the changes (or
%  cancels the dialog), a callback is invoked with the closure action
%  and selected topic type.
%
%  Sample use:
%   selector = robotics.slgazebo.internal.dlg.TopicSelector;
%   selector.openDialog(@(isAccepted,topicName) disp(topicName))

%   Copyright 2019-2021 The MathWorks, Inc.

    properties(SetAccess=private)
        %GazeboTopic topic name user selected
        GazeboTopic = ''

        %GazeboMessageType topic type corresponding to the topic user selected
        GazeboMessageType = ''

        %TopicList list of available topics
        TopicList = {}

        %TopicMsgTypes message type corresponding to each topic
        TopicMsgTypes = {}

        %CloseFcnHandle function handle to call when dialog is closed
        CloseFcnHandle = function_handle.empty

        %GazeboClient Use client to establish connection to gazebo server
        GazeboClient

        MessageGetterType = ''
    end
    properties(SetAccess=public)
        InitialTopic = ''
    end

    methods
        function obj = TopicSelectorGazebo(messageGetterType)
            obj.GazeboClient = robotics.internal.GazeboClient;
            obj.MessageGetterType = messageGetterType;
        end

        function dlg = getHighlitedTopicDialog(obj, initialTopicSelection)
            dlg = DAStudio.Dialog(obj);
            % find index number of previously selected topic name from
            % topic list
            index = find(strcmpi(initialTopicSelection, obj.TopicList), 1);
            if(isempty(index))
                prevIndex = 0;
            else
                prevIndex = index - 1;
            end
            % select previous topic name from dialog
            dlg.setWidgetValue('gazebotopiclist', prevIndex);
        end

        function dlg = getCustomTopicMessage(obj, initialTopicSelection, closeFcnHandle)

        %get the list of custom topics and its custom message type from Gazebo
            profileStore = robotics.slgazebo.internal.sim.GazeboPreferenceStore;
            profile = profileStore.getProfile();
            obj.GazeboClient.connect(profile.MasterHost, profile.MasterPort, profile.SimulationTimeout*1000);
            topicList = obj.GazeboClient.getTopicList();

            msgTypes = [topicList.data.type];

            % read from custom message folder
            if(exist('importCustomMessageTypeList.m','file'))
                customMsgList = importCustomMessageTypeList();
            else
                customMsgList.MsgList = [];
                customMsgList.MsgNameList = [];
            end

            customMsgType = convertCharsToStrings(customMsgList.MsgList);
            customMsgName = convertCharsToStrings(customMsgList.MsgNameList);

            idx = false(1,length(msgTypes));
            for index= 1 : length(msgTypes)
                if nnz(strcmp(customMsgType,msgTypes(index)))
                    idx(index) = true;
                end
            end

            topics = [topicList.data.name];

            topics = topics(idx);

            % retrieve message type name as per Gazebo
            msgTypes = msgTypes(idx);

            % retrieve message type name as per Simulink Message Type
            %
            % replaced strcmp instead of strfind as,
            % find that strfind returns true for pose and poseStamped
            msgNames = strings(1,length(msgTypes));
            for index = 1:length(msgTypes)
                mIdx = strcmp(customMsgType,msgTypes(index));
                if(any(mIdx))
                    msgNames(index) = customMsgName(mIdx);
                else
                    msgNames(index) = "";
                end
            end

            obj.GazeboClient.shutdown();

            % convertStringsToChars provides non-array output on single
            % string input
            if( length(topics) == 1)
                obj.TopicList = {convertStringsToChars(topics)};
            else
                obj.TopicList = convertStringsToChars(topics);
            end

            if( length(msgNames) == 1)
                obj.TopicMsgTypes = {convertStringsToChars(msgNames)};
            else
                obj.TopicMsgTypes = convertStringsToChars(msgNames);
            end

            validateattributes(closeFcnHandle, {'function_handle'}, {'scalar'});
            obj.CloseFcnHandle = closeFcnHandle;
            % get highlighted topic dialog
            dlg = getHighlitedTopicDialog(obj, initialTopicSelection);

            if( ~isempty(obj.TopicList) && ~isempty(obj.TopicMsgTypes))
                obj.GazeboTopic = obj.TopicList{1};
                obj.GazeboMessageType = obj.TopicMsgTypes{1};
            else
                obj.GazeboTopic = obj.TopicList;
                obj.GazeboMessageType = obj.TopicMsgTypes;
            end

        end


        function dlg = getTopicMessage(obj, initialTopicSelection, closeFcnHandle)

        %get the list of topics and its message type from Gazebo
            profileStore = robotics.slgazebo.internal.sim.GazeboPreferenceStore;
            profile = profileStore.getProfile();
            obj.GazeboClient.connect(profile.MasterHost, profile.MasterPort, profile.SimulationTimeout*1000);
            topicList = obj.GazeboClient.getTopicList();

            msgTypes = [topicList.data.type];
            idx = contains(msgTypes, ["gazebo.msgs.ImageStamped", ...
                                "gazebo.msgs.IMU",...
                                "gazebo.msgs.LaserScanStamped"]);
            topics = [topicList.data.name];

            msgTypes = msgTypes(idx);
            msgTypes = strrep(msgTypes, "gazebo.msgs.", "gazebo_msgs/");
            msgTypes = strrep(msgTypes, "Stamped", "");
            topics = topics(idx);

            %append the ground truth pose for model links
            modelInfo = obj.GazeboClient.getModelEntityList();
            for modelIdx = 1:numel(modelInfo.model_data)
                for linkIdx = 1:numel(modelInfo.model_data(modelIdx).links.link_name)
                    msgTypes(end+1) = "gazebo_msgs/Pose"; %#ok<AGROW>
                    topics(end+1) = strcat(robotics.gazebo.internal.GazeboReader.GroundTruthWorldPosePrefix, ...
                                           modelInfo.model_data(modelIdx).model_name, ...
                                           "/",...
                                           modelInfo.model_data(modelIdx).links.link_name(linkIdx)); %#ok<AGROW>
                end

                for jointIdx = 1:numel(modelInfo.model_data(modelIdx).joints.joint_name)
                    msgTypes(end+1) = "gazebo_msgs/JointState"; %#ok<AGROW>
                    topics(end+1) = strcat(robotics.gazebo.internal.GazeboReader.JointStatePrefix, ...
                                           modelInfo.model_data(modelIdx).model_name, ...
                                           "/",...
                                           modelInfo.model_data(modelIdx).joints.joint_name(jointIdx)); %#ok<AGROW>
                end

                for linkIdx = 1:numel(modelInfo.model_data(modelIdx).links.link_name)
                    msgTypes(end+1) = "gazebo_msgs/LinkState"; %#ok<AGROW>
                    topics(end+1) = strcat(robotics.gazebo.internal.GazeboReader.LinkStatePrefix, ...
                                           modelInfo.model_data(modelIdx).model_name, ...
                                           "/",...
                                           modelInfo.model_data(modelIdx).links.link_name(linkIdx)); %#ok<AGROW>
                end
            end

            obj.GazeboClient.shutdown();

            obj.TopicList = convertStringsToChars(topics);
            obj.TopicMsgTypes = convertStringsToChars(msgTypes);

            validateattributes(closeFcnHandle, {'function_handle'}, {'scalar'});
            obj.CloseFcnHandle = closeFcnHandle;
            % get highlighted topic dialog
            dlg = getHighlitedTopicDialog(obj, initialTopicSelection);

            obj.GazeboTopic = obj.TopicList{1};
            obj.GazeboMessageType = obj.TopicMsgTypes{1};
        end

        function dlg = openDialog(obj, closeFcnHandle)
        % closeFcnHandle: handle to function that takes two arguments
        %   closeFcn(isAcceptedSelection, gazeboTopic, gazeboMsgType)
        %      isAcceptedSelection: true of user clicked on 'ok', false
        %        if user clicked on 'cancel' or closed window
        %      gazeboTopic: last selected Gazebo topic (string)
        %      gazeboMsgType: message type of last selected Gazebo topic (string)

        % check message type is builtin or custom
            if(strcmp(obj.MessageGetterType,'builtin'))
                dlg = getTopicMessage(obj, obj.InitialTopic, closeFcnHandle);
            else
                dlg = getCustomTopicMessage(obj,obj.InitialTopic,closeFcnHandle);
            end

        end
    end


    methods(Hidden)
        % Called when user selects an item from the list
        function dlgCallback(obj, dlg, tag, value) %#ok<INUSL>
            obj.GazeboTopic = obj.TopicList{value+1}; % value is zero-based
            obj.GazeboMessageType = obj.TopicMsgTypes{value+1};
            dlg.refresh;
        end

        function dlgClose(obj, closeaction)
        % closeaction is 'ok' if user clicked OK
        %                'cancel' if user clicked cancel or closed window
            if ~isempty(obj.CloseFcnHandle)
                isAcceptedSelection = strcmpi(closeaction, 'ok');
                % Need to change 'InitialTopic' with user selected topic,
                % once user press 'ok' in dialog box of 'TopicSelect'. This
                % helps, when user again open topic select dialog then user
                % can see highlight on the last selected topic name.
                if isAcceptedSelection
                    obj.InitialTopic = obj.GazeboTopic;
                end
                try
                    feval(obj.CloseFcnHandle, isAcceptedSelection, obj.GazeboTopic, obj.GazeboMessageType);
                catch
                    % Absorb all errors. If they are propagated back to
                    % DDG, this causes MATLAB to crash, (Can't convert to
                    % warnings are not as they are not displayed either).
                end
            end
        end

        function dlgstruct = getDialogSchema(obj)
            msglist.Name    = '';
            msglist.Type    = 'listbox';
            msglist.Entries = obj.TopicList;
            msglist.Tag     = 'gazebotopiclist';
            msglist.MultiSelect = false;
            msglist.ObjectMethod = 'dlgCallback'; % call method on UDD source object
            msglist.MethodArgs = {'%dialog', '%tag', '%value'}; % object handle is implicit first arg
            msglist.ArgDataTypes = {'handle', 'string', 'mxArray'}; % 'handle' is type of %dialog
            msglist.Value = 0;
            msglist.NameLocation = 2; % top left

            % Main dialog
            dlgstruct.DialogTitle = message('robotics:robotslgazebo:topicselector:DialogTitle').getString;
            dlgstruct.HelpMethod = 'helpview';

            % adds 'HelpArgs' based on block type. 'builtin' is for read
            % block while 'customSubscribe' and 'customPublish' is for
            % subscribe and publish block. This is needed for 'help-button'
            % in subroutine like 'TopicSelect' and 'MessageTypeSelect'
            switch obj.MessageGetterType
              case 'builtin'
                dlgstruct.HelpArgs =  {fullfile(docroot, 'robotics', 'helptargets.map'), 'rstGazeboRead'};
              case 'customSubscribe'
                dlgstruct.HelpArgs =  {fullfile(docroot, 'robotics', 'helptargets.map'), 'rstGazeboSubscribe'};
              case 'customPublish'
                dlgstruct.HelpArgs =  {fullfile(docroot, 'robotics', 'helptargets.map'), 'rstGazeboPublish'};
            end
            dlgstruct.CloseMethod = 'dlgClose';
            dlgstruct.CloseMethodArgs = {'%closeaction'};
            dlgstruct.CloseMethodArgsDT = {'string'};

            % Make this dialog modal wrt to other DDG dialogs
            % (doesn't block MATLAB command line)
            dlgstruct.Sticky = true;

            % Buttons to show on dialog (these are options to pass to DDG,
            % not the final strings, so there is no need to use message
            % catalog)
            dlgstruct.StandaloneButtonSet =  ...
                {'Ok', 'Cancel', 'Help'}; % also available: 'Revert', 'Apply'

            dlgstruct.Items = {msglist};
        end
    end
end
