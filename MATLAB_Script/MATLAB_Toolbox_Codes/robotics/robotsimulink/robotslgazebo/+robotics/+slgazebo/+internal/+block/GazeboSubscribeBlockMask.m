classdef GazeboSubscribeBlockMask < robotics.slcore.internal.block.CommonMessageMaskInternal
%This class is for internal use only. It may be removed in the future.

%GazeboSubscribeBlockMask - Block mask callbacks for Gazebo Subscribe block

%   Copyright 2020 The MathWorks, Inc.

    properties

        %BusUtil Create the bus util object
        BusUtil = robotics.slcore.internal.bus.Util

    end


    properties (Constant)

        %TopicSelectorDlg Select topic selector dialog object
        TopicSelectorDlg = robotics.slgazebo.internal.dlg.TopicSelectorGazebo('customSubscribe');

        %MsgTypeSelectorDlg Select message selector dialog object
        MsgTypeSelectorDlg = robotics.slgazebo.internal.dlg.MessageTypeSelector('customSubscribe')

        %MaskType - Type of block mask
        MaskType = 'Gazebo Subscribe'

        MaskParamIndex = struct( ...
            'TopicSourceDropdown', 1, ...
            'TopicEdit', 2, ...
            'MessageTypeEdit', 3 ...
            );

        MaskDlgIndex = struct( ...
            'TopicSelect', [2 1 3], ...  % Tab Container > "Main" tab > Topic Select Button
            'MessageTypeSelect', [2 1 5] ... % Tab Container > "Main" tab > Msg Select Button
            );

        TopicSourceFromNetwork =  message('robotics:robotslgazebo:blockmask:TopicSourceFromNetwork').getString;

        %Source block name
        SourceBlockName = 'SourceBlock';

        %SourceBlkMsgType Gazebo message type mask parameter of matlab
        %system block
        SourceBlkMsgType = 'TopicType'

        %SourceBlkTopic Gazebo topic name mask parameter of matlab system
        %block
        SourceBlkTopic = 'TopicName'

        %SourceBlkBusName Bus name mask parameter of matlab system block
        SourceBlkBusName = 'OutputBusName';

    end

    methods

        function updateSubsystem(obj, block)
            sourceBlock = [block '/' obj.SourceBlockName];
            msgType = get_param(block, 'messageType');
            [~, slBusName] = robotics.slcore.internal.bus.Util.messageTypeToDataTypeStr(msgType, bdroot(block), obj.BusUtil.BusNamePrefix);
            set_param(sourceBlock, obj.SourceBlkBusName, ['''', slBusName, '''']);
        end

    end

    methods(Static)

        function dispatch(methodName, varargin)
            obj = robotics.slgazebo.internal.block.GazeboSubscribeBlockMask();
            % To highlight previously selected topic list dialog, need to
            % retrieve previously assigned topic name. Here, previously
            % assigned topic name is retrieved and stored in
            % TopicSelectorDlg to utilize in 'TopicSelectorGazebo'
            if(strcmp(methodName,'topicEdit'))
                obj.TopicSelectorDlg.InitialTopic = get_param(varargin{1}, 'topic');
            end
            obj.(methodName)(varargin{:});
        end

    end
end
