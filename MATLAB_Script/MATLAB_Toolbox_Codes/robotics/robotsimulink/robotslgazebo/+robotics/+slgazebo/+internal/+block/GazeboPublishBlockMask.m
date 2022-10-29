classdef GazeboPublishBlockMask < robotics.slcore.internal.block.CommonMessageMaskInternal
%This class is for internal use only. It may be removed in the future.

%GazeboPublishBlockMask - Block mask callbacks for Gazebo Publish block

%   Copyright 2020 The MathWorks, Inc.

    properties

        %BusUtil Create the bus util object
        BusUtil = robotics.slcore.internal.bus.Util

    end


    properties (Constant)

        %TopicSelectorDlg Select topic selector dialog object
        TopicSelectorDlg = robotics.slgazebo.internal.dlg.TopicSelectorGazebo('customPublish');

        %MsgTypeSelectorDlg Select message selector dialog object
        MsgTypeSelectorDlg = robotics.slgazebo.internal.dlg.MessageTypeSelector('customPublish');

        %MaskType - Type of block mask
        MaskType = 'Gazebo Publish'

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
            sigspec_block = [block '/SignalSpecification'];

            sourceBlock = [block '/' obj.SourceBlockName];
            msgType = get_param(block, 'messageType');

            [busDataType, slBusName] = robotics.slcore.internal.bus.Util.messageTypeToDataTypeStr(msgType, bdroot(block), obj.BusUtil.BusNamePrefix);

            set_param(sourceBlock, obj.SourceBlkBusName, ['''', slBusName, '''']);

            set_param(sigspec_block, 'OutDataTypeStr', busDataType);
        end

    end

    methods(Static)

        function dispatch(methodName, varargin)
            obj = robotics.slgazebo.internal.block.GazeboPublishBlockMask();
            obj.(methodName)(varargin{:});
        end

    end
end
