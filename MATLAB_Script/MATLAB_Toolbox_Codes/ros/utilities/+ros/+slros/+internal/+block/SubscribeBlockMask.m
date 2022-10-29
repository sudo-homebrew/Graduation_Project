classdef SubscribeBlockMask < ros.slros.internal.block.CommonMessageMask
%This class is for internal use only. It may be removed in the future.

%SubscribeBlockMask - Block mask callbacks for Subscribe block

%   Copyright 2014-2020 The MathWorks, Inc.

    properties (Constant)
        %MaskType - Type of block mask
        %   Retrieve is with get_param(gcb, 'MaskType')
        MaskType = 'ROS Subscribe'

        MaskParamIndex = struct( ...
            'TopicSourceDropdown', 1, ...
            'TopicEdit', 2, ...
            'MessageTypeEdit', 3 ...
            );

        MaskDlgIndex = struct( ...
            'TopicSelect', [2 1 3], ...  % Tab Container > "Main" tab > Topic Select Button
            'MessageTypeSelect', [2 1 5] ... % Tab Container > "Main" tab > Msg Select Button
            );

        SysObjBlockName = 'SourceBlock';
    end

    methods
        function maskInitialize(~, block)
            blkH = get_param(block, 'handle');
            topicName = get_param(block, 'topic');
            maskDisplayText = sprintf('color(''black'');');
            if length(topicName) > 16
                maskDisplayText = sprintf('%s\ntext(0.83, 0.15, ''%s'', ''horizontalAlignment'', ''right'');', ...
                    maskDisplayText, topicName);
            else
                maskDisplayText = sprintf('%s\ntext(0.45, 0.15, ''%s'', ''horizontalAlignment'', ''center'');', ...
                    maskDisplayText, topicName);
            end
            maskDisplayText = sprintf('%s\nport_label(''output'', 1, ''IsNew'');',maskDisplayText);
            maskDisplayText = sprintf('%s\nport_label(''output'', 2, ''Msg'');',maskDisplayText);
            set_param(blkH, 'MaskDisplay', maskDisplayText);
            ros.internal.setBlockIcon(blkH, 'rosicons.robotlib_subscribe');
        end
        
        function updateSubsystem(obj, block)
            sysobj_block = [block '/' obj.SysObjBlockName];
            const_block = [block '/Constant'];

            % Do not canonicalize the topic name (i.e., if user entered
            % "foo", don't convert it to "/foo"). This enables user to
            % control whether to have a relative or absolute topic name in
            % generated code.

            topic = get_param(block, 'topic');
            rosMessageType = get_param(block, 'messageType');
            [busDataType, slBusName] = ros.slros.internal.bus.Util.rosMsgTypeToDataTypeStr(rosMessageType, bdroot(block));

            % note: we use the block id of the parent, not the sys_obj block
            blockId = ros.slros.internal.block.getCppIdentifierForBlock(block, 'Sub_');
            modelName = bdroot(block);

            set_param(sysobj_block, 'SLBusName', slBusName);
            set_param(sysobj_block, 'ROSMessageType', rosMessageType);
            set_param(sysobj_block, 'ROSTopic', topic);
            set_param(sysobj_block, 'ModelName', modelName);
            set_param(sysobj_block, 'BlockId', blockId);
            set_param(const_block,'OutDataTypeStr', busDataType);
        end

    end

    methods(Static)

        function dispatch(methodName, varargin)
            obj = ros.slros.internal.block.SubscribeBlockMask();
            obj.(methodName)(varargin{:});
        end

    end
end
