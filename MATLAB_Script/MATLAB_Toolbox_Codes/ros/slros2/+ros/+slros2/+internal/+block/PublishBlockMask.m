classdef PublishBlockMask < ros.slros.internal.block.PublishBlockMask
%This class is for internal use only. It may be removed in the future.

%PublishBlockMask - Block mask callbacks for ROS2 Publish block

%   Copyright 2019 The MathWorks, Inc.

    methods
        function updateSubsystem(obj, block)
            sysobj_block = [block '/' obj.SysObjBlockName];
            sigspec_block = [block '/SignalSpecification'];

            % Do not canonicalize the topic name (i.e., if user entered
            % "foo", don't convert it to "/foo"). This enables user to
            % control whether to have a relative or absolute topic name in
            % generated code.

            topic = get_param(block, 'topic');
            rosMessageType = get_param(block, 'messageType');
            [busDataType, slBusName] = ros.slros2.internal.bus.Util.rosMsgTypeToDataTypeStr(rosMessageType);

            % note: we use the block id of the parent, not the sys_obj block
            blockId = ros.slros.internal.block.getCppIdentifierForBlock(block, 'Pub_');
            modelName = bdroot(block);

            set_param(sysobj_block, 'SLBusName', slBusName);
            set_param(sysobj_block, 'ROSMessageType', rosMessageType);
            set_param(sysobj_block, 'ROSTopic', topic);
            set_param(sysobj_block, 'ModelName', modelName);
            set_param(sysobj_block, 'BlockId', blockId);
            set_param(sigspec_block, 'OutDataTypeStr', busDataType);
        end

        % When the user updates the message type and/or topic, updateSubsystem
        % has be called in a callback context (since it is modifying the block).
        %
        % However, topicSelect() and messageTypeSelect() cannot call
        % updateSubsystem as the dialog is still open when the function is
        % done (the user hasn't made the selection yet). So topicSelect and
        % messageTypeSelect will return without applying any changes.
        % Note - we can't pass around a handle to updateSubsystem  as it
        % can only be called from within the callback context.
        %
        % Solution:
        %   Ensure either topicEdit() or messageTypeEdit() callbacks are
        %   configured. Once the user applies the change by clicking on OK/Apply
        %   in the mask dialog, the maskInitialize() will be invoked, and
        %   also the callbacks will be called on the next model update, so these
        %   will invoke updateSubsystem().
        %
        % The above solution also ensures that the pub and sub block
        % subsystems will be updated correctly if the user modifies the
        % 'topic' or 'messageType' parameters using SET_PARAM (since
        % the mask callbacks will be invoked during model update)

        function topicEdit(obj, block)
            sysobj_block = [block '/' obj.SysObjBlockName];
            curValue = get_param(sysobj_block, 'ROSTopic');
            newValue = get_param(block, 'topic');
            if ~ros.internal.Namespace.isValidGraphName(newValue)
                set_param(block, 'topic', curValue);
                error(message('ros:slros:blockmask:InvalidTopicName', newValue));
            end
            rosMsgType = get_param(block, 'messageType');
            ros.slros2.internal.bus.Util.createBusIfNeeded(rosMsgType, bdroot(block));
            obj.updateSubsystem(block);
        end
        
        function messageTypeEdit(obj, block)
            obj.messageLoadFcn(block)
            obj.updateSubsystem(block);
        end

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
            set_param(blkH, 'MaskDisplay', maskDisplayText);
            ros.internal.setBlockIcon(blkH, 'rosicons.ros2lib_publish');
        end        
    end

    methods(Static)
        function ret = getMaskType()
            ret = 'ROS2 Publish';
        end
        
        function messageLoadFcn(block)
            rosMsgType = get_param(block, 'messageType');
            ros.slros2.internal.bus.Util.createBusIfNeeded(rosMsgType, bdroot(block));
        end        

        function dispatch(methodName, varargin)
            obj = ros.slros2.internal.block.PublishBlockMask();
            obj.(methodName)(varargin{:});
        end

    end
end
