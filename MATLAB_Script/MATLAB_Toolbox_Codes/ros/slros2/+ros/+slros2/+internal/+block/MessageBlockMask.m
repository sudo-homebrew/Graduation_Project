classdef MessageBlockMask < ros.slros.internal.block.CommonMessageMask
    %This class is for internal use only. It may be removed in the future.

    %MessageBlockMask - Block mask callbacks for "Blank Message" block

    %   Copyright 2019-2021 The MathWorks, Inc.
    properties (Constant)
        %MaskType - Type of block mask
        %   Retrieve is with get_param(gcb, 'MaskType')
        MaskType = 'ROS2 Blank Message'

        MaskParamIndex = ros.slros.internal.block.MessageBlockMask.MaskParamIndex;

        MaskDlgIndex = ros.slros.internal.block.MessageBlockMask.MaskDlgIndex;

        SysObjBlockName = ''  % No system object block
    end

    properties(Access=private,Hidden)
        %MsgMaskObj Shared message block mask
        MsgMaskObj = ros.slros.internal.block.MessageBlockMask();
    end

    methods

        function messageClassChange(obj, block)
            %messageClassChange The message class drop-down changed

            % If current entity type is compatible with the selected message class,
            % don't change it
            srvList = obj.getServiceList();
            if obj.MsgMaskObj.isEntityTypeCompatible(block,srvList)
                obj.setMessageTypeBasedOnEntity(block);
                return;
            end

            msgClass = get_param(block, 'messageClass');
            if isKey(obj.MsgMaskObj.MessageClassMap, msgClass)
                entityType = obj.MsgMaskObj.MessageClassMap(msgClass);

                % Set entity type
                set_param(block, 'entityType', entityType)
                obj.setMessageTypeBasedOnEntity(block);
            end
        end

        function entityTypeEdit(obj, block)
            %entityTypeEdit Update buses and subsystem based on entity type selection

            obj.setMessageTypeBasedOnEntity(block);
            obj.updateSubsystem(block);
        end

        function updateSubsystem(~, block)
            %updateSubsystem Update the constant block in subsystem

            msgType = get_param(block, 'messageType');
            constantBlock = [block '/Constant'];

            [busDataType, ~] = ros.slros2.internal.bus.Util.rosMsgTypeToDataTypeStr(msgType);
            set_param(constantBlock, 'OutDataTypeStr', busDataType);
        end

        function selectButtonPressed(obj, block)
            %selectButtonPressed Callback for "Select" button
            %   This brings up either the message type selection dialog or
            %   the service type selection dialog.

            if obj.MsgMaskObj.isServiceClass(block)
                % Configure a service type selection dialog
                msgDlg = ros.slros2.internal.dlg.ServiceTypeSelector();
            else
                msgDlg = ros.slros2.internal.dlg.MessageTypeSelector();
            end
            entityType = get_param(block, 'entityType');
            msgDlg.openDialog(entityType, @dialogCloseCallback);

            function dialogCloseCallback(isAcceptedSelection, selectedMsg)
                if isAcceptedSelection
                    set_param(block, 'entityType', selectedMsg);
                    obj.setMessageTypeBasedOnEntity(block);
                end
            end
        end

        function loadFcn(obj, block)
            %loadFcn Verify consistency of parameters when LoadFcn is called
            %   This function is primarily used to ensure
            %   backwards-compatibility.
            load(obj.MsgMaskObj,block);
        end
        
        function preSaveFcn(obj, block)
            %preSaveFcn Verify consistency of parameters before saving
            preSaveFcn(obj.MsgMaskObj, block);
        end

        function maskInitialize(obj, block)
            obj.setMaskDisplayText(block);
            blkH = get_param(block, 'handle');
            ros.internal.setBlockIcon(blkH, 'rosicons.ros2lib_blankmessage');
        end

        function setMaskDisplayText(obj, block)
            blkH = get_param(block, 'handle');
            entityType = get_param(block,'entityType');
            msgClass = get_param(block, 'messageClass');
            messageType = obj.MsgMaskObj.entityToMessageType(entityType,msgClass);
            maskDisplayText = sprintf('color(''black'');');
            if ~obj.isLibraryBlock(block)
                if length(messageType) > 22
                    maskDisplayText = sprintf('%s\ntext(0.96, 0.15, ''%s'', ''horizontalAlignment'', ''right'');', ...
                        maskDisplayText, messageType);
                else
                    maskDisplayText = sprintf('%s\ntext(0.5, 0.15, ''%s'', ''horizontalAlignment'', ''center'');', ...
                        maskDisplayText, messageType);
                end
            end
            set_param(blkH, 'MaskDisplay', maskDisplayText);
        end

    end
    
    methods (Access = protected)
        function setMessageTypeBasedOnEntity(obj, block)
        %setMessageTypeBasedOnEntity Set message type based on entity type
        %   For example, when dealing with services, we have to attach
        %   "Request" or "Response".

            entityType = get_param(block, 'entityType');
            msgClass = get_param(block, 'messageClass');
            msgType = obj.MsgMaskObj.entityToMessageType(entityType, msgClass);
            % Update buses and subsystem
            set_param(block, 'messageType', msgType);
            obj.messageLoadFcn(block);
        end
    end    

    methods(Hidden,Static)
        function [outData] =  forwardBlock(inData)
            % FORWARDBLOCK Forward old ROS 2 Blank Message block to new
            % parameters
            
            % Old version of blank message block did not have the
            % parameters 'entityType' and 'messageClass'. This function
            % forwards ROS 2 Blank Message block to new version
            
            % Create new instance data
            outData.NewBlockPath = ''; % No change in library path
            outData.NewInstanceData = [];
            
            % Get old instance data
            instanceData = inData.InstanceData;
            % Get the field type 'Name' from instanceData
            [ParameterNames{1:length(instanceData)}] = instanceData.Name;
            % Get the field type 'Value' from instanceData
            [ParamValues{1:length(instanceData)}] = instanceData.Value;
            if (~ismember('entityType',ParameterNames) || ~ismember('messageClass',ParameterNames))
                % entityType parameter is not present in old ROS 2 Blank message block
                instanceData(end+1).Name = 'entityType';
                instanceData(end).Value = ParamValues{contains(ParameterNames,'messageType')};
                instanceData(end+1).Name = 'messageClass';
                instanceData(end).Value = message('ros:slros:blockmask:MessageType_Msg').getString(matlab.internal.i18n.locale("en"));
            end
            outData.NewInstanceData = instanceData;
        end
        
        function messageLoadFcn(block)
            % if StaticLinkStatus is 'none', then the current system must
            % be ros2lib. Global buses should not be created in this case 
            % because the variable length information is not known.  
            
            if ~ros.slros.internal.block.CommonMask.isLibraryBlock(block)
                rosMsgType = get_param(block,'messageType');
                ros.slros2.internal.bus.Util.createBusIfNeeded(rosMsgType, bdroot(block));
            end
        end

        function dispatch(methodName, varargin)
            obj = ros.slros2.internal.block.MessageBlockMask();
            obj.(methodName)(varargin{:});
        end

        function srvList = getServiceList()
            msgList = ros2("msg","list");
            srvList = extractBefore(msgList(endsWith(msgList,'Request')),'Request');
        end
        function allMsgs = getMessageList()
            msgList = ros2("msg","list");
            allMsgs = setdiff(msgList,[msgList(endsWith(msgList,'Request'));...
                msgList(endsWith(msgList,'Response'))]);
        end

    end
end

