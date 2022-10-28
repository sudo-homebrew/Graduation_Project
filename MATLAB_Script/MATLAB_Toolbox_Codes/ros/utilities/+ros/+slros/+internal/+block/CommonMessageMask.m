classdef (Abstract) CommonMessageMask < ros.slros.internal.block.CommonMask
%CommonMessageMask Common base class for ROS message producers and consumers
%   In particular, this includes Publisher, Subscriber, and Blank
%   Message blocks.

%   Copyright 2015-2020 The MathWorks, Inc.

    methods
        %% ModelCloseFcn
        function modelCloseFcn(~, block)
        %modelCloseFcn Called when the model is closed

        % Avoid clearing buses when library is closed
        % (prevents clearing when library unloaded on simulation)
            if ~bdIsLibrary(bdroot(block))
                ros.slros.internal.bus.clearBusesOnModelClose(block);
            end
        end
    end

    methods
        %% Block InitFcns

        % Callbacks for individual params are invoked when:
        % * User opens a mask dialog
        % * User modifies a param value and changes focus
        % * User modifies a param value and selects OK or Apply
        % * The model is updated (user presses Ctrl+D or simulates the model)

        function constantBlkInitFcn(obj, constantBlk)                     %#ok<INUSL>
            block = get_param(constantBlk, 'parent');
            rosMsgType = get_param(block, 'messageType');
            locCreateBusIfNeeded(block, rosMsgType, bdroot(constantBlk));
        end

        function sysobjInitFcn(obj, sysobjBlock) %#ok<INUSL>
            rosMsgType = get_param(sysobjBlock, 'ROSMessageType');
            expectedBusName = ros.slros.internal.bus.Util.rosMsgTypeToBusName(rosMsgType, bdroot(sysobjBlock));

            currentBusName = get_param(sysobjBlock, 'SLBusName');
            if ~strcmp(currentBusName, expectedBusName)
                % Do not mark the model dirty when the Simulink bus name
                % is updated on the underlying mask of Simulink ROS I/O
                % blocks.
                % All changes to model is ignored when this
                % "preserveDirty" variable is in Scope, hence this only
                % encompasses the set_param call in this if-else section.
                preserveDirty = Simulink.PreserveDirtyFlag(bdroot(sysobjBlock),'blockDiagram');  %#ok<NASGU>
                set_param(sysobjBlock, 'SLBusName', expectedBusName);
            end
        end
    end

    %%
    methods
        %% Callbacks
        % Callbacks for individual params are invoked when the user:
        % * Opens a mask dialog
        % * Modifies a param value and changes focus
        % * Modifies a param value and selects OK or Apply
        % * Updates the model (presses Ctrl+D or simulates the model)
        %
        % Note - these are **not** invoked when user does a SET_PARAM

        function topicSourceSelect(obj, block)
            maskValues = get_param(block, 'MaskValues');
            maskVisibilities = get_param(block, 'MaskVisibilities');
            maskEnables = get_param(block,'MaskEnables');

            mask = Simulink.Mask.get(block);
            dlg = mask.getDialogControls;

            d = obj.MaskDlgIndex.TopicSelect;
            m = obj.MaskDlgIndex.MessageTypeSelect;

            if strcmpi(maskValues{obj.MaskParamIndex.TopicSourceDropdown}, obj.TopicSourceSpecifyOwn)
                % Custom topic
                % Enable editing of topic
                maskEnables{obj.MaskParamIndex.TopicEdit} = 'on';
                % Hide Topic selection button
                dlg(d(1)).DialogControls(d(2)).DialogControls(d(3)).Visible = 'off';
                % Show MessageType selection button
                dlg(m(1)).DialogControls(m(2)).DialogControls(m(3)).Visible = 'on';
            else % select topic from ROS network
                 % Disable editing of topic
                maskEnables{obj.MaskParamIndex.TopicEdit} = 'off';
                % Show Topic selection button
                dlg(d(1)).DialogControls(d(2)).DialogControls(d(3)).Visible = 'on';
                % Hide MessageType selection button
                dlg(m(1)).DialogControls(m(2)).DialogControls(m(3)).Visible = 'off';
            end

            set_param(block,'MaskEnables', maskEnables);
            set_param(block,'MaskVisibilities', maskVisibilities);
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
            locCreateBusIfNeeded(block, rosMsgType, bdroot(block));
            obj.updateSubsystem(block);
        end


        function messageTypeEdit(obj, block)
            rosMsgType = get_param(block, 'messageType');
            locCreateBusIfNeeded(block, rosMsgType, bdroot(block));
            obj.updateSubsystem(block);
        end


        function topicSelect(~, block, getDlgFcn)
            try
                msgDlg = feval(getDlgFcn);
                msgDlg.openDialog(@dialogCloseCallback);
            catch ME
                % Send error to Simulink diagnostic viewer rather than a
                % DDG dialog.
                % NOTE: This does NOT stop execution.
                reportAsError(MSLDiagnostic(ME));
            end

            function dialogCloseCallback(isAcceptedSelection, selectedTopic, selectedMsgType)
                if isAcceptedSelection
                    set_param(block, 'topic', selectedTopic);
                    set_param(block, 'messageType', selectedMsgType);
                end
            end
        end

        function messageTypeSelect(~, block, getDlgFcn)
            currentMsgType = get_param(block, 'messageType');

            msgDlg = feval(getDlgFcn);
            msgDlg.openDialog(currentMsgType, @dialogCloseCallback);

            function dialogCloseCallback(isAcceptedSelection, selectedMsg)
                if isAcceptedSelection
                    set_param(block, 'messageType', selectedMsg);
                end
            end
        end
    end
    
end

function locCreateBusIfNeeded(block, rosMsgType, mdlName)

hasLegacyStructParam = contains(get_param(block, 'MaskVariables'),'legacyBusStruct');
if hasLegacyStructParam
    if isequal(get_param(block, 'legacyBusStruct'),'on')
        ros.slros.internal.bus.Util.createBusIfNeeded(rosMsgType, mdlName);
    else
        ros.slroscpp.internal.bus.Util.createBusIfNeeded(rosMsgType, mdlName);
    end
else
    ros.slros.internal.bus.Util.createBusIfNeeded(rosMsgType, mdlName);
end
end

