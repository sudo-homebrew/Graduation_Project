classdef ServiceCallBlockMask < ros.slros.internal.block.ServiceCallBlockMask
%This class is for internal use only. It may be removed in the future.

%ServiceCallerBlockMask - Block mask callbacks for ROS 2 Call Service
%block

%   Copyright 2021 The MathWorks, Inc.

    methods
        function serviceSourceSelect(obj, block)
        %serviceSourceSelect Source of the service name has changed

            maskValues = get_param(block, 'MaskValues');
            maskVisibilities = get_param(block, 'MaskVisibilities');
            maskEnables = get_param(gcb,'MaskEnables');

            mask = Simulink.Mask.get(block);
            dlg = mask.getDialogControls;

            d = obj.MaskDlgIndex.ServiceSelect;
            m = obj.MaskDlgIndex.ServiceTypeSelect;

            if strcmpi(maskValues{obj.MaskParamIndex.SourceDropdown}, obj.TopicSourceSpecifyOwn)
                % Custom topic
                % Enable editing of topic
                maskEnables{obj.MaskParamIndex.ServiceNameEdit} = 'on';
                % Hide Topic selection button
                dlg(d(1)).DialogControls(1).DialogControls(d(2)).Visible = 'off';
                % Show MessageType selection button
                dlg(m(1)).DialogControls(1).DialogControls(m(2)).Visible = 'on';
            else % select topic from ROS network
                 % Disable editing of topic
                maskEnables{obj.MaskParamIndex.ServiceNameEdit} = 'off';
                % Show Topic selection button
                dlg(d(1)).DialogControls(1).DialogControls(d(2)).Visible = 'on';
                % Hide MessageType selection button
                dlg(m(1)).DialogControls(1).DialogControls(m(2)).Visible = 'off';
            end

            set_param(gcb,'MaskEnables', maskEnables);
            set_param(gcb,'MaskVisibilities', maskVisibilities);
        end

        function updateSubsystem(obj, block)
        %updateSubsystem Callback executed on subsystem update

        % There are 3 blocks in the subsystem
        %  * The MATLAB System block with name SysObjBlockName
        %  * The Signal Specification block
        %  * A Constant block
            sysobjBlock = [block '/' obj.SysObjBlockName];
            sigspecBlock = [block '/SignalSpecification'];
            constBlock = [block '/Constant'];

            % Do not canonicalize the service name (i.e., if user entered
            % "foo", don't convert it to "/foo"). This enables user to
            % control whether to have a relative or absolute service name in
            % generated code.
            service = get_param(block, 'service');
            rosServiceType = get_param(block, 'serviceType');

            [inputBusDataType, slInputBusName, outputBusDataType, slOutputBusName] = ...
                ros.slros2.internal.bus.Util.rosServiceTypeToDataTypeStr(rosServiceType);

            % note: we use the block id of the parent, not the sys_obj block
            blockId = ros.slros.internal.block.getCppIdentifierForBlock(block, 'ServCall_');
            modelName = bdroot(block);

            connectionTimeout = get_param(block, 'connectionTimeout');

            set_param(sysobjBlock, 'SLOutputBusName', slOutputBusName);
            set_param(sysobjBlock, 'SLInputBusName', slInputBusName);
            set_param(sysobjBlock, 'ServiceType', rosServiceType);
            set_param(sysobjBlock, 'ServiceName', service);
            set_param(sysobjBlock, 'ModelName', modelName);
            set_param(sysobjBlock, 'BlockId', blockId);
            set_param(sigspecBlock, 'OutDataTypeStr', inputBusDataType);
            set_param(constBlock, 'OutDataTypeStr', outputBusDataType);
            set_param(sysobjBlock, 'ConnectionTimeout', connectionTimeout);
        end

        function serviceEdit(obj, block)
        %serviceEdit - Callback when service name changes

            sysobj_block = [block '/' obj.SysObjBlockName];
            curValue = get_param(sysobj_block, 'ServiceName');
            newValue = get_param(block,'service');

            % Check for validity and make sure that the name is a valid
            % ROS2 name
            if ~ros.internal.Namespace.isValidGraphName(newValue)
                set_param(block, 'service', curValue);
                error(message('ros:slros:svccaller:InvalidServiceName', newValue));
            end

            obj.messageLoadFcn(block);
            obj.updateSubsystem(block);
        end

        function maskInitialize(obj, block)
        %maskInitialize Mask initialization callback
        %   It is invoked when the user:
        %   * Changes the value of a mask parameter by using the block dialog box or set_param.
        %   * Changes any of the parameters that define the mask
        %   * Causes the icon to be redrawn
        %   * Copies the block
        %
        %   Mask initialization is invoked after the individual parameter
        %   callbacks

        % Show or hide the ErrorCode output port
            showErrorCode = get_param(block, 'ShowErrorCodeOutput');

            % existingErrorCodeSink is the sink block for the ErrorCode output in
            % the current subsystem. This can either be a standard outport,
            % or a terminator.
            existingErrorCodeSink = [block '/' obj.ErrorCodeSinkName];

            % Determine what type the sink block should be based on the
            % checkbox setting on the mask.
            if strcmp(showErrorCode, 'on')
                newErrorCodeSink = sprintf('built-in/Outport');
            else
                newErrorCodeSink = sprintf('built-in/Terminator');
            end

            % Only modify the subsystem if new block type is different
            existingOutportType = get_param(existingErrorCodeSink, 'BlockType');
            newOutportType = get_param(newErrorCodeSink, 'BlockType');
            if ~strcmp(existingOutportType, newOutportType)
                % Preserve orientation and position to ensure that the
                % existing signal line connects without any issues.
                orient  = get_param(existingErrorCodeSink, 'Orientation');
                pos     = get_param(existingErrorCodeSink, 'Position');
                delete_block(existingErrorCodeSink);
                add_block(newErrorCodeSink, existingErrorCodeSink, ...
                          'Name',        obj.ErrorCodeSinkName, ...
                          'Orientation', orient, ...
                          'Position',    pos);
            end

            % set block mask display
            blkH = get_param(block, 'handle');
            serviceType = get_param(block, 'service');
            maskDisplayText = sprintf('color(''black'');');
            if length(serviceType) > 20
                maskDisplayText = sprintf('%s\ntext(0.95, 0.15, ''%s'', ''horizontalAlignment'', ''right'');', ...
                                          maskDisplayText, serviceType);
            else
                maskDisplayText = sprintf('%s\ntext(0.5, 0.15, ''%s'', ''horizontalAlignment'', ''center'');', ...
                                          maskDisplayText, serviceType);
            end
            maskDisplayText = sprintf('%s\nport_label(''input'', 1, ''Req'');',maskDisplayText);
            maskDisplayText = sprintf('%s\nport_label(''output'', 1, ''Resp'');',maskDisplayText);
            if isequal(get_param(block, 'ShowErrorCodeOutput'), 'on')
                maskDisplayText = sprintf('%s\nport_label(''output'', 2, ''ErrorCode'');',maskDisplayText);
            end
            set_param(blkH, 'MaskDisplay', maskDisplayText);
            ros.internal.setBlockIcon(blkH, 'rosicons.ros2lib_callservice');
        end
    end

    methods(Static)
        function ret = getMaskType()
            ret = 'ROS2 Call Service';
        end

        function messageLoadFcn(block)
            serviceType = get_param(block,'serviceType');
            ros.slros2.internal.bus.Util.createBusIfNeeded(strcat(serviceType,'Request'),bdroot(block));
            ros.slros2.internal.bus.Util.createBusIfNeeded(strcat(serviceType,'Response'),bdroot(block));
        end

        function dispatch(methodName, varargin)
        %dispatch Dispatch to Static methods in this class
            obj = ros.slros2.internal.block.ServiceCallBlockMask;
            obj.(methodName)(varargin{:});
        end

    end
end
