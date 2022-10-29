classdef ServiceCallBlockMask < ros.slros.internal.block.CommonServiceMask
%This class is for internal use only. It may be removed in the future.

%ServiceCallBlockMask - Block mask callbacks for Call Service block

%   Copyright 2018-2020 The MathWorks, Inc.

    properties (Constant)
        %MaskType - Type of block mask
        %   Retrieve is with get_param(gcb, 'MaskType')
        MaskType = 'ROS Call Service'

        %ErrorCodeSinkName - Name of sink block for ErrorCode in subsystem
        %   In practice, this block is either a terminator (if user does
        %   not want the output) or a standard outport.
        ErrorCodeSinkName = 'ErrorCodeOut'
    end

    properties (Constant)
        %% Abstract properties inherited from CommonMask base class
        MaskParamIndex = struct( ...
            'SourceDropdown', 1, ...
            'ServiceNameEdit', 2, ...
            'ServiceTypeEdit', 3 ...
            );

        MaskDlgIndex = struct( ...
            'ServiceSelect', [2 3], ...       % Service Group Box > Service Select Button
            'ServiceTypeSelect', [2 5] ...    % Service Group Box > Service Type Select Button
            );

        SysObjBlockName = 'ServiceCaller';
    end

    methods

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
            ros.internal.setBlockIcon(blkH, 'rosicons.robotlib_callservice');
        end

        function connectionTimeoutEdit(obj, block)
        %connectionTimeoutEdit Validate timeout value specified by user
        %   If the entered timeout value is invalid, keep the current
        %   value.
        %   The parameter on the actual system block is set by the
        %   "updateSubsystem" callback.

            sysobjBlock = [block '/' obj.SysObjBlockName];
            curValue = get_param(sysobjBlock, 'ConnectionTimeout');
            newValue = eval(get_param(block, 'connectionTimeout'));

            try
                validateattributes(newValue, {'numeric'}, {'nonempty', 'scalar', 'positive'}, '', 'ConnectionTimeout');
            catch ex
                set_param(block, 'connectionTimeout', curValue);
                rethrow(ex);
            end
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

            [inputBusDataType,slInputBusName, outputBusDataType, slOutputBusName] = ...
                ros.slros.internal.bus.Util.rosServiceTypeToDataTypeStr(rosServiceType, bdroot(block));

            % note: we use the block id of the parent, not the sys_obj block
            blockId = ros.slros.internal.block.getCppIdentifierForBlock(block, 'ServCall_');
            modelName = bdroot(block);

            connectionTimeout = get_param(block, 'connectionTimeout');
            keepPersistent = get_param(block, 'keepPersistent');

            set_param(sysobjBlock, 'SLOutputBusName', slOutputBusName);
            set_param(sysobjBlock, 'SLInputBusName', slInputBusName);
            set_param(sysobjBlock, 'ServiceType', rosServiceType);
            set_param(sysobjBlock, 'ServiceName', service);
            set_param(sysobjBlock, 'ModelName', modelName);
            set_param(sysobjBlock, 'BlockId', blockId);
            set_param(sigspecBlock, 'OutDataTypeStr', inputBusDataType);
            set_param(constBlock, 'OutDataTypeStr', outputBusDataType);
            set_param(sysobjBlock, 'ConnectionTimeout', connectionTimeout);
            set_param(sysobjBlock, 'IsPersistentConnection',keepPersistent);
        end
    end

    methods(Static)

        function dispatch(methodName, varargin)
        %dispatch Dispatch to Static methods in this class
            obj = ros.slros.internal.block.ServiceCallBlockMask;
            obj.(methodName)(varargin{:});
        end

    end
end
