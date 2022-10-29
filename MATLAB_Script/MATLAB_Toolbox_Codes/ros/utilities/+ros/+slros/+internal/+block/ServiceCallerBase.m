classdef (Abstract) ServiceCallerBase < ...
        matlab.System & ros.slros.internal.block.mixin.NodeDependent

    %This class is for internal use only. It may be removed in the future.

    %ServiceCallerBase Call service on ROS network and receive response
    %
    %   H = ros.slros.internal.block.ServiceCaller creates a system
    %   object, H, that sends a request to a service server on the ROS network and
    %   outputs the response message received
    %
    %   This system object is intended for use with the MATLAB System
    %   block. In order to access the ROS functionality from MATLAB, see
    %   ROSSVCCLIENT
    %
    %   See also rossvcclient.

    %   Copyright 2018-2021 The MathWorks, Inc.

    %#codegen

    properties (Nontunable)
        %ServiceName - Service name
        %   This system object will use ServiceName as specified in both
        %   simulation and code generation. In particular, it will not add a
        %   "/" in front of service, as that forces the service to be in the
        %   absolute namespace.
        ServiceName = '/my_service'

        %ServiceType - Service type
        %   Default: 'std_srvs/Empty'
        ServiceType = 'std_srvs/Empty'

        %ConnectionTimeout - Timeout for service server connection (in seconds)
        %   Default: 5 (s)
        ConnectionTimeout = 5

        %IsPersistentConnection - Indication if connection is persistent
        %   Default: false
        IsPersistentConnection = 'off'
    end

    % The following should ideally not show up in the MATLAB System block
    % dialog. However, setting them as 'Hidden' will prevent them from
    % being accessible via set_param & get_param.
    properties(Nontunable)
        %SLInputBusName - Simulink Bus Name for input (service request)
        SLInputBusName = ''

        %SLOutputBusName - Simulink Bus Name for output (service response)
        SLOutputBusName = ''

        %StreamWSVar - Workspace stream
        %   Used to redirect system object to get service responses from a
        %   workspace variable instead of from ROS network (used for testing only).
        %   The workspace variable should be created like this:
        %
        %   workspaceVar = ros.slros.internal.sim.ServiceCallListStream;
        %   workspaceVar.ResponseList = {rosmessage('std_srvs/EmptyRequest'), uint8(0)}
        %
        %   This capability of using workspace variables is only used in
        %   internal testing, and is not documented for Simulink users.
        StreamWSVar = ''

        %ModelName Name of Simulink model
        %   Used for managing node instance
        ModelName = 'untitled'

        %BlockId Simulink Block Identifier
        %  Used to generate unique identifier for the block during code
        %  generation. This should be obtained using Simulink.ID.getSID()
        %  on the library block (*not* the MATLAB system block). The
        %  SID has the format '<modelName>:<blocknum>'
        BlockId = 'serv1'
    end

    properties (Constant, Access=?ros.slros.internal.block.mixin.NodeDependent)
        %MessageCatalogName - Name of this block used in message catalog
        %   This property is used by the NodeDependent base class to
        %   customize error messages with the block name.

        %   Due a limitation in Embedded MATLAB code-generation with UTF-8 characters,
        %   use English text instead of message("ros:slros:svccaller:MaskTitle").getString
        MessageCatalogName = 'ROS Call Service'
    end

    properties(Constant, Access=private)
        %HeaderFile - Name of header file with declarations for variables and types
        %   This is used in code emitted by setupImpl and stepImpl.
        HeaderFile = ros.slros.internal.cgen.Constants.InitCode.HeaderFile
    end

    properties (Abstract, Access = ?matlab.unittest.TestCase, Transient)
        %InputConverter - Conversion for service request bus
        InputConverter

        %OutputConverter - Conversion for service response bus
        OutputConverter 
    end

    properties (Access = ?matlab.unittest.TestCase, Transient)        
        %ServiceStream - Connection to service
        ServiceStream
    end

    methods
        function set.ServiceName(obj, val)
            validateattributes(val, {'char'}, {'nonempty'}, '', 'ServiceName');
            if coder.target('MATLAB')
                ros.internal.Namespace.canonicalizeName(val);
            end
            obj.ServiceName = val;
        end

        function set.ConnectionTimeout(obj,val)
            validateattributes(val, {'numeric'}, {'nonempty', 'scalar', 'positive'}, '', 'ConnectionTimeout');
            obj.ConnectionTimeout = double(val);
        end

        function set.SLOutputBusName(obj, val)
            validateattributes(val, {'char'}, {}, '', 'SLOutputBusName');
            obj.SLOutputBusName = val;
        end

        function set.SLInputBusName(obj, val)
            validateattributes(val, {'char'}, {}, '', 'SLInputBusName');
            obj.SLInputBusName = val;
        end

        function set.ModelName(obj, val)
            validateattributes(val, {'char'}, {'nonempty'}, '', 'ModelName');
            obj.ModelName = val;
        end

        function set.BlockId(obj, val)
            validateattributes(val, {'char'}, {'nonempty'}, '', 'BlockId');
            obj.BlockId = val;
        end
    end

    methods (Access = protected)

        function num = getNumInputsImpl(~)
            num = 2;
        end

        function num = getNumOutputsImpl(~)
            num = 2;
        end
        function varargout = getOutputSizeImpl(~)
            varargout = {[1 1], [1 1]};
        end

        function varargout = isOutputFixedSizeImpl(~)
            varargout =  {true, true};
        end

        function varargout = getOutputDataTypeImpl(obj)
            varargout =  {obj.SLOutputBusName, 'uint8'};
        end

        function varargout = isOutputComplexImpl(~)
            varargout = {false, false};
        end

    end

    methods (Abstract, Access = protected)
        setConverters(obj)
    end
    methods (Access = protected)
        %% Overloaded system object functions
        function sts = getSampleTimeImpl(obj)
        % Enable this system object to inherit constant ('inf') sample times
            sts = createSampleTime(obj, 'Type', 'Inherited', 'Allow', 'Constant');
        end
        
        function setupImpl(obj, ~, ~)
        %setupImpl System block initialization

            obj.setupNodeDependent;

            if coder.target("MATLAB")
                % Interpreted execution
                setConverters(obj);
            elseif coder.target("Rtw")
                % ROS node generation

                coder.cinclude(obj.HeaderFile);
            end

        end

        function [respMsg, errorCode] = stepImpl(obj, inputbusstruct, outputbusstruct)
        %stepImpl Call the service and return a response

            successCode = uint8(ros.slros.internal.block.ServiceCallErrorCode.SLSuccess);
            errorCode = successCode;

            if coder.target('MATLAB')
                % Executing in interpreted mode

                respMsg = outputbusstruct; %return empty bus if ErrCode ~= 0

                % Execute a step
                reqMsg = obj.InputConverter.convert(inputbusstruct);
                [resp, errorCode] = obj.ServiceStream.callService(reqMsg);

                if errorCode == successCode
                    respMsg = obj.OutputConverter.convert(resp);
                end
            elseif coder.target("Rtw")
                % Execute in ROS node generation

                % Ensure that output is always assigned
                respMsg = coder.nullcopy(outputbusstruct);

                connectionTimeout = obj.ConnectionTimeout;

                % In C++ code an infinite timeout is denoted with -1
                if (connectionTimeout == Inf)
                    connectionTimeout = -1;
                end

                % Append \0 to service name, since MATLAB does not
                % automatically zero-terminate strings in generated code
                zeroDelimServiceName = [obj.ServiceName 0];

                isCreated = false;
                isCreated = coder.ceval([obj.BlockId '.getIsCreated']);

                % Create the service client, if needed
                if( ~isCreated )
                    errorCode = coder.ceval([obj.BlockId '.createServiceCaller'], ...
                                            zeroDelimServiceName, strcmp(obj.IsPersistentConnection, 'on'), ...
                                            connectionTimeout);

                    if errorCode ~= uint8(ros.slros.internal.block.ServiceCallErrorCode.SLSuccess)
                        return;
                    end
                end

                % Call the service client
                errorCode = coder.ceval([obj.BlockId '.call'], coder.rref(inputbusstruct), coder.wref(respMsg));
            end
        end

        function releaseImpl(obj)
        %releaseImpl Release the system object

            obj.releaseNodeDependent;
        end

    end

    methods (Access = protected)
        %% Methods that are implementations of abstract NodeDependent mixin
        function initializeDataStream(obj, modelState)
        %initializeDataStream Initialize data stream
        %   The stream is either connected to the ROS network or
        %   connected to a workspace variable.

            if ~isempty(obj.StreamWSVar)
                % Use a variable in MATLAB Workspace as source of service responses.
                obj.ServiceStream = evalin('base', obj.StreamWSVar);
                validateattributes(obj.ServiceStream, "ros.slros.internal.sim.ServiceCallListStream", "scalar", ...
                                   "initializeDataStream", "ServiceStream"); 
            else
                % Use ROS network as a source of service responses.
                if isempty(modelState.ROSNode) || ~isvalid(modelState.ROSNode)
                    obj.ROSMaster = ros.slros.internal.sim.ROSMaster;
                    %  verifyReachable() errors if ROS master is unreachable
                    obj.ROSMaster.verifyReachable;
                    % createNode() errors if unable to create node
                    % (e.g., if node with same name already exists)
                    uniqueName = obj.ROSMaster.makeUniqueName(obj.ModelName);
                    modelState.ROSNode = obj.ROSMaster.createNode(uniqueName);
                end

                % Create the stream
                obj.ServiceStream = ros.slros.internal.sim.ServiceCallNetStream(modelState.ROSNode);
            end

            obj.ServiceStream.ServiceName = obj.ServiceName;
            obj.ServiceStream.ConnectionTimeout = obj.ConnectionTimeout;
            obj.ServiceStream.IsConnectionPersistent = (obj.IsPersistentConnection == "on");
        end

        function name = modelName(obj)
        %modelName Retrieve model name

            name = obj.ModelName;
        end
    end

    methods(Access = protected, Static)
        function simMode = getSimulateUsingImpl
        %getSimulateUsingImpl Restrict simulation mode to interpreted execution
            simMode = "Interpreted execution";
        end

        function flag = showSimulateUsingImpl
        %showSimulateUsingImpl Do now show simulation execution mode dropdown in block mask
            flag = false;
        end
    end

end
