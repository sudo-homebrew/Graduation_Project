classdef ServiceCaller < ros.slros2.internal.block.ROS2ServiceCallerBase & ...
        ros.internal.mixin.InternalAccess
%ServiceCaller call for a service on a ROS2 network
%
%   H = ros.slros2.internal.block.ServiceCaller creates a system
%   object, H, that sends a request to a service server on the ROS2
%   network and outputs the response message received
%
%   This system object is intended for use with the MATLAB System
%   block. In order to access the ROS functionality from MATLAB, see
%   ROS2SVCCLIENT.
%
%   See also ros2svcclient.

%   Copyright 2021 The MathWorks, Inc.

%#codegen

% The following should ideally not show up in the MATLAB System block
% dialog. However, setting them as 'Hidden' will prevent them from
% being accessible via set_param & get_param.
    properties (Constant, Access=private)
        % MessageCatelogName - Name of this block used in messag catelog
        MessageCatelogName = message("ros:slros2:blockmask:ServiceCallerMaskTitle").getString
    end

    properties (Access=private, Transient)
        % pSvcClient - Handle to object that implements ros2svcclient
        pSvcClient = []
        % OutputConverter - Conversion for service response bus
        OutputConverter = ros.slros2.internal.sim.ROSMsgToBusStructConverter.empty
    end

    properties (Access = protected)
        % InputConversionFcn Conversion function for input message
        InputConversionFcn
        % OutputConversionFcn Conversion function for output message
        OutputConversionFcn
        % EmptySeedInputMsg Empty Seed Input ROSMessage
        EmptySeedInputMsg
        % EmptySeedOutputBusStruct Empty Seed output ROSMessage
        EmptySeedOutputBusStruct
    end

    methods
        function obj = ServiceCaller(varargin)
        % Enable code to be generated even if this file is p-coded
            coder.allowpcode('plain');
            obj = obj@ros.slros2.internal.block.ROS2ServiceCallerBase(varargin{:});
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
            varargout = {true, true};
        end

        function varargout = getOutputDataTypeImpl(obj)
            varargout =  {obj.SLOutputBusName, 'uint8'};
        end

        function varargout = isOutputComplexImpl(~)
            varargout = {false, false};
        end
    end

    methods (Access = protected, Static)
        function header = getHeaderImpl
        % Define header panel for System block dialog
            header = matlab.system.display.Header(mfilename("class"), ...
                                                  'ShowSourceLink', false, ...
                                                  'Title', message('ros:slros2:blockmask:ServiceCallerMaskTitle').getString, ...
                                                  'Text', message('ros:slros2:blockmask:ServiceCallerDescription').getString);
        end

        function throwSimStateError()
            coder.internal.errorIf(true, 'ros:slros:sysobj:BlockSimStateNotSupported', 'ROS2 Call Service');
        end
    end

    methods (Access = protected)
        function sts = getSampleTimeImpl(obj)
        % Enable this system object to inherit constant ('inf') sample
        % times
            sts = createSampleTime(obj,'Type','Inherited','Allow','Constant');
        end

        function setupImpl(obj)
        % setupImpl is called when model is being initialized at the
        % start of a simulation
            if coder.target('MATLAB')
                % Executing in MATLAB interpreted mode
                modelState = ros.slros.internal.sim.ModelStateManager.getState(obj.ModelName, 'create');
                % The following could be a separate method, but system
                % object infrastructure doesn't appear to allow it
                if isempty(modelState.ROSNode) || ~isvalid(modelState.ROSNode)
                    uniqueName = obj.makeUniqueName(obj.ModelName);
                    modelState.ROSNode = ros2node(uniqueName, ...
                                                  ros.ros2.internal.NetworkIntrospection.getDomainIDForSimulink, ...
                                                  'RMWImplementation', ...
                                                   ros.ros2.internal.NetworkIntrospection.getRMWImplementationForSimulink);
                end
                qosArgs = getQOSArguments(obj);
                obj.pSvcClient = ros2svcclient(modelState.ROSNode, obj.ServiceName, obj.ServiceType, qosArgs{:});
                %TODO: Add errorCode for connection timeout when
                %waitForService is available in ros2svcclient
                modelState.incrNodeRefCount();
                obj.OutputConverter = ros.slros2.internal.sim.ROSMsgToBusStructConverter(...
                    strcat(obj.ServiceType, 'Response'), obj.ModelName);
                obj.EmptySeedInputMsg = ros.slros2.internal.bus.Util.newMessageFromSimulinkMsgType([obj.ServiceType 'Request']);
                emptySeedOutputMsg = ros.slros2.internal.bus.Util.newMessageFromSimulinkMsgType([obj.ServiceType 'Response']);
                obj.EmptySeedOutputBusStruct = obj.OutputConverter.convert(emptySeedOutputMsg);
                [emptyInputMsg,inputMsgInfo]= ros.internal.getEmptyMessage([obj.ServiceType 'Request'],'ros2');
                [emptyOutputMsg,outputMsgInfo]= ros.internal.getEmptyMessage([obj.ServiceType 'Response'],'ros2');
                cachedMap = containers.Map();
                refCachedMap = containers.Map();
                % This map contains the values of empty message data which
                % can be reused when required.
                refCachedMapStoragePath = fullfile(pwd,'+bus_conv_fcns','+ros2','+busToMsg','RefCachedMap.mat');
                if isfile(refCachedMapStoragePath)
                    load(refCachedMapStoragePath, 'refCachedMap'); %#ok<EMLOAD>
                end
                cachedMap([obj.ServiceType 'Request']) = emptyInputMsg;
                [pkgNameIn,msgNameIn] = fileparts([obj.ServiceType 'Request']);
                cachedMap([obj.ServiceType 'Response']) = emptyOutputMsg;
                [pkgNameOut,msgNameOut] = fileparts([obj.ServiceType 'Response']);
                obj.InputConversionFcn = generateStaticConversionFunctions(obj,emptyInputMsg,...
                                                                           inputMsgInfo,'ros2','busToMsg',pkgNameIn,msgNameIn,cachedMap,refCachedMap,refCachedMapStoragePath);
                obj.OutputConversionFcn = generateStaticConversionFunctions(obj,emptyOutputMsg,...
                                                                            outputMsgInfo,'ros2','msgToBus',pkgNameOut,msgNameOut,cachedMap,refCachedMap,refCachedMapStoragePath);

            elseif coder.target('RtwForRapid')
                % Rapid Accelerator. In this mode, coder.target('Rtw')
                % returns true as well, so it is important to check for
                % 'RtwForRapid' before checking for 'Rtw'
                coder.internal.errorIf(true, 'ros:slros2:codegen:RapidAccelNotSupported', 'ROS2 Call Service');
            elseif coder.target('Rtw')
                coder.cinclude([obj.ModelName, '_', obj.ROS2NodeConst.PubSubCommonHeader]);
                % Append 0 to obj.ServiceName, since MATLAB doesn't
                % automatically zero-terminate strings in generated code
                zeroDelimTopic = [obj.ServiceName 0]; % null-terminated topic name

                qos_profile = coder.opaque('rmw_qos_profile_t', ...
                                           'rmw_qos_profile_default', 'HeaderFile', 'rmw/qos_profiles.h');
                obj.setQOSProfile(qos_profile, obj.QOSHistory, obj.getDepth, ...
                                  obj.QOSReliability, obj.QOSDurability);
                coder.ceval(['ros2::matlab::create_' obj.BlockId], ...
                            zeroDelimTopic, qos_profile);
            elseif  coder.target('Sfun')
                % 'Sfun'  - Simulation through CodeGen target
                % Do nothing. MATLAB System block first does a pre-codegen
                % compile with 'Sfun' target, & then does the "proper"
                % codegen compile with Rtw or RtwForRapid, as appropriate.

            else
                % 'RtwForSim' - ModelReference SIM target
                % 'MEX', 'HDL', 'Custom' - Not applicable to MATLAB System block
                coder.internal.errorIf(true, 'ros:slros:sysobj:UnsupportedCodegenMode', coder.target);
            end
        end

        function [respMsg, errorCode] = stepImpl(obj, inputbusstruct, outputbusstruct)
        % stepImpl - Call the service and return a response

            successCode = uint8(ros.slros.internal.block.ServiceCallErrorCode.SLSuccess);
            errorCode = successCode;

            if coder.target('MATLAB')
                % Execute in interpreted mode
                respMsg = outputbusstruct; %return empty bus if ErrorCode ~= 0

                % Execute a step
                thisMsg = obj.InputConversionFcn(inputbusstruct, obj.EmptySeedInputMsg);

                if obj.pSvcClient.waitForServer("Timeout",obj.ConnectionTimeout)
                    [respFromML, status, ~] = obj.pSvcClient.call(thisMsg);
                else
                    % Failed to connect to server on time
                    errorCode = uint8(ros.slros.internal.block.ServiceCallErrorCode.SLConnectionTimeout);
                    return;
                end

                if ~status
                    % Failed to receive valid response
                    errorCode = uint8(ros.slros.internal.block.ServiceCallErrorCode.SLCallFailure);
                    return;
                end
                % Successfully receive valid response message
                respMsg = obj.OutputConversionFcn(respFromML, obj.EmptySeedOutputBusStruct);
            elseif coder.target("Rtw")
                % Service Client has been created in setupImpl, so there is
                % no need to create again.

                % Ensure that output is always assigned
                respMsg = coder.nullcopy(outputbusstruct);

                connectionTimeout = obj.ConnectionTimeout;
                % In C++ code an infinite timeout is denoted with -1
                if (connectionTimeout == Inf)
                    connectionTimeout = -1;
                end
                serverAvailableOnTime = false;
                serverAvailableOnTime = coder.ceval(['ros2::matlab::waitForServer_' obj.BlockId], connectionTimeout);
                if ~serverAvailableOnTime
                    % Failed to connect to server on time
                    errorCode = uint8(ros.slros.internal.block.ServiceCallErrorCode.SLConnectionTimeout);
                    return;
                end

                % Call the service
                errorCode = coder.ceval(['ros2::matlab::call_' obj.BlockId], coder.rref(inputbusstruct), coder.wref(respMsg));
            end
        end

        function releaseImpl(obj)
            if coder.target('MATLAB')
                st = ros.slros.internal.sim.ModelStateManager.getState(obj.ModelName);
                st.decrNodeRefCount();
                try
                    delete(obj.pSvcClient);
                catch
                    obj.pSvcClient = [];
                end
                if  ~st.nodeHasReferrers()
                    ros.slros.internal.sim.ModelStateManager.clearState(obj.ModelName);
                end
            end
        end
    end
end
