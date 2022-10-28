classdef Subscriber < ros.slros2.internal.block.ROS2PubSubBase & ...
        ros.internal.mixin.InternalAccess
    %Subscribe to a topic on a ROS2 network
    %
    %   H = ros.slros2.internal.block.Subscriber creates a system
    %   object, H, that subscribes to a topic on a ROS network and
    %   receives messages on that topic.
    %
    %   This system object is intended for use with the MATLAB System
    %   block. In order to access the ROS functionality from MATLAB, see
    %   ROS2SUBSCRIBER.
    %
    %   See also ROS2PUBLISHER ROS2SUBSCRIBER

    %   Copyright 2019-2021 The MathWorks, Inc.

    %#codegen

    % The following should ideally not show up in the MATLAB System block
    % dialog. However, setting them as 'Hidden' will prevent them from
    % being accessible via set_param & get_param
    properties (Constant,Access=private)
        %MessageCatalogName - Name of this block used in message catalog
        MessageCatalogName = message("ros:slros:blockmask:SubscriberMaskTitle").getString
    end

    properties (Access=private, Transient)
        % InputStream - Handle to object that implements ros2subscriber
        pSubscriber = []
        % LastReceivedMsg - Last received "new" message structure
        LastReceivedMsg = []
        % LastConvertedMsg - Last received "new" message converted to SL
        % bus structure
        LastConvertedMsg = [] 
        % LastMessageCount - Number of messages received last time-step
        LastMessageCount

        % Converter - Handle to object that encapsulates converting a
        % Simulink bus struct to a MATLAB ROS message. It is initialized to
        % indicate the class of the object
        Converter = ros.slros2.internal.sim.ROSMsgToBusStructConverter.empty
    end

    methods
        function obj = Subscriber(varargin)
        % Enable code to be generated even if this file is p-coded
            coder.allowpcode('plain');
            obj = obj@ros.slros2.internal.block.ROS2PubSubBase(varargin{:});
        end
    end

    methods (Access = protected)

        function num = getNumInputsImpl(~)
            num = 1;
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
            varargout =  {'logical', obj.SLBusName};
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
                                                  'Title', message('ros:slros:blockmask:SubscriberMaskTitle').getString, ...
                                                  'Text', message('ros:slros:blockmask:SubscriberDescription').getString);
        end

        function throwSimStateError()
            coder.internal.errorIf(true, 'ros:slros:sysobj:BlockSimStateNotSupported', 'ROS 2 Subscriber');
        end

    end

    methods (Access = protected)
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
                obj.pSubscriber = ros2subscriber(modelState.ROSNode, obj.ROSTopic, obj.ROSMessageType, qosArgs{:}, 'EnableCallback', false);
                modelState.incrNodeRefCount();
                obj.Converter = ros.slros2.internal.sim.ROSMsgToBusStructConverter(...
                    obj.ROSMessageType, obj.ModelName);
                obj.LastMessageCount = uint64(0);
                obj.TruncateAction = obj.Converter.MsgInfoMap(obj.ROSMessageType).VarLenTruncateAction;
                obj.EmptySeedBusStruct = obj.Converter.convert(ros2message(obj.ROSMessageType));
                [emptyMsg,info]= ros.internal.getEmptyMessage(obj.ROSMessageType,'ros2');
                cachedMap = containers.Map();
                refCachedMap = containers.Map();
                % This map contains the values of empty message data which
                % can be reused when required.
                refCachedMapStoragePath = fullfile(pwd,'+bus_conv_fcns','+ros2','+msgToBus','RefCachedMap.mat');
                if isfile(refCachedMapStoragePath)
                    load(refCachedMapStoragePath, 'refCachedMap'); %#ok<EMLOAD>
                end
                cachedMap(obj.ROSMessageType) = emptyMsg;
                [pkgName,msgName] = fileparts(obj.ROSMessageType);
                obj.ConversionFcn = generateStaticConversionFunctions(obj,emptyMsg,...
                        info,'ros2','msgToBus',pkgName,msgName,cachedMap,refCachedMap,refCachedMapStoragePath);
                %addpath(pwd);
            elseif coder.target('RtwForRapid')
                % Rapid Accelerator. In this mode, coder.target('Rtw')
                % returns true as well, so it is important to check for
                % 'RtwForRapid' before checking for 'Rtw'
                coder.internal.errorIf(true, 'ros:slros2:codegen:RapidAccelNotSupported', 'ROS2 Subscriber');

            elseif coder.target('Rtw')
                coder.cinclude([obj.ModelName, '_', obj.ROS2NodeConst.PubSubCommonHeader]);
                % Append 0 to obj.ROSTopic, since MATLAB doesn't
                % automatically zero-terminate strings in generated code
                zeroDelimTopic = [obj.ROSTopic 0]; % null-terminated topic name

                qos_profile = coder.opaque('rmw_qos_profile_t', ...
                                           'rmw_qos_profile_default', 'HeaderFile', 'rmw/qos_profiles.h');
                obj.setQOSProfile(qos_profile, obj.QOSHistory, obj.QOSDepth, ...
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

        %%
        function [isNewData, msg] = stepImpl(obj, busstruct)
        % <busstruct> is a blank (empty) bus structure. It is necessary
        % since there is no convenient way to create the (arbitrarily
        % complex and nested) bus structure.
            isNewData = false;
            msg = coder.nullcopy(busstruct);
            if coder.target('MATLAB')
                newMessageCount = obj.pSubscriber.MessageCount;
                if newMessageCount ~= obj.LastMessageCount
                    latestMsg = obj.pSubscriber.LatestMessage;
                    obj.LastMessageCount = newMessageCount;
                    isNewData = true;
                    if isequal(latestMsg, obj.LastReceivedMsg)
                        msg = obj.LastConvertedMsg;
                    else
                        msg = obj.ConversionFcn(latestMsg, obj.EmptySeedBusStruct, obj.TruncateAction, obj.ModelName);
                        obj.LastConvertedMsg = msg;
                        obj.LastReceivedMsg = latestMsg;                        
                    end
                else
                    isNewData = false;
                    if ~isempty(obj.LastConvertedMsg)
                        msg = obj.LastConvertedMsg;
                    end
                end
            elseif coder.target('Rtw')
                isNewData = coder.ceval(['ros2::matlab::getLatestMessage_' obj.BlockId], coder.wref(msg));
            end

        end

        %%
        function releaseImpl(obj)
            if coder.target('MATLAB')
                st = ros.slros.internal.sim.ModelStateManager.getState(obj.ModelName);
                st.decrNodeRefCount();
                try
                    % there may be another subscriber thread running/subscribing, deleting
                    % live thread may result in error
                    delete(obj.pSubscriber);
                catch
                    % catch the error and clear the subscriber property instead of explicitly
                    % calling the delete method
                    obj.pSubscriber = [];
                end
                obj.LastMessageCount = uint64(0);
                if  ~st.nodeHasReferrers()
                    ros.slros.internal.sim.ModelStateManager.clearState(obj.ModelName);
                end
            end
        end
    end


end
