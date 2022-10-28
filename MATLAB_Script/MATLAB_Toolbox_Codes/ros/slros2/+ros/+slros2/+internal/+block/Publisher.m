classdef Publisher < ros.slros2.internal.block.ROS2PubSubBase
%Publish messages to a ROS2 network
%
%   H = ros.slros2.internal.block.Publisher creates a system
%   object, H, that advertises a topic to a ROS network and publishes
%   messages to that topic.
%
%   This system object is intended for use with the MATLAB System
%   block. In order to access the ROS2 functionality from MATLAB, see
%   ROS2PUBLISHER.
%
%   See also ROS2PUBLISHER ROS2SUBSCRIBER

%   Copyright 2019-2021 The MathWorks, Inc.

%#codegen

    properties (Constant,Access=private)
        %MessageCatalogName - Name of this block used in message catalog
        MessageCatalogName = message("ros:slros:blockmask:PublisherMaskTitle").getString
    end

    properties (Access=private, Transient)
        % pPublisher - Private ROS2 Publisher object
        pPublisher

        % Converter - Handle to object that encapsulates converting a
        % Simulink bus struct to a MATLAB ROS message. It is initialized to
        % indicate the class of the object
        Converter = ros.slros2.internal.sim.BusStructToROSMsgConverter.empty
    end

    methods
        function obj = Publisher(varargin)
        % Enable code to be generated even if this file is p-coded
            coder.allowpcode('plain');
            obj = obj@ros.slros2.internal.block.ROS2PubSubBase(varargin{:});
        end
    end

    methods (Access = protected, Static)
        function header = getHeaderImpl
        % Define header panel for System block dialog
            header = matlab.system.display.Header(mfilename("class"), ...
                                                  'ShowSourceLink', false, ...
                                                  'Title', message('ros:slros:blockmask:PublisherMaskTitle').getString, ...
                                                  'Text', message('ros:slros:blockmask:PublisherDescription').getString);
        end

        function throwSimStateError()
            coder.internal.errorIf(true, 'ros:slros:sysobj:BlockSimStateNotSupported', 'ROS 2 Publisher');
        end
    end


    methods (Access = protected)

        function num = getNumInputsImpl(~)
            num = 1;
        end

        function num = getNumOutputsImpl(~)
            num = 0;
        end

    end

    methods (Access = protected)
        function setupImpl(obj, ~)
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
                obj.pPublisher = ros2publisher(modelState.ROSNode, obj.ROSTopic, obj.ROSMessageType, qosArgs{:});
                modelState.incrNodeRefCount();
                obj.Converter = ros.slros2.internal.sim.BusStructToROSMsgConverter(...
                    obj.ROSMessageType, obj.ModelName);
                obj.EmptySeedMsg = ros.slros2.internal.bus.Util.newMessageFromSimulinkMsgType(obj.ROSMessageType);
                [emptyMsg,info]= ros.internal.getEmptyMessage(obj.ROSMessageType,'ros2');
                cachedMap = containers.Map();
                refCachedMap = containers.Map();
                % This map contains the values of empty message data which
                % can be reused when required.
                refCachedMapStoragePath = fullfile(pwd,'+bus_conv_fcns','+ros2','+busToMsg','RefCachedMap.mat');
                if isfile(refCachedMapStoragePath)
                    load(refCachedMapStoragePath, 'refCachedMap'); %#ok<EMLOAD>
                end
                cachedMap(obj.ROSMessageType) = emptyMsg;
                [pkgName,msgName] = fileparts(obj.ROSMessageType);
                obj.ConversionFcn = generateStaticConversionFunctions(obj,emptyMsg,...
                        info,'ros2','busToMsg',pkgName,msgName,cachedMap,refCachedMap,refCachedMapStoragePath);
                %addpath(pwd);
            elseif coder.target('RtwForRapid')
                % Rapid Accelerator. In this mode, coder.target('Rtw')
                % returns true as well, so it is important to check for
                % 'RtwForRapid' before checking for 'Rtw'
                coder.internal.errorIf(true, 'ros:slros2:codegen:RapidAccelNotSupported', 'ROS 2 Publisher');

            elseif coder.target('Rtw')
                coder.cinclude([obj.ModelName, '_', obj.ROS2NodeConst.PubSubCommonHeader]);
                % Append 0 to obj.ROSTopic, since MATLAB doesn't
                % automatically zero-terminate strings in generated code
                zeroDelimTopic = [obj.ROSTopic 0];

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


        function stepImpl(obj,busstruct)
        % Buses are treated as structures
            if coder.target('MATLAB')
                % executing in MATLAB
                thisMsg = obj.ConversionFcn(busstruct,obj.EmptySeedMsg);
                send(obj.pPublisher, thisMsg);
            elseif coder.target('Rtw')
                % The datatype of msg will be derived from the input to the block
                coder.ceval(['ros2::matlab::publish_' obj.BlockId], coder.rref(busstruct));
            end

        end

        %%
        function releaseImpl(obj)
            if coder.target('MATLAB')
                st = ros.slros.internal.sim.ModelStateManager.getState(obj.ModelName);
                st.decrNodeRefCount();
                delete(obj.pPublisher);
                if  ~st.nodeHasReferrers()
                    ros.slros.internal.sim.ModelStateManager.clearState(obj.ModelName);
                end
            end
        end
    end

end
