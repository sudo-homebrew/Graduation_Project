classdef Publisher < ros.slroscpp.internal.block.ROSPubSubBase & ...
        ros.internal.mixin.ROSInternalAccess
    %Publish messages to a ROS network
    %
    %   H = ros.slros.internal.block.Publisher creates a system
    %   object, H, that advertises a topic to a ROS network and publishes
    %   messages to that topic.
    %
    %   This system object is intended for use with the MATLAB System
    %   block. In order to access the ROS functionality from MATLAB, see
    %   ROSPUBLISHER.
    %
    %   See also ROSSUBSCRIBER

    %   Copyright 2019-2021 The MathWorks, Inc.

    %#codegen

    properties (Nontunable)
        %OutputStreamWsVar Output message to variable in base workspace
        %  Used to redirect Publisher to send messages to a workspace
        %  variable instead of to ROS network (used for testing only).
        %  The workspace variable should be created like this:
        %
        %   workspaceVar =  ros.slros.internal.sim.MsgListOutputStream(rostopic, msgtype);
        %
        % After simulation, workspaceVar.MessageList has the list of
        % messages that have been sent to MsgListOutputStream object.
        % This capability of using workspace variables is only used in
        % internal testing, and is not documented for Simulink users.
        OutputStreamWsVar = ''
    end

    properties (Constant,Access=private)
        %MessageCatalogName - Name of this block used in message catalog
        MessageCatalogName = message("ros:slros:blockmask:PublisherMaskTitle").getString
    end

    properties (Access=private, Transient)
        % pPublisher - Private ROS Publisher object
        pPublisher

        % Converter - Handle to object that encapsulates converting a
        % Simulink bus struct to a MATLAB ROS message. It is initialized to
        % indicate the class of the object
        Converter = ros.slroscpp.internal.sim.BusStructToROSMsgConverter.empty

        % ROSMaster - Handle to an object that encapsulates interaction
        % with the ROS master. It is initialized to indicate the class of
        % the object.
        ROSMaster = ros.slros.internal.sim.ROSMaster.empty
    end

    methods
        function obj = Publisher(varargin)
        % Enable code to be generated even if this file is p-coded
            coder.allowpcode('plain');
            obj = obj@ros.slroscpp.internal.block.ROSPubSubBase(varargin{:});
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
            coder.internal.errorIf(true, 'ros:slros:sysobj:PublisherSimStateNotSupported');
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
                modelHasPriorState = ros.slros.internal.sim.ModelStateManager.hasState(obj.ModelName);
                nodeRefCountIncremented = false;

                try
                    modelState = ros.slros.internal.sim.ModelStateManager.getState(obj.ModelName, 'create');
                    % The following could be a separate method, but system
                    % object infrastructure doesn't appear to allow it
                    if isempty(modelState.ROSNode) || ~isvalid(modelState.ROSNode)
                        obj.ROSMaster = ros.slros.internal.sim.ROSMaster();
                        %  verifyReachable() errors if ROS master is unreachable
                        obj.ROSMaster.verifyReachable();
                        % createNode() errors if unable to create node
                        % (e.g., if node with same name already exists)
                        uniqueName = obj.ROSMaster.makeUniqueName(obj.ModelName);
                        modelState.ROSNode = obj.ROSMaster.createNode(uniqueName);
                    end
                    obj.pPublisher = ros.Publisher(modelState.ROSNode, ...
                        obj.ROSTopic, ...
                        obj.ROSMessageType, ...
                        'DataFormat', 'struct');
                    modelState.incrNodeRefCount();
                    nodeRefCountIncremented = true;
                    obj.EmptySeedMsg = ros.slroscpp.internal.bus.Util.newMessageFromSimulinkMsgType(obj.ROSMessageType);
                    [emptyMsg,info]= ros.internal.getEmptyMessage(obj.ROSMessageType,'ros');
                    cachedMap = containers.Map();
                    refCachedMap = containers.Map();
                    % This map contains the values of empty message data
                    % which can be reused when required.
                    refCachedMapStoragePath = fullfile(pwd,'+bus_conv_fcns','+ros','+busToMsg','RefCachedMap.mat');
                    if isfile(refCachedMapStoragePath)
                        load(refCachedMapStoragePath, 'refCachedMap'); %#ok<EMLOAD> 
                    end
                    cachedMap(obj.ROSMessageType) = emptyMsg;
                    [pkgName,msgName] = fileparts(obj.ROSMessageType);
                    obj.ConversionFcn = generateStaticConversionFunctions(obj,emptyMsg,...
                        info,'ros','busToMsg',pkgName,msgName,cachedMap,refCachedMap,refCachedMapStoragePath);
                    %addpath(pwd);
                    obj.Converter = ros.slroscpp.internal.sim.BusStructToROSMsgConverter(...
                        obj.ROSMessageType, obj.ModelName);
                catch ME
                    if nodeRefCountIncremented
                        modelState.decrNodeRefCount();
                    end
                    if ~modelHasPriorState || ~modelState.nodeHasReferrers()
                        ros.slros.internal.sim.ModelStateManager.clearState(obj.ModelName);
                    end
                    % RETHROW will generate a hard-to-read stack trace, so
                    % use THROW instead.
                    throw(ME);
                end
            elseif coder.target('RtwForRapid')
                % Rapid Accelerator. In this mode, coder.target('Rtw')
                % returns true as well, so it is important to check for
                % 'RtwForRapid' before checking for 'Rtw'
                coder.internal.errorIf(true, 'ros:slros:sysobj:PublisherRapidAccelNotSupported');

            elseif coder.target('Rtw')
                coder.cinclude(obj.HeaderFile);
                % Append 0 to obj.ROSTopic, since MATLAB doesn't
                % automatically zero-terminate strings in generated code
                zeroDelimTopic = [obj.ROSTopic 0];
                coder.ceval([obj.BlockId, '.createPublisher'], ...
                            zeroDelimTopic, int32(obj.MessageQueueLen));

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
        % Simulink Buses are treated as structures in MATLAB
            if coder.target('MATLAB')
                % executing in MATLAB
                thisMsg = obj.ConversionFcn(busstruct, obj.EmptySeedMsg);
                send(obj.pPublisher, thisMsg);
            elseif coder.target('Rtw')
                % The datatype of msg will be derived from the input to the block
                coder.ceval([obj.BlockId, '.publish'], ...
                            coder.rref(busstruct));
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
