classdef Subscriber < ros.slroscpp.internal.block.ROSPubSubBase & ...
        ros.internal.mixin.ROSInternalAccess
%Subscribe to a topic on a ROS network
%
%   H = ros.slroscpp.internal.block.Subscriber creates a system
%   object, H, that subscribes to a topic on a ROS network and
%   receives messages on that topic.
%
%   This system object is intended for use with the MATLAB System
%   block. In order to access the ROS functionality from MATLAB, see
%   ROSSUBSCRIBER.
%
%   See also ROSPUBLISHER 

%   Copyright 2019-2021 The MathWorks, Inc.

%#codegen

    properties(Nontunable)
        %InputStreamWSVar Receive message from variable in base workspace
        %  Used to redirect Subscriber to receive messages from a workspace
        %  variable instead of from ROS network (used for testing only).
        %  The workspace variable should be created like this:
        %
        %   workspaceVar =  ros.slros.internal.sim.MsgListInputStream(rostopic, msgtype);
        %   workspaceVar.setMessageList( {msg1, msg2, msg3} );
        %
        % This capability of using workspace variables is only used in
        % internal testing, and is not documented for Simulink users.
        InputStreamWsVar = ''
    end
    
    % The following should ideally not show up in the MATLAB System block
    % dialog. However, setting them as 'Hidden' will prevent them from
    % being accessible via set_param & get_param
    properties (Constant,Access=private)
        %MessageCatalogName - Name of this block used in message catalog
        MessageCatalogName = message("ros:slros:blockmask:SubscriberMaskTitle").getString
    end

    properties (Access=private, Transient)
        % InputStream - Handle to object that implements rossubscriber
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
        Converter = ros.slroscpp.internal.sim.ROSMsgToBusStructConverter.empty

        % ROSMaster - Handle to an object that encapsulates interaction
        % with the ROS master. It is initialized to indicate the class of
        % the object.
        ROSMaster = ros.slros.internal.sim.ROSMaster.empty
    end

    methods
        function obj = Subscriber(varargin)
        % Enable code to be generated even if this file is p-coded
            coder.allowpcode('plain');
            obj = obj@ros.slroscpp.internal.block.ROSPubSubBase(varargin{:});
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
            coder.internal.errorIf(true, 'ros:slros:sysobj:SubscriberSimStateNotSupported');
        end        
    end

    methods (Access = protected)
        function setupImpl(obj)
        % setupImpl is called when model is being initialized at the
        % start of a simulation
            if coder.target('MATLAB')
                % Executing in MATLAB interpreted mode
                modelHasPriorState = ros.slros.internal.sim.ModelStateManager.hasState(obj.ModelName);
                nodeRefCountIncremented = false;
                
                try
                    % Executing in MATLAB interpreted mode
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
                    obj.pSubscriber = ros.Subscriber(modelState.ROSNode, ...
                        obj.ROSTopic, ...
                        obj.ROSMessageType, ...
                        'EnableCallback', false, ...
                        'DataFormat', 'struct');
                    modelState.incrNodeRefCount();
                    nodeRefCountIncremented = true;
                    obj.Converter = ros.slroscpp.internal.sim.ROSMsgToBusStructConverter(...
                        obj.ROSMessageType, obj.ModelName);
                    obj.LastMessageCount = uint64(0);
                    obj.TruncateAction = obj.Converter.MsgInfoMap(obj.ROSMessageType).VarLenTruncateAction;
                    obj.EmptySeedBusStruct = obj.Converter.convert(rosmessage(obj.ROSMessageType,"DataFormat","struct"));
                    [emptyMsg,info]= ros.internal.getEmptyMessage(obj.ROSMessageType,'ros');
                    cachedMap = containers.Map();
                    refCachedMap = containers.Map();
                    % This map contains the values of empty message data
                    % which can be reused when required.
                    refCachedMapStoragePath = fullfile(pwd,'+bus_conv_fcns','+ros','+msgToBus','RefCachedMap.mat');
                    if isfile(refCachedMapStoragePath)
                        load(refCachedMapStoragePath, 'refCachedMap'); %#ok<EMLOAD> 
                    end
                    cachedMap(obj.ROSMessageType) = emptyMsg;
                    [pkgName,msgName] = fileparts(obj.ROSMessageType);
                    obj.ConversionFcn = generateStaticConversionFunctions(obj,emptyMsg,...
                        info,'ros','msgToBus',pkgName,msgName,cachedMap,refCachedMap,refCachedMapStoragePath);
                    %addpath(pwd);
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
                coder.internal.errorIf(true, 'ros:slros:sysobj:SubscriberRapidAccelNotSupported');
            elseif coder.target('Rtw')
                coder.cinclude(obj.HeaderFile);
                % Append 0 to obj.ROSTopic, since MATLAB doesn't
                % automatically zero-terminate strings in generated code
                zeroDelimTopic = [obj.ROSTopic 0]; % null-terminated topic name
                coder.ceval([obj.BlockId, '.createSubscriber'], ...
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
                % TODO: Depends on colcon project generation
                isNewData = coder.ceval([obj.BlockId, '.getLatestMessage'],...
                    coder.wref(msg));
            end

        end

        %%
        function releaseImpl(obj)
            if coder.target('MATLAB')
                st = ros.slros.internal.sim.ModelStateManager.getState(obj.ModelName);
                st.decrNodeRefCount();
                delete(obj.pSubscriber);
                obj.LastMessageCount = uint64(0);
                if  ~st.nodeHasReferrers()
                    ros.slros.internal.sim.ModelStateManager.clearState(obj.ModelName);
                end
            end
        end
    end


end
