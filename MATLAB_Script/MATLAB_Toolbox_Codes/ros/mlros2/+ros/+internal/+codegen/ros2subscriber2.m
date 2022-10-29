classdef ros2subscriber < ros.internal.mixin.InternalAccess & ...
        coder.ExternalDependency
%ros2subscriber Subscribe to messages on a topic
%   Use the ros2subscriber object to receive messages on a topic. When
%   ROS 2 nodes publish messages on that topic, the subscriber receives the
%   messages.
%
%   SUB = ros2subscriber(NODE,"TOPIC","TYPE") creates a subscriber for
%   a topic and adds that topic to the network topic list. If the topic
%   list already contains a matching topic, NODE is added to the list of
%   subscribers for that topic.
%   If TYPE differs from the message type for that topic on the network
%   topic list, the function displays an error. Use this syntax to avoid
%   errors when subscribing to a topic before another subscriber or
%   publisher for the topic is created.
%
%   SUB = ros2subscriber(NODE,"TOPIC",CB) specifies a callback function,
%   CB, and optional data, to run when the subscriber object handle
%   receives a topic message. Use this syntax if action needs to be taken
%   on every message without blocking code execution.
%
%   The subscriber callback function requires at least one input argument.
%   The first argument is the received message object, MSG.
%   The function header for the callback is as follows:
%
%      function subCallback(MSG)
%
%   You pass additional parameters to the callback function by including
%   both the callback function and the parameters as elements of a cell
%   array when setting the callback.
%
%   SUB = ros2subscriber(NODE,"TOPIC","TYPE",CB) specifies a callback
%   function CB, and subscribes to a topic that has the specified name
%   TOPIC and message type TYPE.
%
%   SUB = ros2subscriber(___,Name,Value) provides additional options
%   specified by one or more Name,Value pair arguments. You can specify
%   several name-value pair arguments in any order as
%   Name1,Value1,...,NameN,ValueN:
%
%      "History"     - Mode for storing messages in the queue. If the queue
%                      fills with messages waiting to be processed, then
%                      old messages are dropped to make room for new.
%                      Options are:
%                         "keeplast"       - Store up to the number of
%                                            messages set by "Depth".
%                         "keepall"        - Store all messages
%                                            (up to resource limits).
%      "Depth"       - Size of the message queue in number of messages.
%                      Only applies if "History" property is "keeplast".
%      "Reliability" - Requirement on method for ensuring message delivery.
%                      Options are:
%                         "reliable"       - Require guaranteed delivery,
%                                            which may need multiple
%                                            attempts to publish.
%                         "besteffort"     - Attempt delivery once.
%      "Durability"  - Requirement on storage of messages on the publisher.
%                      Late-joining subscribers will receive messages if
%                      they require persistence. Options are:
%                         "volatile"       - Messages do not need to
%                                            persist.
%                         "transientlocal" - Publisher must persist latest
%                                            messages.
%
%   NOTE: The "Reliability" and "Durability" quality of service settings
%   must be compatible between publishers and subscribers for a connection
%   to be made.
%
%   ros2subscriber properties:
%      TopicName     - (Read-only) Name of the subscribed topic
%      MessageType   - (Read-only) Message type of subscribed messages
%      LatestMessage - (Read-only) Latest message that was received
%      NewMessageFcn - Callback function for processing new messages
%      History       - (Read-only) Message queue mode
%      Depth         - (Read-only) Message queue size
%      Reliability   - (Read-Only) Required delivery guarantee of messages
%      Durability    - (Read-Only) Required persistence of messages
%
%   ros2subscriber methods:
%      receive     - Wait for new message
%      ros2message - Create an empty message based on the topic type
%
%   Example:
%
%      % Create a ROS 2 node
%      node = ros2node("/node_1");
%
%      % Create subscriber
%      laserSub = ros2subscriber(node,"/scan","sensor_msgs/LaserScan");
%
%      % Show latest message that was received (if any)
%      scan = laserSub.LatestMessage;
%
%      % Create subscriber with callback function
%      % The topic type is inferred (if topic /chatter exists)
%      chatPub = ros2publisher(node,"/chatter","std_msgs/String");
%      chatSub = ros2subscriber(node,"/chatter",@testCallback);
%
%      % Create subscriber with message queue depth of 5
%      subQueue = ros2subscriber(node,"/chatter","Depth",5);

%#codegen
%   Copyright 2021-2022 The MathWorks, Inc.

    properties (Dependent, SetAccess = private)
        %LatestMessage - The most recent message that was received
        %   This does not depend on setting a callback function.
        %   Using the latest message can be more efficient than using
        %   callbacks if MATLAB has many subscribers or messages arrive at
        %   a high rate.
        LatestMessage
    end

    properties (SetAccess = immutable)
        %TopicName - The name of the subscribed topic
        TopicName

        %MessageType - The message type of subscribed messages
        MessageType

        %NewMessageFcn - Callback property for subscriber callbacks
        NewMessageFcn

        %History - The message queue mode
        History

        %Depth - The message queue size
        Depth

        %Reliability - The delivery guarantee of messages
        Reliability

        %Durability - The persistence of messages
        Durability
    end

    properties (SetAccess = private)
        %MessageCount - Number of messages received
        %   Required to determine when new message is received
        MessageCount
    end

    properties (Access = private)
        SubscriberHelper
        MsgStruct
        Mutex
        % Args - Function arguments for NewMessageFcn
        Args
        IsInitialized = false
    end

    methods
        function obj = ros2subscriber(node, topic, varargin)
        %ros2subscriber Create a ROS 2 subscriber object
        %   Attach a new subscriber to the ROS 2 "node" object. The "topic"
        %   argument is required and specifies the topic on which this
        %   subscriber should listen. Please see the class documentation
        %   (help ros2subscriber) for more details.

            coder.inline('never');
            narginchk(2, inf);
            coder.extrinsic('ros.codertarget.internal.getCodegenInfo');
            coder.extrinsic('ros.codertarget.internal.ROSMATLABCgenInfo');
            coder.extrinsic('ros.codertarget.internal.ROSMATLABCgenInfo.getInstance');
            coder.extrinsic('ros.codertarget.internal.getEmptyCodegenMsg');

            % Ensure varargin is not empty
            coder.internal.assert(nargin>2,'ros:mlros2:codegen:MissingMessageType',topic,'ros2subscriber');

            % Specialize ros2subscriber class based on messageType
            coder.internal.prefer_const(varargin{1});

            %% Check input arguments
            % Validate input ros2node
            validateattributes(node, {'ros2node'}, {'scalar'}, ...
                               'ros2subscriber','node');
            % Message type must be specified for codegen. It is always the
            % first argument passed after topic (varargin{1})
            topic = convertStringsToChars(topic);
            validateattributes(topic,{'char','string'},{'nonempty'}, ...
                               'ros2subscriber','topic');
            messageType = convertStringsToChars(varargin{1});
            validateattributes(messageType,{'char','string'},{'nonempty'}, ...
                               'ros2subscriber','messageType');

            % Ensure that the message type has been specified
            coder.internal.assert(contains(messageType,'/'),'ros:mlros2:codegen:MissingMessageType',topic,'ros2subscriber');

            % Extract callback function if specified
            indx = 2;
            if nargin > 3
                % laserSub = ros2subscriber(node,"/scan","sensor_msgs/LaserScan");
                % laserSub.NewMessageFcn = @function1;
                %
                % % A subsequent assignment will override the previous value
                % userData = "extra data";
                % laserSub.NewMessageFcn = {@function2, userData};
                if isa(varargin{2},'function_handle')
                    obj.NewMessageFcn = varargin{2};
                    indx = indx + 1;
                elseif iscellstr(varargin{2})
                    cb = varargin{2};
                    obj.NewMessageFcn = cb{1};
                    obj.Args = cb(2:end);
                    indx = indx + 1;
                end
            end

            % Parse NV pairs
            nvPairs = struct('History',uint32(0),...
                             'Depth',uint32(0),...
                             'Reliability',uint32(0),...
                             'Durability',uint32(0));
            pOpts = struct('PartialMatching',true,'CaseSensitivity',false);
            pStruct = coder.internal.parseParameterInputs(nvPairs,pOpts,varargin{indx:end});

            qosHistory = coder.internal.getParameterValue(pStruct.History,'keeplast',varargin{indx:end});
            validateStringParameter(qosHistory,{'keeplast', 'keepall'},'ros2subscriber','History');

            qosDepth = coder.internal.getParameterValue(pStruct.Depth,1,varargin{indx:end});
            validateattributes(qosDepth,{'numeric'},...
                               {'scalar','nonempty','integer','nonnegative'},...
                               'ros2subscriber','Depth');

            qosReliability = coder.internal.getParameterValue(pStruct.Reliability,'reliable',varargin{indx:end});
            validateStringParameter(qosReliability,{'reliable', 'besteffort'},'ros2subscriber','Reliability');

            qosDurability = coder.internal.getParameterValue(pStruct.Durability,'volatile',varargin{indx:end});
            validateStringParameter(qosDurability,{'transientlocal', 'volatile'},'ros2subscriber','Durability');

            % Resolve the topic name based on the node
            resolvedTopic = resolveName(node, topic);

            % Store inputs
            obj.TopicName = resolvedTopic;
            obj.MessageType = messageType;
            obj.History = convertStringsToChars(qosHistory);
            obj.Depth = qosDepth;
            obj.Reliability = convertStringsToChars(qosReliability);
            obj.Durability = convertStringsToChars(qosDurability);

            qos_profile = coder.opaque('rmw_qos_profile_t', ...
                                       'rmw_qos_profile_default', 'HeaderFile', 'rmw/qos_profiles.h');
            qos_profile = ros.ros2.internal.setQOSProfile(qos_profile, obj.History, obj.Depth, ...
                                                          obj.Reliability, obj.Durability);

            obj.MessageCount = 0; % Incremented to zero by obj.callback() below

            % Get and register code generation information
            cgInfo = coder.const(@ros.codertarget.internal.getCodegenInfo,topic,messageType,'sub','ros2');
            msgStructGenFcn = str2func(cgInfo.MsgStructGen);
            obj.MsgStruct = msgStructGenFcn();  % Setup return type

            %% Create an instance of MATLABROS2Subscriber object
            % template <class MsgType, class StructType>
            %MATLABROS2Subscriber(const StructType* structPtr, std::function<void(void)> callback)
            templateTypeStr = ['MATLABROS2Subscriber<' cgInfo.MsgClass ',' cgInfo.MsgStructGen '_T>'];
            coder.ceval('auto structPtr = ', coder.wref(obj.MsgStruct));
            obj.SubscriberHelper = coder.opaque(['std::unique_ptr<', templateTypeStr, '>'],...
                                                'HeaderFile', 'mlros2_sub.h');
            if ros.internal.codegen.isCppPreserveClasses
                % Create subscriber by passing in class method as callback
                obj.SubscriberHelper = coder.ceval(['std::unique_ptr<', templateTypeStr, ...
                                                    '>(new ', templateTypeStr, '(structPtr, [this]{this->callback();}));//']);
            else
                % Create subscriber by passing in static function as
                % callback
                obj.SubscriberHelper = coder.ceval(['std::unique_ptr<', templateTypeStr, ...
                                                    '>(new ', templateTypeStr, '(structPtr, [obj]{ros2subscriber_callback(obj);}));//']);
            end
            coder.ceval('MATLABROS2Subscriber_createSubscriber',...
                        obj.SubscriberHelper, node.NodeHandle, coder.rref(obj.TopicName),...
                        size(obj.TopicName,2),qos_profile);

            % Ensure callback is not optimized away by making an explicit
            % call here
            obj.callback();
            obj.IsInitialized = true;
        end

        %% Called from MATLABROS2Subscriber class
        function callback(obj)
            coder.inline('never') % This functions is called from MATLABROS2Subscriber and cannot be inlined
            obj.MessageCount = obj.MessageCount + 1;
            if ~isempty(obj.NewMessageFcn) && obj.IsInitialized
                % Call user defined callback function
                if isempty(obj.Args)
                    obj.NewMessageFcn(obj,obj.MsgStruct);
                else
                    obj.NewMessageFcn(obj,obj.MsgStruct,obj.Args{:});
                end
            end
        end

        function messageCount = get.MessageCount(obj)
        % Function is needed to prevent C++ compiler optimizing while
        % loop in receive call.
            messageCount = obj.MessageCount;
        end

        function [receivedMsg, status, statusText] = receive(obj, timeout)
        %receive Wait for new message
        %   MSG = receive(SUB) waits until a new message is received by the
        %   subscriber, SUB, for the specific topic.
        %
        %   MSG = receive(SUB,TIMEOUT) specifies a TIMEOUT period, in
        %   seconds. If the subscriber does not receive a topic message and
        %   the timeout period elapses, the function displays an error
        %   message.
        %
        %   [MSG, STATUS, STATUSTEXT] = receive(____) returns the final receive status
        %   and the associated status text using any of the previous syntaxes.
        %   The STATUS indicates if the message has been received successfully or not and
        %   the associated STATUSTEXT will capture information about the status.
        %   The STATUSTEXT can be one of the following:
        %
        %       'success' - The message was successfully received
        %       'timeout' - The message was not received within
        %                   the specified timeout.
        %       'unknown' - The message was not received due to
        %                   unknown errors.
        %
        %   Choosing between receive and using a callback:
        %   - Use receive when your program should wait until the next
        %     message is received on the topic and no other processing
        %     should happen in the meantime.
        %   - If you want your program to keep running and be notified
        %     whenever a new message arrives, consider using a callback
        %     instead of receive.
        %
        %
        %   Example:
        %
        %      % Create subscriber and receive data (blocking)
        %      laser = ros2subscriber(node,"/scan","sensor_msgs/LaserScan");
        %      scan = receive(laser);
        %
        %      % Receive data with 2 second timeout
        %      scan = receive(laser,2)

        % Track current number of messages received
            coder.inline('never');
            nMessages = obj.MessageCount;
            statusText = 'unknown';

            % Warning if no status output
            if nargout<2
                coder.internal.compileWarning('ros:mlros2:codegen:MissingStatusOutput','receive');
            end

            if (nargin > 1) && isfinite(timeout)
                validateattributes(timeout,{'double','scalar'},{'nonempty','nonnan','nonnegative','finite'},...
                                   'receive','timeout');

                coder.cinclude("mlros2_sub.h");
                tTimeoutNsec = coder.opaque("rcl_duration_value_t","HeaderFile","rclcpp/rclcpp.hpp");
                tTimeoutNsec = coder.ceval("RCL_S_TO_NS",timeout);
                tClock = coder.opaque("::rclcpp::Clock::SharedPtr","HeaderFile","rclcpp/rclcpp.hpp");
                tStop = coder.opaque("::rclcpp::Time","HeaderFile","rclcpp/rclcpp.hpp");
                tClock = coder.ceval("std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);//");
                tStop = coder.ceval("tClock->now() + rclcpp::Duration",tTimeoutNsec);
                status = true;
                while (obj.MessageCount == nMessages)
                    %Initialize variable currentTime as required by coder
                    currentTime = tStop;
                    currentTime = coder.ceval('tClock->now();//',tClock);
                    if currentTime >= tStop
                        status = false;
                        break;
                    end
                end
            else
                % timeout == inf
                while obj.MessageCount == nMessages
                    % Avoid being optimized away
                    coder.ceval("std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);//");
                end
                status = true;
            end

            coder.ceval('getStatusText',status,coder.wref(statusText));
            statusIndicator = status;
            if ~statusIndicator && nargout<2
                coder.internal.error('ros:mlros2:subscriber:WaitTimeout');
            end
            receivedMsg = obj.LatestMessage;
        end

        function msgFromSub = ros2message(obj)
        % ROS2MESSAGE Create an empty message structure based on the topic type
        %   MSG = ROS2MESSAGE(SUB) creates and returns an empty message
        %   structure MSG. The message type of MSG is determined by the
        %   topic this subscriber SUB is connected to.
        %
        %   Example:
        %
        %      % Create a ROS 2 node
        %      node = ros2node("/node_1");
        %
        %      % Create subscriber and message
        %      laserSub = ros2subscriber(node,"/scan","sensor_msgs/LaserScan");
        %      msg = ros2message(laserSub);
        %
        %   See also RECEIVE

            msgFromSub = ros2message(obj.MessageType);
        end

        function lastSubMsg = get.LatestMessage(obj)
        % MsgStruct is updated by MATLABROS2Subscriber whenever a new
        % message is received. We need to synchronize access here so
        % that MsgStruct is not read while being written by
        % MATLABROS2Subscriber as part of msg2struct call.
            coder.ceval('MATLABROS2Subscriber_lock',obj.SubscriberHelper)
            lastSubMsg = obj.MsgStruct;
            coder.ceval('MATLABROS2Subscriber_unlock',obj.SubscriberHelper)
        end
    end

    methods (Access = protected)
        function subInfo = getServerInfo(~)  %#ok<STOUT>
        %getServerInfo Get subscriber properties from node server
            if ~isempty(coder.target)
                coder.internal.assert(false,...
                                      'ros:mlros2:codegen:UnsupportedMethodCodegen','getServerInfo');
            end
        end
    end

    methods (Static)
        function props = matlabCodegenNontunableProperties(~)
            props = {'MessageType'};
        end

        function ret = getDescriptiveName(~)
            ret = 'ROS 2 Subscriber';
        end

        function ret = isSupportedContext(bldCtx)
            ret = bldCtx.isCodeGenTarget('rtw');
        end

        function updateBuildInfo(buildInfo,bldCtx)
            if bldCtx.isCodeGenTarget('rtw')
                srcFolder = fullfile(toolboxdir('ros'),'codertarget','src');
                addIncludeFiles(buildInfo,'mlros2_sub.h',srcFolder);
                addIncludeFiles(buildInfo,'mlros2_qos.h',srcFolder);
                addSourceFiles(buildInfo,'mlros2_sub.cpp',srcFolder);
            end
        end
    end

    methods (Static, Access = ?ros.internal.mixin.InternalAccess)
        function props = getImmutableProps()
            props = {'TopicName','MessageType','BufferSize',...
                     'NewMessageFcn','DataFormat'};
        end
    end
end

function validateStringParameter(value, options, funcName, varName)
% Separate function to suppress output and just validate
    validatestring(value, options, funcName, varName);
end
