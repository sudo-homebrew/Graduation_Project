classdef ros2subscriber < ros.ros2.internal.QOSUser & ...
        ros.internal.mixin.InternalAccess & ...
        robotics.core.internal.mixin.Unsaveable & handle
%ros2subscriber Subscribe to messages on a topic
%   Use the ros2subscriber object to receive messages on a topic. When
%   ROS 2 nodes publish messages on that topic, the subscriber receives the
%   messages.
%
%   SUB = ros2subscriber(NODE,"TOPIC") creates a subscriber, SUB, for a
%   topic with name TOPIC that already exists on the ROS 2 network topic
%   list. NODE is the ros2node object to which this subscriber attaches.
%   The subscriber gets the topic message type from the network topic list.
%   If the topic is not on the network topic list, this function displays
%   an error message.
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
%   Subscriber properties:
%      TopicName     - (Read-only) Name of the subscribed topic
%      MessageType   - (Read-only) Message type of subscribed messages
%      LatestMessage - (Read-only) Latest message that was received
%      NewMessageFcn - Callback function for processing new messages
%      History       - (Read-only) Message queue mode
%      Depth         - (Read-only) Message queue size
%      Reliability   - (Read-Only) Required delivery guarantee of messages
%      Durability    - (Read-Only) Required persistence of messages
%
%   Subscriber methods:
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

%   Copyright 2019-2021 The MathWorks, Inc.

    properties (Dependent, SetAccess = private)
        %TopicName - The name of the subscribed topic
        TopicName

        %LatestMessage - The most recent message that was received
        %   This does not depend on setting a callback function.
        %   Using the latest message can be more efficient than using
        %   callbacks if MATLAB has many subscribers or messages arrive at
        %   a high rate.
        LatestMessage
    end

    properties (SetAccess = private)
        %MessageType - The message type of subscribed messages
        MessageType
    end

    properties
        %NewMessageFcn - Callback property for subscriber callbacks
        NewMessageFcn
    end

    properties (Dependent, Access = ?ros.internal.mixin.InternalAccess)
        %MessageCount - Number of messages received
        %   Required to determine when new message is received
        MessageCount
    end

    properties (Transient, Access = ?ros.internal.mixin.InternalAccess)
        %NewMessageCallbackHandler - Helper to handle callbacks
        NewMessageCallbackHandler = []

        %NewMessageFunctionHandle - Actual function to call
        %   Function handle or string
        %   User-provided additional arguments held separately
        ActualNewMessageFcn = function_handle.empty

        %NewMessageCallbackArgs - User-provided additional arguments
        %   Cell array
        NewMessageCallbackArgs = {}

        %InternalNode - Internal representation of the node object
        %   Node required to get subscriber property information
        InternalNode = []

        %ServerNodeHandle - Designation of the node on the server
        %   Node handle required to get subscriber property information
        ServerNodeHandle = []

        %ServerSubscriberHandle - Designation of the subscriber on the server
        %   Subscriber handle required to get property information
        ServerSubscriberHandle = []

        %SubscriberStarted - Indicates if subscriber started successfully
        %   or not.
        SubscriberStarted = false;

        %MessageInfo - includes other information for a given message
        MessageInfo = struct.empty

        %RequestGUID - Indicates if publisher GID is added to messages
        %   Global Unique Identifier tells which publisher message is from
        %   If requested, the GUID is provided to the new message function
        %   as the second input argument
        %       function subCallback(MSG, GUID, OPTIONAL_ARGS)
        RequestGUID

        %MaxConcurrentCallbacks - Number of callbacks allowed in queue
        %   This is separate from the History/Depth Quality of Service
        %   combination, as that limits the ROS processing queue.
        %   The concurrent callbacks limits the number of callbacks allowed
        %   on the main MATLAB thread, and is set to the recursion limit
        %   upon construction by default.
        MaxConcurrentCallbacks

        %EnableCallback - Indicates if callback is allowed for subscriber
        %   When true, a standard callback is created for all subscribers,
        %   which calls the user-specified callback if set. When false, no
        %   callback handler is created, and any added callback on
        %   construction or later will not be called.
        EnableCallback
    end

    properties (Constant, Access = private)
        %DefaultRequestGUID - Do not put GUID in messages by default
        DefaultRequestGUID = false

        %DefaultEnableCallback - Enable callback use by default
        DefaultEnableCallback = true

        %NumTopicRetries - Number of times to retry getting topic
        NumTopicRetries = 3

        %TopicRetryDelay - Time between retries to get topic
        TopicRetryDelay = 0.1
    end

    methods
        function obj = ros2subscriber(node, topic, varargin)
        %ros2subscriber Create a ROS 2 subscriber object
        %   Attach a new subscriber to the ROS 2 "node" object. The "topic"
        %   argument is required and specifies the topic on which this
        %   subscriber should listen. Please see the class documentation
        %   (help ros2subscriber) for more details.

            narginchk(2, inf)

            % Extract the callback manually to avoid confusion over whether
            % the third input is the type or callback
            % It will always be the third or fourth input, if set, and no
            % other input should be a cell array or function handle
            if nargin > 2 && ...
                    (isa(varargin{1}, 'function_handle') || ...
                     iscell(varargin{1}))
                % Syntax: SUB = ros2subscriber(NODE,"TOPIC",CB)
                cb = varargin{1};
                varargin(1) = [];
            elseif nargin > 3 && ...
                    (isa(varargin{2}, 'function_handle') || ...
                     iscell(varargin{2}))
                % Syntax: SUB = ros2subscriber(NODE,"TOPIC","TYPE",CB)
                cb = varargin{2};
                varargin(2) = [];
            else
                % No callback specified
                cb = [];
            end

            % Convert all string arguments to characters
            [topic, varargin{:}] = convertStringsToChars(topic, varargin{:});

            % Parse name-value pairs separately from other input
            [ordinalParser, paramParser] = getParsers(obj);
            nvPairsStart = ros.internal.Parsing.findNameValueIndex(...
                varargin, paramParser.Parameters);
            if isempty(nvPairsStart)
                % Additional arguments past type assume to be parameters
                % Callback has already been removed from inputs
                nvPairsStart = min(2, numel(varargin)+1);
            end
            parse(ordinalParser, node, topic, varargin{1:nvPairsStart-1});
            parse(paramParser, varargin{nvPairsStart:end})

            % Save the internal node information for later use
            obj.InternalNode = ordinalParser.Results.node.InternalNode;
            obj.ServerNodeHandle = ordinalParser.Results.node.ServerNodeHandle;
            obj.RequestGUID = paramParser.Results.RequestGUID;
            obj.MaxConcurrentCallbacks = ...
                paramParser.Results.MaxConcurrentCallbacks;
            obj.EnableCallback = paramParser.Results.EnableCallback;

            % Set up callback handler if required
            if paramParser.Results.EnableCallback
                obj.NewMessageCallbackHandler = ...
                    ros.internal.CallbackHandler(matlab.internal.WeakHandle(obj), ...
                                                 @processNewMessage);
                obj.NewMessageFcn = cb;
            end

            % Resolve the topic name based on the node
            resolvedTopic = resolveName(ordinalParser.Results.node, ...
                                        ordinalParser.Results.topic);

            % Determine message type for topic from network
            requestedType = ordinalParser.Results.type;
            try
                existingType = ...
                    ros.ros2.internal.NetworkIntrospection.getTypeFromTopicWithNode(...
                    ordinalParser.Results.node, ...
                    resolvedTopic);

                % Retry if no topic supplied as node simply may not have
                % received the information from the network yet
                retryCount = 0;
                while isempty(requestedType) && isempty(existingType) && ...
                        retryCount < obj.NumTopicRetries
                    pause(obj.TopicRetryDelay)
                    existingType = ...
                        ros.ros2.internal.NetworkIntrospection.getTypeFromTopicWithNode(...
                        ordinalParser.Results.node, ...
                        resolvedTopic);
                    retryCount = retryCount+1;
                end
            catch ex
                newEx = MException(message('ros:mlros2:subscriber:CreateError', ...
                                           resolvedTopic, ...
                                           requestedType));
                throw(newEx.addCause(ex));
            end

            % Determine message type for subscriber
            % Topic unlikely to but may have multiple valid types
            if isempty(requestedType)           % Select type from network
                if isempty(existingType)        % Topic not yet used
                    error(message('ros:mlros2:subscriber:TopicNameNotFound', ...
                                  resolvedTopic))
                else                            % Use message from topic
                    obj.MessageType = existingType{1};
                end
            else                                % User-specified type
                if isempty(existingType) || ... % Topic not yet used
                        ismember(requestedType, existingType)   % Matching type
                    obj.MessageType = requestedType;
                else                            % Conflicting type
                    error(message('ros:mlros2:subscriber:TopicTypeNoMatch', ...
                                  resolvedTopic, ...
                                  existingType{1}, ...
                                  requestedType))
                end
            end

            % Handle quality of service settings
            qosSettings = getQosSettings(obj, paramParser.Results);

            % Set info based on the message
            setupInfo(obj);

            % Create the subscriber
            createSubscriber(obj, ordinalParser.Results.node.ServerNodeHandle, ...
                             resolvedTopic, ...
                             obj.MessageType, ...
                             qosSettings)

            function [ordinalParser, paramParser] = getParsers(obj)
            % Set up separate parsers for parameters and other input

            % Set up ordered inputs
            % node and topic always needed, type and callback optional
                ordinalParser = inputParser;
                addRequired(ordinalParser, 'node', ...
                            @(x) validateattributes(x, ...
                                                    {'ros2node'}, ...
                                                    {'scalar'}, ...
                                                    'ros2subscriber', ...
                                                    'node'));
                addRequired(ordinalParser, 'topic', ...
                            @(x) validateattributes(x, ...
                                                    {'char', 'string'}, ...
                                                    {'scalartext', 'nonempty'}, ...
                                                    'ros2subscriber', ...
                                                    'topic'));
                addOptional(ordinalParser, 'type', '', ...
                            @(x) validateattributes(x, ...
                                                    {'char', 'string'}, ...
                                                    {'scalartext', 'nonempty'}, ...
                                                    'ros2subscriber', ...
                                                    'type'))
                % Skip the callback argument - it will be extracted and
                % validated manually

                % Set up name-value pairs
                paramParser = inputParser;
                paramParser = addQOSToParser(obj, paramParser, 'ros2subscriber');
                addParameter(paramParser, 'RequestGUID', obj.DefaultRequestGUID, ...
                             @(x) validateattributes(x, ...
                                                     {'logical'}, ...
                                                     {'scalar'}, ...
                                                     'ros2subscriber', ...
                                                     'RequestGUID'))
                addParameter(paramParser, 'MaxConcurrentCallbacks', ...
                             get(0, 'RecursionLimit'), ...
                             @(x) validateattributes(x, ...
                                                     {'numeric'}, ...
                                                     {'scalar', 'nonnegative', 'finite'}, ...
                                                     'ros2subscriber', ...
                                                     'MaxConcurrentCallbacks'))
                addParameter(paramParser, 'EnableCallback', ...
                             obj.DefaultEnableCallback, ...
                             @(x) validateattributes(x, ...
                                                     {'logical'}, ...
                                                     {'scalar'}, ...
                                                     'ros2subscriber', ...
                                                     'EnableCallback'))
            end
        end

        function delete(obj)
        %delete Shut down subscriber
        %   delete(SUB) shuts down the ROS 2 subscriber object SUB and
        %   removes it from the network

            obj.NewMessageFcn = []; % Immediately halt callbacks

            % Cannot tell server to remove the subscriber without valid
            % internal node and server handle value
            if ~isempty(obj.InternalNode) && ...
                    isvalid(obj.InternalNode) && ...
                    ~isempty(obj.ServerSubscriberHandle)
                % Try "stop" and "remove" separately in case destructor is
                % being called after incomplete construction
                try
                    if obj.SubscriberStarted
                        % If subscriber is started, stop it.
                        stopSubscribe(obj.InternalNode, ...
                                      obj.ServerSubscriberHandle);
                    end
                catch ex
                    % If subscriber never started, no issue not stopping it
                    warning(message('ros:mlros2:subscriber:ShutdownStopError', ...
                                    ex.message))
                end
                try
                    removeSubscriber(obj.InternalNode, ...
                                     obj.ServerSubscriberHandle);
                catch ex
                    warning(message('ros:mlros2:subscriber:ShutdownRemoveError', ...
                                    ex.message))
                end
                % No need to check replies - should error on failure
            end
            obj.InternalNode = [];
        end

        function [msg, status, statusText] = receive(obj, timeout)
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

        % Initialize status
            status = false;
            statusText = 'unknown';

            % Parse timeout property
            % By default this will block until data is received
            try
                % Track current number of messages received
                nMessages = obj.MessageCount;
                if nargin > 1
                    timeout = ...
                        robotics.internal.validation.validatePositiveNumericScalar(timeout, ...
                                                                                   'receive', ...
                                                                                   'timeout');
                else
                    timeout = Inf;
                end
            catch ex
                if nargout > 1
                    % Return a default empty ros2message object
                    msg = ros2message(obj);
                    % status and statusText already default to unknown error
                    return
                end
                rethrow(ex)
            end

            % Wait until message is received, or timeout occurs
            try
                util = ros.internal.Util.getInstance;
                util.waitUntilTrue(@() obj.MessageCount > nMessages, ...
                                   timeout);
                msg = obj.LatestMessage;
            catch ex
                if nargout <= 1
                    if strcmp(ex.identifier, 'ros:mlros:util:WaitTimeout')
                        error(message('ros:mlros2:subscriber:WaitTimeout'))
                    else
                        rethrow(ex)
                    end
                else
                    % Return a default empty ros2message object
                    msg = ros2message(obj);
                    % status already defaults to indicate error
                    statusText = 'timeout';
                    return
                end
            end

            status = true;
            statusText = 'success';
        end

        function msg = ros2message(obj)
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
        %   See also RECEIVE.

            msg = ros2message(obj.MessageType);
        end

        function set.NewMessageFcn(obj, cb)
        %set.NewMessageFcn Set the subscriber callback
        %   sub.NewMessageFcn = CB sets the callback function that should
        %   be invoked when a new message is received. Here, CB is either
        %   a scalar function handle or a cell array. You can pass
        %   additional parameters to the callback function by including
        %   both the function handle and the parameters as elements
        %   of a cell array and assign it to CB. If no callbacks should
        %   be executed, assign the empty matrix [] to CB.
        %
        %   Each callback function must have the
        %   following signature:
        %
        %      function functionName(MESSAGE,VARARGIN)
        %
        %   Example:
        %
        %      laserSub = ros2subscriber(node,"/scan","sensor_msgs/LaserScan");
        %      laserSub.NewMessageFcn = @function1;
        %
        %      % A subsequent assignment will override the previous value
        %      userData = "extra data";
        %      laserSub.NewMessageFcn = {@function2, userData};

        % Do not modify handler if callback is disabled for subscriber
            if ~isempty(obj.NewMessageCallbackHandler) %#ok<MCSUP>
                if isempty(cb)
                    obj.ActualNewMessageFcn = function_handle.empty; %#ok<MCSUP>
                    obj.NewMessageCallbackArgs = {}; %#ok<MCSUP>
                else
                    % Make sure this is a valid function specifier
                    [fcnHandle, userData] = ...
                        ros.internal.Parsing.validateFunctionHandle(cb);

                    % Set properties used when message is received
                    obj.ActualNewMessageFcn = fcnHandle; %#ok<MCSUP>
                    obj.NewMessageCallbackArgs = userData; %#ok<MCSUP>
                end
            end

            % Set object property if validated
            obj.NewMessageFcn = cb;
        end
    end

    % All dependent properties are read from the server
    methods
        function topic = get.TopicName(obj)
        %get.TopicName Getter for topic name

        % Allow errors to be thrown from getServerInfo
            subInfo = getServerInfo(obj);
            topic = subInfo.topic;
        end

        function latestMessage = get.LatestMessage(obj)
        %get.LatestMessage Getter for most recently-received message

        % Avoid validating properties to improve performance

        % Extract message from subscriber
            try
                latestMessage = ...
                    getLatestMessage(obj.InternalNode, ...
                                     obj.ServerSubscriberHandle);
            catch ex
                newEx = MException(message('ros:mlros2:subscriber:GetLatestMessageError'));
                throw(newEx.addCause(ex));
            end
        end

        function messageCount = get.MessageCount(obj)
        %get.MessageCount Getter for number of messages received

        % Avoid validating properties to improve performance

        % Extract message count from subscriber
            try
                messageCount = ...
                    getLatestMessageCount(obj.InternalNode, ...
                                          obj.ServerSubscriberHandle);
            catch ex
                newEx = MException(message('ros:mlros2:subscriber:GetMessageCountError'));
                throw(newEx.addCause(ex));
            end
        end
    end

    methods (Access = ?ros.internal.mixin.InternalAccess)
        function processNewMessage(obj, msg, guid)
        %processNewMessage Take action based on new message from subscriber
        %   If the GUID is requested by the subscriber, it will be provided
        %   to the user-supplied callback as the second input argument

        % Call the callback function if assigned
            if ~isempty(obj.ActualNewMessageFcn)
                if nargin <= 2
                    feval(obj.ActualNewMessageFcn, ...
                          msg, ...
                          obj.NewMessageCallbackArgs{:})
                elseif isfield(guid, 'publisher_gid')
                    feval(obj.ActualNewMessageFcn, ...
                          msg, ...
                          guid.publisher_gid, ...
                          obj.NewMessageCallbackArgs{:})
                end
            end
        end
    end

    methods (Access = private)
        function setupInfo(obj)
        %setupInfo Set info based on the type

            obj.MessageInfo = ros.internal.ros2.getMessageInfo(obj.MessageType);
            [obj.MessageInfo.cppFactoryClass , obj.MessageInfo.cppElementType] = ...
                ros.internal.ros2.getCPPFactoryClassAndType(obj.MessageType);
        end

        function createSubscriber(obj, serverNodeHandle, topic, type, additionalSettings)
        %createSubscriber Create subscriber on ROS 2 network

        % If enabled, set up to use callback handler method as callback
            if obj.EnableCallback
                callbackFcn = obj.NewMessageCallbackHandler.CallbackName;
            else
                callbackFcn = '';
            end

            % Create subscriber but do not begin receiving messages
            additionalSettings.cppElementType = obj.MessageInfo.cppElementType;
            dllPaths = ros.internal.utilities.getPathOfDependentDlls(obj.MessageType,'ros2');
            try
                returnCall = addSubscriber(obj.InternalNode, ...
                                           serverNodeHandle, ...
                                           obj.MessageInfo.path, ...
                                           topic, ...
                                           obj.MessageInfo.cppFactoryClass, ...
                                           callbackFcn, ...
                                           additionalSettings, ...
                                           dllPaths, ...
                                           obj.RequestGUID);
                if isempty(returnCall) || ~isstruct(returnCall)
                    error(message('ros:mlros2:node:InvalidReturnCallError'))
                elseif ~isfield(returnCall, 'handle') || ...
                        isempty(returnCall.handle)
                    error(message('ros:mlros2:node:InvalidReturnCallHandleError'))
                end
                obj.ServerSubscriberHandle = returnCall.handle;
            catch ex
                newEx = MException(message('ros:mlros2:subscriber:CreateError', ...
                                           topic, type));
                throw(newEx.addCause(ex));
            end

            % Start subscriber, receiving messages
            try
                startSubscribe(obj.InternalNode, ...
                               returnCall.handle, ...
                               obj.NewMessageCallbackHandler, ...
                               obj.MaxConcurrentCallbacks);
                % No need to check reply - should error on failure
                % If subscriber started without any issue or exception,
                % update SubscriberStarted status as true.
                obj.SubscriberStarted = true;
            catch ex
                % Checking if error is invalid topic name
                % Catching it and creating own error

                invalidTopicErrorText1 = ...
                    message('ros:mlros2:subscriber:InvalidTopicErrorText1');
                invalidTopicErrorText2 = ...
                    message('ros:mlros2:subscriber:InvalidTopicErrorText2');

                if contains(ex.message, invalidTopicErrorText1.string) || ...
                        contains(ex.message, invalidTopicErrorText2.string)
                    error(message('ros:mlros2:subscriber:InvalidTopicName', ...
                                  topic));
                end

                newEx = MException(message('ros:mlros2:subscriber:StartError', ...
                                           topic, type));
                throw(newEx.addCause(ex));
            end
        end
    end

    methods (Access = protected)
        function subInfo = getServerInfo(obj)
        %getServerInfo Get subscriber properties from node server

        % Ensure properties are valid
            if isempty(obj.InternalNode) || ~isvalid(obj.InternalNode)
                error(message('ros:mlros2:subscriber:InvalidInternalNodeError'))
            elseif isempty(obj.ServerNodeHandle) || ...
                    isempty(obj.ServerSubscriberHandle)
                error(message('ros:mlros2:subscriber:InvalidServerHandleError'))
            end

            % Extract node information
            try
                nodeInfo = nodeinfo(obj.InternalNode, ...
                                    obj.ServerNodeHandle, []);
            catch ex
                newEx = MException(message('ros:mlros2:subscriber:GetInfoError'));
                throw(newEx.addCause(ex));
            end
            subHandles = [nodeInfo.subs.handle];
            whichSub = obj.ServerSubscriberHandle == subHandles;
            if ~any(whichSub)
                % Must be the wrong handle(s) if this subscriber exists
                error(message('ros:mlros2:subscriber:InvalidServerHandleError'))
            elseif nnz(whichSub) > 1
                % Duplicate subscriber handles found, error on node side
                error(message('ros:mlros2:subscriber:DuplicateSubHandlesError'))
            end
            subInfo = nodeInfo.subs(whichSub);
        end
    end


    %----------------------------------------------------------------------
    % MATLAB Code-generation
    %----------------------------------------------------------------------
    methods (Static = true, Access = private)
        function name = matlabCodegenRedirect(~)
            name = 'ros.internal.codegen.ros2subscriber';
        end
    end
end
