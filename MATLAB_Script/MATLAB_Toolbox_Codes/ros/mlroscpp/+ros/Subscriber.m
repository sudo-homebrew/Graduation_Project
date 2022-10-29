classdef Subscriber < ros.internal.mixin.ROSInternalAccess & ...
        ros.internal.DataFormatBase & ...
        robotics.core.internal.mixin.Unsaveable & handle
%Subscriber Subscribe to messages on a topic
%   The primary tool for transferring data in ROS is publishing and
%   subscribing to messages. Messages are sent through topics and
%   ros.Subscriber allows you to subscribe to these topics.
%   When ROS nodes publish messages on that topic, MATLAB will receive
%   those message through this subscriber.
%
%   SUB = ros.Subscriber(NODE,TOPIC) subscribes to a topic with name
%   TOPIC. NODE is the ros.Node object handle that this subscriber
%   should attach to.
%   If the ROS master topic list includes TOPIC, this syntax returns a
%   subscriber object handle, SUB. If the ROS master topic list does not
%   include the topic, this syntax displays an error. TOPIC is a string scalar.
%
%   SUB = ros.Subscriber(NODE,TOPIC,TYPE) subscribes to a topic
%   that has the specified name TOPIC and message type TYPE. This syntax
%   returns a subscriber object handle. If the topic list on ROS master
%   does not include a topic with the specified name and type, a topic
%   with the specific name and type is added to the topic list.
%   Use this syntax to avoid errors when it is possible for the subscriber
%   to subscribe to a topic before a publisher has added the topic to
%   the topic list on the ROS master. TYPE is a string scalar.
%
%   SUB = ros.Subscriber(NODE,TOPIC,CB) specifies a callback function CB,
%   and optional data, to run when the subscriber object handle receives
%   a topic message. Use this syntax to avoid blocking wait functions.
%   CB can be a single function handle or a cell array. The
%   first element of the cell array needs to be a function handle or a
%   string containing the name of a function. The remaining elements of
%   the cell array can be arbitrary user data that will be passed to
%   the callback function.
%
%   SUB = ros.Subscriber(NODE,TOPIC,TYPE,CB)  specifies a
%   callback function CB, and subscribes to a topic that has the
%   specified name TOPIC and message type TYPE.
%
%   SUB = ros.Subscriber(___,Name,Value) provides additional options
%   specified by one or more Name,Value pair arguments. You can specify
%   several name-value pair arguments in any order as
%   Name1,Value1,...,NameN,ValueN:
%
%      "BufferSize" - specifies the size of the queue for incoming
%                     messages. If messages are arriving faster and
%                     your callback is unable to process them, they
%                     will be thrown away once the incoming queue
%                     is full.
%                     Default: 1
%
%      "DataFormat" - Determines format of ROS message provided by
%                     LatestMessage and receive, and to the
%                     NewMessageFcn callback.
%                     Using structs can be faster than using message
%                     objects.
%                     Options are:
%                        "object" - Message object of the specified type
%                        "struct" - Message struct with compatible fields
%                     Default: "object"
%
%   The subscriber callback function requires at least two input
%   arguments. The first argument SRC is the associated subscriber object.
%   The second argument is the received message object MSG. The function
%   header for the callback is as follows:
%
%      function subCallback(SRC,MSG)
%
%   You pass additional parameters to the callback function by including
%   both the callback function and the parameters as elements of a cell array
%   when setting the callback.
%
%
%   Subscriber properties:
%      TopicName       - (Read-only) The name of the subscribed topic
%      MessageType     - (Read-only) The message type of subscribed messages
%      DataFormat      - (Read-Only) Message format provided by subscriber
%      LatestMessage   - (Read-only) The latest message that was received
%      BufferSize      - (Read-only) The buffer size of the incoming queue
%      NewMessageFcn   - Callback property for subscriber callbacks
%
%   Subscriber methods:
%      receive     - Block until data is received and return new message
%      rosmessage  - Create an empty message based on the message type
%
%
%   Example:
%
%      % Create the ROS master and a node
%      master = ros.Core;
%      node = ros.Node("/test1");
%
%      % Create subscriber for lidar data
%      % Use struct message format for better performance
%      laserSub = ros.Subscriber(node,"/scan","sensor_msgs/LaserScan",...
%          "DataFormat","struct");
%
%      % Get latest message that was received (if any)
%      scanMsg = laserSub.LatestMessage;
%
%      % Wait for next message to arrive (blocking)
%      scanMsg = receive(laserSub);
%
%      % Create subscriber with callback function
%      % Have the publisher and subscriber use message objects
%      % The topic type is inferred (if topic /chatter exists)
%      chatPub = ros.Publisher(node,"/chatter","std_msgs/String",...
%          "DataFormat","object");
%      chatSub = ros.Subscriber(node,"/chatter",@testCallback,...
%          "DataFormat","object");
%
%      % Create subscriber with buffer size of 5
%      bufferSub = ros.Subscriber(node,"/chatter","BufferSize",5);
%
%   See also ROSSUBSCRIBER, ROSMESSAGE, ROSTOPIC.

%   Copyright 2014-2021 The MathWorks, Inc.

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

        %BufferSize - The buffer size of the incoming queue
        BufferSize = []
    end

    properties
        %NewMessageFcn - Callback property for subscriber callbacks
        NewMessageFcn = function_handle.empty
    end

    properties (Dependent, Access = ?ros.internal.mixin.ROSInternalAccess)
        %MessageCount - Number of messages received
        %   Required to determine when new message is received
        MessageCount
    end

    properties (Transient, Access = ?ros.internal.mixin.ROSInternalAccess)
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

        %MessageInfo - includes other information for a given message
        MessageInfo = struct.empty

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
        %DefaultEnableCallback - Enable callback use by default
        DefaultBufferSize = 1

        %DefaultEnableCallback - Enable callback use by default
        DefaultEnableCallback = true

        %NumTopicRetries - Number of times to retry getting topic
        NumTopicRetries = 3

        %TopicRetryDelay - Time between retries to get topic
        TopicRetryDelay = 0.1
    end

    methods
        function obj = Subscriber(node, topic, varargin)
        %Subscriber - Create a ROS subscriber object
        %   Attach a new subscriber to the ROS node object. The topic
        %   argument is required and specifies the topic on which this
        %   subscriber should receive messages. Please see the class documentation
        %   (help ros.Subscriber) for more details.

            if isempty(node)
                node = ros.internal.Global.getNodeHandle(false);
            end

            % Extract the callback manually to avoid confusion over whether
            % the third input is the type or callback
            % It will always be the third or fourth input, if set, and no
            % other input should be a cell array or function handle
            if nargin > 2 && ...
                    (isa(varargin{1}, 'function_handle') || iscell(varargin{1}))
                % Syntax: SUB = ros.Subscriber(NODE,"TOPIC",CB)
                cb = varargin{1};
                varargin(1) = [];

            elseif nargin > 3 && ...
                    (isa(varargin{2}, 'function_handle') || ...
                     iscell(varargin{2}))
                % Syntax: SUB = ros.subscriber(NODE,"TOPIC","TYPE",CB)
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

            try
                parse(ordinalParser, node, topic, varargin{1:nvPairsStart-1});
            catch except
                if contains(except.message, 'ros.Node')
                    error(message('ros:mlros:subscriber:NoNodeInput',class(node)));
                else
                    throw(except);
                end
            end

            parse(paramParser, varargin{nvPairsStart:end});

            % Resolve the topic name based on the node
            resolvedTopic = resolveName(ordinalParser.Results.node, ...
                                        ordinalParser.Results.topic);

            % Determine message type for topic from network
            requestedType = ordinalParser.Results.type;

            % Make sure that message types agree if topic is already
            % published
            % Need to check for existing type at least once, continue to
            % retry if no topic supplied as node simply may not have
            % received the information from the network yet
            masterType = [];
            retryCount = 0;
            while (isempty(requestedType) && retryCount < 1) || ...
                    (isempty(masterType) && retryCount < obj.NumTopicRetries)
                try
                    masterType = ros.internal.NetworkIntrospection.getPublishedTopicType(...
                        resolvedTopic, false, ordinalParser.Results.node.MasterURI);
                catch except
                    if ~strcmp(except.identifier, 'ros:mlros:topic:TopicNameNotFound')
                        rethrow(except);
                    elseif isempty(requestedType)
                        % Only pause if retrying
                        pause(obj.TopicRetryDelay)
                    end
                end
                retryCount = retryCount+1;
            end

            % If topic is not published determine message type
            if isempty(masterType)      % No existing topic on network
                if isempty(requestedType)
                    error(message('ros:mlros:topic:CannotDetermineType', resolvedTopic));
                else                    % Use user-requested type
                    masterType = requestedType;
                end
            else                        % Existing topic on network
                if ~isempty(requestedType) && ...
                        ~strcmp(masterType, requestedType)
                    error(message('ros:mlros:topic:TopicTypeNoMatch', ...
                                  resolvedTopic, masterType, masterType, requestedType));
                end
            end

            % Save the internal node information for later use
            obj.InternalNode = ordinalParser.Results.node.InternalNode;
            obj.ServerNodeHandle = ordinalParser.Results.node.ServerNodeHandle;
            obj.BufferSize = paramParser.Results.BufferSize;
            obj.MaxConcurrentCallbacks = ...
                paramParser.Results.MaxConcurrentCallbacks;
            obj.EnableCallback = paramParser.Results.EnableCallback;
            obj.MessageType = masterType;
            setDataFormat(obj, paramParser.Results.DataFormat)
            node.ListofNodeDependentHandles{end+1} = matlab.internal.WeakHandle(obj);

            % Set info based on the message
            setupInfo(obj);

            % Set up callback handler if required
            if obj.EnableCallback
                obj.NewMessageCallbackHandler = ...
                    ros.internal.CallbackHandler(matlab.internal.WeakHandle(obj), @processNewMessage);
                obj.NewMessageFcn = cb;
            end

            % Create the subscriber
            bufferSizeSettingsstruct = ...
                struct('buffersize', uint32(obj.BufferSize));
            createSubscriber(obj, obj.ServerNodeHandle, ...
                             resolvedTopic, ...
                             obj.MessageType, ...
                             bufferSizeSettingsstruct)
        end

        function delete(obj)
        %delete Shut down subscriber
        %   delete(SUB) shuts down the ROS subscriber object SUB and
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
                    stopSubscribe(obj.InternalNode, ...
                                  obj.ServerSubscriberHandle);
                catch ex
                    % If subscriber never started, no issue not stopping it
                    warning(message('ros:mlroscpp:subscriber:ShutdownStopError', ...
                                    ex.message))
                end
                try
                    removeSubscriber(obj.InternalNode, ...
                                     obj.ServerSubscriberHandle);
                catch ex
                    warning(message('ros:mlroscpp:subscriber:ShutdownRemoveError', ...
                                    ex.message))
                end
                % No need to check replies - should error on failure
            end
            obj.InternalNode = [];
        end

        function [msg, status, statusText] = receive(obj, timeout)
        %RECEIVE Block until data is received
        %   MSG = RECEIVE(SUB) blocks MATLAB from running the current program
        %   until the specified subscriber SUB receives a topic message
        %   and returns it in MSG. Press Ctrl+C to abort the wait. The
        %   format of MSG will be determined by the DataFormat of SUB.
        %
        %   MSG = RECEIVE(SUB,TIMEOUT) specifies a TIMEOUT period, in
        %   seconds. If the subscriber does not receive a topic message
        %   and the timeout period elapses, RECEIVE displays an error
        %   message and lets MATLAB continue running the current program.
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
        %   Use RECEIVE to block MATLAB from running the current program
        %   until the subscriber receives a topic message. When the
        %   subscriber receives a message, RECEIVE lets MATLAB continue
        %   running the current program.
        %   To unblock the current RECEIVE and let MATLAB continue running
        %   the program, press Ctrl+C.
        %
        %   Choosing between RECEIVE and using a callback argument:
        %   - Use RECEIVE when your program should wait until the next
        %     message is received on the topic and no other processing
        %     should happen in the meantime.
        %   - If you want your program to keep running and be notified
        %     whenever a new message arrives, consider using
        %     the callback argument, CB, instead of RECEIVE.
        %
        %
        %   Example:
        %
        %      % Create subscriber and receive data (blocking)
        %      laserSub = ros.Subscriber(node,"/scan","sensor_msgs/LaserScan");
        %      scanMsg = RECEIVE(laserSub);
        %
        %      % Receive data with 2 second timeout
        %      scanMsg = RECEIVE(laserSub,2)

        % Initialize status
            status = false;
            statusText = 'unknown';

            % Parse timeout property
            % By default this will block until data is received
            try
                % Track current number of messages received
                nMessages = obj.MessageCount;
                if nargin > 1
                    if ~isequal(timeout,Inf)
                        timeout = ...
                            robotics.internal.validation.validatePositiveNumericScalar(timeout, ...
                                                                                       'receive', ...
                                                                                       'timeout');
                    end
                else
                    timeout = Inf;
                end
            catch ex
                if nargout > 1
                    % Return a default empty rosmessage object
                    msg = rosmessage(obj);
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
                        error(message('ros:mlros:subscriber:WaitTimeout'))
                    else
                        rethrow(ex)
                    end
                else
                    % Return a default empty rosmessage object
                    msg = rosmessage(obj);
                    % status already defaults to indicate error
                    statusText = 'timeout';
                    return
                end
            end

            status = true;
            statusText = 'success';
        end

        function msg = rosmessage(obj, varargin)
        % ROSMESSAGE Create an empty message structure based on the topic type
        %   MSG = ROSMESSAGE(SUB) creates and returns an empty message MSG.
        %   The message type of MSG is determined by the topic this
        %   subscriber SUB is connected to. The format of MSG is determined
        %   by the DataFormat of the subscriber.
        %
        %   Example:
        %      % Create subscriber and message
        %      laserSub = ros.Subscriber(node,"/scan","sensor_msgs/LaserScan");
        %      msgObj = ROSMESSAGE(laserSub);
        %
        %      % Improve performance by using struct messages
        %      poseSub = ros.Subscriber(node,"/pose","geometry_msgs/Pose2D","DataFormat","struct");
        %      msgStruct = ROSMESSAGE(poseSub);
        %
        %   See also RECEIVE.

            validateDataFormatROSMessage(obj, varargin{:})

            msg = rosmessage(obj.MessageType, 'DataFormat', obj.DataFormat);
        end

        function set.NewMessageFcn(obj, cb)
        %set.NewMessageFcn Set the subscriber callback
        %
        %   sub.NewMessageFcn = CB sets the callback function that should
        %   be invoked when a new message is received. Here, CB is
        %   either a scalar function handle or a cell array. You can
        %   pass additional parameters to the callback function by including
        %   both the function handle and the parameters as elements
        %   of a cell array and assign it to CB. If no callbacks should
        %   be executed, assign the empty matrix [] to CB.
        %
        %   Each callback function must have the
        %   following signature:
        %
        %      function functionName(SRC,MSG,VARARGIN)
        %
        %   Example:
        %
        %      laserSub = Subscriber(node,"/scan","sensor_msgs/LaserScan");
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
                latestMessage = getLatestMessage(obj.InternalNode, ...
                                                 obj.ServerSubscriberHandle);
                if obj.UseObjectMsg
                    latestMessage = feval(obj.MessageInfo.msgClassGen, ...
                                          latestMessage);
                end
            catch ex
                newEx = MException(message('ros:mlroscpp:subscriber:GetLatestMessageError'));
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
                newEx = MException(message('ros:mlroscpp:subscriber:GetMessageCountError'));
                throw(newEx.addCause(ex));
            end
        end
    end

    methods (Access = ?ros.internal.mixin.ROSInternalAccess)
        function processNewMessage(obj, msg)
        %processNewMessage Take action based on new message from subscriber
        %   If the GUID is requested by the subscriber, it will be provided
        %   to the user-supplied callback as the second input argument

        % Call the callback function if assigned
            if ~isempty(obj.ActualNewMessageFcn)
                if obj.UseObjectMsg
                    msg = feval(obj.MessageInfo.msgClassGen, msg);
                end
                feval(obj.ActualNewMessageFcn, obj, msg, ...
                      obj.NewMessageCallbackArgs{:})
            end
        end

        function subInfo = getServerInfo(obj)
        %getServerInfo Get subscriber properties from node server

        % Ensure properties are valid
            if isempty(obj.InternalNode) || ~isvalid(obj.InternalNode)
                error(message('ros:mlroscpp:subscriber:InvalidInternalNodeError'))
            elseif isempty(obj.ServerNodeHandle) || ...
                    isempty(obj.ServerSubscriberHandle)
                error(message('ros:mlroscpp:subscriber:InvalidServerHandleError'))
            end

            % Extract node information
            try
                nodeInfo = nodeinfo(obj.InternalNode, ...
                                    obj.ServerNodeHandle, []);
            catch ex
                newEx = MException(message('ros:mlroscpp:subscriber:GetInfoError'));
                throw(newEx.addCause(ex));
            end
            subHandles = [nodeInfo.subs.handle];
            whichSub = obj.ServerSubscriberHandle == subHandles;
            if ~any(whichSub)
                % Must be the wrong handle(s) if this subscriber exists
                error(message('ros:mlroscpp:subscriber:InvalidServerHandleError'))
            elseif nnz(whichSub) > 1
                % Duplicate subscriber handles found, error on node side
                error(message('ros:mlroscpp:subscriber:DuplicateSubHandlesError'))
            end
            subInfo = nodeInfo.subs(whichSub);
        end
    end

    methods (Access = private)
        function setupInfo(obj)
        %setupInfo Set info based on the type

            obj.MessageInfo = ros.internal.ros.getMessageInfo(obj.MessageType);
            [obj.MessageInfo.cppFactoryClass , obj.MessageInfo.cppElementType] = ...
                ros.internal.ros.getCPPFactoryClassAndType(obj.MessageType);
        end

        function createSubscriber(obj, serverNodeHandle, topic, type, additionalSettings)
        %createSubscriber Create subscriber on ROS network

        % If enabled, set up to use callback handler method as callback
            if obj.EnableCallback
                callbackFcn = obj.NewMessageCallbackHandler.CallbackName;
            else
                callbackFcn = '';
            end

            dllPaths = ros.internal.utilities.getPathOfDependentDlls(obj.MessageType,'ros');
            additionalSettings.cppElementType = obj.MessageInfo.cppElementType;
            % Create subscriber but do not begin receiving messages
            try
                returnCall = addSubscriber(obj.InternalNode, ...
                                           serverNodeHandle, ...
                                           obj.MessageInfo.path, ...
                                           topic, ...
                                           obj.MessageInfo.cppFactoryClass, ...
                                           callbackFcn, ...
                                           additionalSettings, ...
                                           dllPaths, ...
                                           false);
                if isempty(returnCall) || ~isstruct(returnCall)
                    error(message('ros:mlroscpp:node:InvalidReturnCallError'))
                elseif ~isfield(returnCall, 'handle') || ...
                        isempty(returnCall.handle)
                    error(message('ros:mlroscpp:node:InvalidReturnCallHandleError'))
                end
                obj.ServerSubscriberHandle = returnCall.handle;
            catch ex
                % Checking if error is invalid topic name
                % Catching it and creating own error
                newEx = MException(message('ros:mlroscpp:subscriber:CreateError', ...
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
            catch ex
                newEx = MException(message('ros:mlroscpp:subscriber:StartError', ...
                                           topic, type));
                throw(newEx.addCause(ex));
            end
        end
    end

    methods (Access = ?matlab.unittest.TestCase)
        function [ordinalParser, paramParser] = getParsers(obj)
        % Set up separate parsers for parameters and other input

        % Set up ordered inputs
        % node and topic always needed, type and callback optional
            ordinalParser = inputParser;
            addRequired(ordinalParser, 'node', ...
                        @(x) validateattributes(x, ...
                                                {'ros.Node'}, ...
                                                {'scalar'}, ...
                                                'Subscriber', ...
                                                'node'));
            addRequired(ordinalParser, 'topic', ...
                        @(x) validateattributes(x, ...
                                                {'char', 'string'}, ...
                                                {'scalartext', 'nonempty'}, ...
                                                'Subscriber', ...
                                                'topic'));
            addOptional(ordinalParser, 'type', '', ...
                        @(x) validateattributes(x, ...
                                                {'char', 'string'}, ...
                                                {'scalartext', 'nonempty'}, ...
                                                'Subscriber', ...
                                                'type'))
            % Skip the callback argument - it will be extracted and
            % validated manually

            % Set up name-value pairs
            paramParser = inputParser;
            addParameter(paramParser, 'BufferSize', obj.DefaultBufferSize , ...
                         @(x) validateattributes(x, {'numeric'}, ...
                                                 {'scalar','nonempty','integer','positive'}, ...
                                                 'Subscriber', 'BufferSize'));
            addDataFormatToParser(obj, paramParser, 'Subscriber')
            addParameter(paramParser, 'MaxConcurrentCallbacks', ...
                         get(0, 'RecursionLimit'), ...
                         @(x) validateattributes(x, ...
                                                 {'numeric'}, ...
                                                 {'scalar', 'nonnegative', 'finite'}, ...
                                                 'Subscriber', ...
                                                 'MaxConcurrentCallbacks'))
            addParameter(paramParser, 'EnableCallback', ...
                         obj.DefaultEnableCallback, ...
                         @(x) validateattributes(x, ...
                                                 {'logical'}, ...
                                                 {'scalar'}, ...
                                                 'Subscriber', ...
                                                 'EnableCallback'))

            function validateStringParameter(value, options, funcName, varName)
            % Separate function to suppress output and just validate
                validatestring(value, options, funcName, varName);
            end
        end
    end

    methods(Access = private, Static)
        function name = matlabCodegenRedirect(~)
            name = 'ros.internal.codegen.Subscriber';
        end
    end
end
