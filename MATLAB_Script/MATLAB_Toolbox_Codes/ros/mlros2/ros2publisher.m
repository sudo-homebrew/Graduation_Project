classdef ros2publisher < ros.ros2.internal.QOSUser & ...
        ros.internal.mixin.InternalAccess & ...
        robotics.core.internal.mixin.Unsaveable & handle
    %ros2publisher Publish messages on a topic
    %   Use the ros2publisher object to publish messages on a topic. When
    %   messages are published on that topic, ROS 2 nodes that subscribe to
    %   that topic receive those messages directly.
    %
    %   PUB = ros2publisher(NODE,"TOPIC") creates a publisher, PUB, for a topic
    %   with name TOPIC that already exists on the ROS 2 network topic list.
    %   NODE is the ros2node object handle to which the publisher should
    %   attach.
    %   The publisher gets the topic message type from the network topic
    %   list. If the topic is not on the network topic list, this function
    %   displays an error message.
    %
    %   PUB = ros2publisher(NODE,"TOPIC","TYPE") creates a publisher for a
    %   topic and adds that topic to the network topic list. If the topic list
    %   already contains a matching topic, NODE is added to the list of
    %   publishers for that topic.
    %   If TYPE differs from the message type for that topic on the network
    %   topic list, the function displays an error.
    %
    %   PUB = ros2publisher(___,Name,Value) provides additional options
    %   specified by one or more Name,Value pair arguments. You can specify
    %   several name-value pair arguments in any order as
    %   Name1,Value1,...,NameN,ValueN:
    %
    %      "History"     - Mode for storing messages in the queue. The queued
    %                      messages are be sent to late-joining subscribers.
    %                      If the queue fills with messages waiting to be
    %                      processed, then old messages are dropped to make
    %                      room for new. Options are:
    %                         "keeplast"       - Store up to the number of
    %                                            messages set by 'Depth'.
    %                         "keepall"        - Store all messages
    %                                            (up to resource limits).
    %      "Depth"       - Size of the message queue in number of messages.
    %                      Only applies if "History" property is "keeplast".
    %      "Reliability" - Method for ensuring message delivery. Options are:
    %                         "reliable"       - Guaranteed delivery, but
    %                                            may make multiple attempts to
    %                                            publish.
    %                         "besteffort"     - Attempt delivery once.
    %      "Durability"  - Method for storing messages on the publisher.
    %                      Late-joining subscribers can receive messages if
    %                      they persist. Options are:
    %                         "volatile"       - Messages do not persist.
    %                         "transientlocal" - Recently sent messages persist.
    %
    %   NOTE: The "Reliability" and "Durability" quality of service settings
    %   must be compatible between publishers and subscribers for a connection
    %   to be made.
    %
    %   [PUB,MSG] = ros2publisher(___) returns a message structure, MSG, that
    %   you can send with the publisher PUB. The message will be initialized
    %   with default values.
    %
    %   Publisher properties:
    %      TopicName   - (Read-only) Name of the published topic
    %      MessageType - (Read-Only) Message type of published messages
    %      History     - (Read-only) Message queue mode
    %      Depth       - (Read-only) Message queue size
    %      Reliability - (Read-Only) Delivery guarantee of messages
    %      Durability  - (Read-Only) Persistence of messages
    %
    %   Publisher methods:
    %      send        - Publish a message
    %      ros2message - Create an empty message you can send with this publisher
    %
    %   Example:
    %      % Create a ROS 2 node
    %      node = ros2node("/node_1");
    %
    %      % Create publisher and send string data
    %      chatPub = ros2publisher(node,"/chatter","std_msgs/String");
    %      msg = ros2message(chatPub);
    %      msg.data = 'Message from default publisher';
    %      send(chatPub,msg);
    %
    %      % Create another publisher on the same topic and send a message.
    %      % Set durability to ensure messages persist on the publisher.
    %      persistPub = ros2publisher(node,"/chatter",...
    %          "Durability","transientlocal");
    %      msg.data = 'Message from persisting publisher';
    %      send(persistPub,msg);
    %
    %      % Create a subscriber on the "/chatter" topic and receive the latest
    %      % message. Because of quality of service compatibility, only the
    %      % second publisher's messages are received by this subscriber.
    %      sub = ros2subscriber(node,"/chatter", ...
    %          "Durability","transientlocal");
    %      pause(1)
    %      msgReceived = sub.LatestMessage

    %   Copyright 2019-2021 The MathWorks, Inc.

    properties (Dependent, SetAccess = private)
        %TopicName - The name of the published topic
        TopicName
    end

    properties (SetAccess = private)
        %MessageType - The message type of published messages
        MessageType = ''
    end

    properties (Transient, Access = ?ros.internal.mixin.InternalAccess)
        %InternalNode - Internal representation of the node object
        %   Node required to publish messages
        InternalNode = []

        %ServerNodeHandle - Designation of the node on the server
        %   Node handle required to get publisher property information
        ServerNodeHandle = []

        %ServerPublisherHandle - Designation of the publisher on the server
        %   Publisher handle required to publish messages
        ServerPublisherHandle = []

        %MessageInfo - includes other information for a given message
        MessageInfo = struct.empty
    end

    properties (Dependent, Access = ?ros.internal.mixin.InternalAccess)
        %GUID - Global Unique Identifier of the publisher
        GUID
    end

    properties (Constant, Access = private)
        %NumTopicRetries - Number of times to retry getting topic
        NumTopicRetries = 3

        %TopicRetryDelay - Time between retries to get topic
        TopicRetryDelay = 0.1
    end

    methods (Access = public)
        function [obj, varargout] = ros2publisher(node, topic, varargin)
        %ros2publisher Create a ROS 2 publisher object
        %   Attach a new publisher to the ROS 2 "node" object. The "topic"
        %   argument is required and specifies the topic on which this
        %   publisher should publish. Please see the class documentation
        %   (help ros2publisher) for more details.

            narginchk(2, inf)

            % Convert all string arguments to characters
            [topic, varargin{:}] = convertStringsToChars(topic, varargin{:});

            % Parse name-value pairs separately from other input
            [ordinalParser, paramParser] = getParsers(obj);
            nvPairsStart = ros.internal.Parsing.findNameValueIndex(...
                varargin, paramParser.Parameters);
            if isempty(nvPairsStart)
                % Additional arguments past type assume to be parameters
                nvPairsStart = min(2, numel(varargin)+1);
            end
            parse(ordinalParser, node, topic, varargin{1:nvPairsStart-1});
            parse(paramParser, varargin{nvPairsStart:end})

            % Save the internal node information for later use
            obj.InternalNode = ordinalParser.Results.node.InternalNode;
            obj.ServerNodeHandle = ordinalParser.Results.node.ServerNodeHandle;

            % Resolve the topic name based on the node
            resolvedTopic = resolveName(ordinalParser.Results.node, ...
                                        ordinalParser.Results.topic);

            % Determine message type for topic from network
            requestedType = convertStringsToChars(ordinalParser.Results.type);
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
                newEx = MException(message('ros:mlros2:publisher:CreateError', ...
                                           resolvedTopic, ...
                                           requestedType));
                throw(newEx.addCause(ex));
            end

            % Determine message type for publisher
            % Topic unlikely to but may have multiple valid types
            if isempty(requestedType)           % Select type from network
                if isempty(existingType)        % Topic not yet used
                    error(message('ros:mlros2:publisher:TopicNameNotFound', ...
                                  resolvedTopic))
                else                            % Use message from topic
                    obj.MessageType = existingType{1};
                end
            else                                % User-specified type
                if isempty(existingType) || ... % Topic not yet used
                        ismember(requestedType, existingType)   % Matching type
                    obj.MessageType = requestedType;
                else                            % Conflicting type
                    error(message('ros:mlros2:publisher:TopicTypeNoMatch', ...
                                  resolvedTopic, ...
                                  existingType{1}, ...
                                  requestedType))
                end
            end

            % Handle quality of service settings
            qosSettings = getQosSettings(obj, paramParser.Results);

            % Set info based on the message
            setupInfo(obj);

            % Create the publisher
            createPublisher(obj, ordinalParser.Results.node.ServerNodeHandle, ...
                            resolvedTopic, ...
                            obj.MessageType, ...
                            qosSettings);

            % Return message structure if requested
            if nargout > 1
                varargout{1} = ros2message(obj);
            end

            function [ordinalParser, paramParser] = getParsers(obj)
            % Set up separate parsers for parameters and other input

            % Set up ordered inputs
                ordinalParser = inputParser;
                addRequired(ordinalParser, 'node', ...
                            @(x) validateattributes(x, ...
                                                    {'ros2node'}, ...
                                                    {'scalar'}, ...
                                                    'ros2publisher', ...
                                                    'node'));
                addRequired(ordinalParser, 'topic', ...
                            @(x) validateattributes(x, ...
                                                    {'char', 'string'}, ...
                                                    {'scalartext', 'nonempty'}, ...
                                                    'ros2publisher', ...
                                                    'topic'));
                addOptional(ordinalParser, 'type', '', ...
                            @(x) validateattributes(x, ...
                                                    {'char', 'string'}, ...
                                                    {'scalartext', 'nonempty'}, ...
                                                    'ros2publisher', ...
                                                    'type'))

                % Set up name-value pair inputs
                paramParser = inputParser;
                paramParser = addQOSToParser(obj, paramParser, 'ros2publisher');
            end
        end

        function delete(obj)
        %delete Shut down publisher
        %   delete(PUB) shuts down the ROS 2 publisher object PUB and
        %   removes it from the network

            try
                % Cannot tell server to remove the publisher without valid
                % internal node and server handle value
                if ~isempty(obj.InternalNode) && ...
                        isvalid(obj.InternalNode) && ...
                        ~isempty(obj.ServerPublisherHandle)
                    removePublisher(obj.InternalNode, ...
                                    obj.ServerPublisherHandle);
                    % No need to check reply - should error on failure
                end
                obj.InternalNode = [];
            catch ex
                warning(message('ros:mlros2:publisher:ShutdownError', ...
                                ex.message))
            end
        end

        function send(obj, msg)
        % send Publish a message to a topic.
        %   send(PUB,MSG) publishes a message MSG to the topic advertised
        %   by the publisher PUB.
        %
        %   Example:
        %      % Create publisher
        %      pub = ros2publisher(node,"/chatter","std_msgs/String");
        %
        %      % Create string message and publish it
        %      msg = ros2message(pub);
        %      msg.Data = "Some test string";
        %      send(pub,msg);

        % Do basic error checking (further checking done internally)
            validateattributes(msg, {'struct'}, {'scalar'}, 'send', 'msg')

            % Publish message
            try
                reply = publish(obj.InternalNode, ...
                                obj.ServerPublisherHandle, msg);
                if ~reply
                    error(message('ros:mlros2:publisher:PublishReturnCallError'))
                end
            catch ex
                newEx = MException(message('ros:mlros2:publisher:PublishError', ...
                                           obj.MessageType));
                throw(newEx.addCause(ex));
            end
        end

        function msg = ros2message(obj)
        % ROS2MESSAGE Create an empty message structure you can send with this publisher
        %   MSG = ROS2MESSAGE(PUB) creates and returns an empty message
        %   structure MSG. The message type of MSG is determined by the
        %   topic that the publisher PUB is advertising.
        %
        %   Example:
        %
        %      % Create a ROS 2 node
        %      node = ros2node("/node_1");
        %
        %      % Create publisher and message
        %      chatPub = ros2publisher(node,"/chatter","std_msgs/String");
        %      msg = ros2message(chatPub);
        %
        %   See also SEND.

            msg = ros2message(obj.MessageType);
        end
    end

    % All dependent properties are read from the server
    methods
        function topic = get.TopicName(obj)
        %get.TopicName Custom getter for TopicName property

        % Allow errors to be thrown from getServerInfo
            pubInfo = getServerInfo(obj);
            topic = pubInfo.topic;
        end

        function guid = get.GUID(obj)
        %get.GUID Custom getter for GUID property

        % Allow errors to be thrown from getServerInfo
            pubInfo = getServerInfo(obj);
            guid = pubInfo.gid;
        end
    end

    methods (Access = private)
        function setupInfo(obj)
        %setupInfo Set info based on the type

            obj.MessageInfo = ros.internal.ros2.getMessageInfo(obj.MessageType);
            [obj.MessageInfo.cppFactoryClass , obj.MessageInfo.cppElementType] = ...
                ros.internal.ros2.getCPPFactoryClassAndType(obj.MessageType);
        end

        function createPublisher(obj, serverNodeHandle, topic, type, additionalSettings)
        %createPublisher Create publisher on ROS 2 network
            dllPaths = ros.internal.utilities.getPathOfDependentDlls(obj.MessageType,'ros2');
            additionalSettings.cppElementType = obj.MessageInfo.cppElementType;
            try
                returnCall = addPublisher(obj.InternalNode, ...
                                          serverNodeHandle, ...
                                          obj.MessageInfo.path, ...
                                          topic, ...
                                          obj.MessageInfo.cppFactoryClass, ...
                                          additionalSettings, ...
                                          dllPaths);
                if isempty(returnCall) || ~isstruct(returnCall)
                    error(message('ros:mlros2:node:InvalidReturnCallError'))
                elseif ~isfield(returnCall, 'handle') || ...
                        isempty(returnCall.handle)
                    error(message('ros:mlros2:node:InvalidReturnCallHandleError'))
                end
                obj.ServerPublisherHandle = returnCall.handle;
            catch ex
                % Checking if error is invalid topic name
                % Catching it and creating own error

                invalidTopicErrorText1 = ...
                    message('ros:mlros2:publisher:InvalidTopicErrorText1');
                invalidTopicErrorText2 = ...
                    message('ros:mlros2:publisher:InvalidTopicErrorText2');

                if contains(ex.message,invalidTopicErrorText1.string) || ...
                        contains(ex.message,invalidTopicErrorText2.string)
                    error(message('ros:mlros2:publisher:InvalidTopicName',...
                                  topic));
                end

                newEx = MException(message('ros:mlros2:publisher:CreateError', ...
                                           topic, type));
                throw(newEx.addCause(ex));
            end
        end
    end

    methods (Access = protected)
        function pubInfo = getServerInfo(obj)
        %getServerInfo Get publisher properties from node server

        % Ensure properties are valid
            if isempty(obj.InternalNode) || ~isvalid(obj.InternalNode)
                error(message('ros:mlros2:publisher:InvalidInternalNodeError'))
            elseif isempty(obj.ServerNodeHandle) || ...
                    isempty(obj.ServerPublisherHandle)
                error(message('ros:mlros2:publisher:InvalidServerHandleError'))
            end

            % Extract node information
            try
                nodeInfo = nodeinfo(obj.InternalNode, ...
                                    obj.ServerNodeHandle, []);
            catch ex
                newEx = MException(message('ros:mlros2:publisher:GetInfoError'));
                throw(newEx.addCause(ex));
            end
            pubHandles = [nodeInfo.pubs.handle];
            whichPub = obj.ServerPublisherHandle == pubHandles;
            if ~any(whichPub)
                % Must be the wrong handle(s) if this publisher exists
                error(message('ros:mlros2:publisher:InvalidServerHandleError'))
            elseif nnz(whichPub) > 1
                % Duplicate publisher handles found, error on node side
                error(message('ros:mlros2:publisher:DuplicatePubHandlesError'))
            end
            pubInfo = nodeInfo.pubs(whichPub);
        end
    end

    %----------------------------------------------------------------------
    % MATLAB Code-generation
    %----------------------------------------------------------------------
    methods (Static = true, Access = private)
        function name = matlabCodegenRedirect(~)
            name = 'ros.internal.codegen.ros2publisher';
        end
    end
end
