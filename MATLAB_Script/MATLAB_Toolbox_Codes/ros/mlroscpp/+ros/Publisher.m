classdef Publisher < ros.internal.mixin.ROSInternalAccess & ...
        ros.internal.DataFormatBase & ...
        robotics.core.internal.mixin.Unsaveable & handle
    %Publisher Publish messages on a topic
    %   Use the Publisher object to publish messages on a topic. When MATLAB's
    %   global node publishes messages on that topic, ROS nodes that
    %   subscribe to that topic receive those messages directly from the
    %   global node.
    %
    %   PUB = ros.Publisher(NODE,TOPIC) creates a publisher, PUB,
    %   for a topic with name TOPIC that already exists on the ROS
    %   master topic list. NODE is the ros.Node object handle that this
    %   publisher should attach to. If NODE is [], the publisher will try
    %   to attach to the global ROS node.
    %   The publisher gets the topic message type from the topic list on the ROS master.
    %   If the topic is not on the ROS master topic list, this function
    %   displays an error message. TOPIC is a string scalar.
    %
    %   PUB = ros.Publisher(NODE,TOPIC,TYPE) creates a publisher for a topic
    %   and adds that topic to the ROS master topic list. If the ROS master
    %   topic list already contains a matching topic, the ROS master adds
    %   the MATLAB global node to the list of publishers for that topic.
    %   If the topic type differs from the topic on ROS master topic list,
    %   the function displays an error. TYPE is a string scalar.
    %
    %   PUB = ros.Publisher(___,Name,Value) provides additional options
    %   specified by one or more Name,Value pair arguments. You can specify
    %   several name-value pair arguments in any order as
    %   Name1,Value1,...,NameN,ValueN:
    %
    %      "DataFormat" - Determines format of ROS message to be used by
    %                     the publisher, and returned from rosmessage.
    %                     Using structs can be faster than using message
    %                     objects.
    %                     Options are:
    %                        "object" - Message object of the specified type
    %                        "struct" - Message struct with compatible fields
    %                     Default: "object"
    %
    %      "IsLatching" - If this publisher is latching, it saves the last
    %                     sent message and re-sends it to any new
    %                     subscribers. By default, latching mode is enabled.
    %                     To disable the latching behavior, set Value to false.
    %                     Default: true
    %
    %
    %   Publisher properties:
    %      TopicName       - (Read-Only) The name of the published topic
    %      MessageType     - (Read-Only) The message type of published messages
    %      DataFormat      - (Read-Only) Message format required for use
    %      IsLatching      - (Read-Only) Indicates if publisher is latching
    %      NumSubscribers  - (Read-Only) The number of current subscribers
    %
    %   Publisher methods:
    %      send        - Publish a message
    %      rosmessage  - Create an empty message compatible with this publisher
    %
    %
    %   Example:
    %
    %      % Create the ROS master and a node
    %      master = ros.Core;
    %      node = ros.Node('/test1');
    %
    %      % Create publisher and send string data
    %      % Use struct message format for better performance
    %      chatPub = ros.Publisher(node,"/chatter","std_msgs/String",...
    %          "DataFormat","struct");
    %      msg = rosmessage(chatPub);
    %      msg.Data = 'Some test string';
    %      send(chatPub,msg);
    %
    %      % Create another publisher on the same topic
    %      % Message type will be inferred, disable latching
    %      latchPub = ros.Publisher(node,"/chatter",...
    %          "DataFormat","struct","IsLatching",false);
    %      send(latchPub,msg);
    %
    %      % Create publisher that uses message objects
    %      posePub = ros.Publisher(node,"/location","geometry_msgs/Pose2D",...
    %          "DataFormat","object");
    %      msgObj = rosmessage(posePub); % Uses DataFormat from publisher
    %      msgObj.X = 1;
    %      send(posePub,msgObj)
    %
    %   See also ROSPUBLISHER, ROSMESSAGE, ROSTOPIC.

    %   Copyright 2014-2020 The MathWorks, Inc.

    properties (Dependent, SetAccess = private)
        %TopicName - The name of the published topic
        TopicName

        %NumSubscribers - The number of current subscribers
        NumSubscribers
    end

    properties (SetAccess = private)
        %IsLatching - Indicates if publisher is latching
        IsLatching
    end

    properties (SetAccess = private)
        %MessageType - The message type of published messages
        %   This property is not dependent, since it is accessed frequently
        %   when publishing.
        MessageType = ''
    end

    properties (Transient, Access = ?ros.internal.mixin.ROSInternalAccess)
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

        %MasterURI - includes Master URI of the given node
        MasterURI = []
    end


    properties (Constant, Access = private)
        %NumTopicRetries - Number of times to retry getting topic
        NumTopicRetries = 3

        %TopicRetryDelay - Time between retries to get topic
        TopicRetryDelay = 0.1

        %DefaultIsLatching - Default latching value
        DefaultIsLatching = true
    end

    methods (Access = public)
        function obj = Publisher(node, topic, varargin)
        %Publisher - Create a ROS publisher object
        %   Attach a new publisher to the ROS "node" object. The "topic"
        %   argument is required and specifies the topic on which this
        %   publisher should publish. Please see the class documentation
        %   (help ros.Publisher) for more details.

            if isempty(node)
                node = ros.internal.Global.getNodeHandle(false);
            end

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

            try
                parse(ordinalParser, node, topic, varargin{1:nvPairsStart-1});
            catch except
                if contains(except.message, 'ros.Node')
                    error(message('ros:mlros:publisher:NoNodeInput',class(node)));
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
            obj.MasterURI = ordinalParser.Results.node.MasterURI;
            obj.InternalNode = ordinalParser.Results.node.InternalNode;
            obj.ServerNodeHandle = ordinalParser.Results.node.ServerNodeHandle;
            obj.IsLatching = paramParser.Results.IsLatching;
            obj.MessageType = masterType;
            setDataFormat(obj, paramParser.Results.DataFormat)
            node.ListofNodeDependentHandles{end+1} = matlab.internal.WeakHandle(obj);

            % Set info based on the message
            setupInfo(obj);

            % Create the publisher
            latchingSettingsStruct = ...
                struct('latching', paramParser.Results.IsLatching);
            createPublisher(obj, node.ServerNodeHandle, ...
                            resolvedTopic, ...
                            obj.MessageType, ...
                            latchingSettingsStruct);

        end

        function delete(obj)
        %DELETE Shut down publisher
        %   DELETE(PUB) shuts down the ROS publisher object PUB and removes
        %   its registration from the ROS master

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
                warning(message('ros:mlroscpp:publisher:ShutdownError', ...
                                ex.message))
            end
        end

        function send(obj, msg)
        % SEND Publish a message to a topic.
        %   SEND(PUB,MSG) publishes a message MSG to the topic advertised
        %   by the publisher PUB. The type and format of the message must
        %   match the MessageType and DataFormat of the publisher.
        %
        %   Example:
        %      % Create publisher, using struct message format for performance
        %      chatPub = ros.Publisher(node,"/chatter","std_msgs/String",...
        %          "DataFormat","struct");
        %
        %      % Create string message and publish it
        %      msg = rosmessage(chatPub);
        %      msg.Data = 'Some test string';
        %      SEND(chatPub,msg);
        %
        %      % Create a publisher and send a message object
        %      scanPub = ros.Publisher(node,"/scan","sensor_msgs/LaserScan",...
        %          "DataFormat","object");
        %      msgObj = rosmessage(scanPub);
        %      SEND(scanPub,msgObj)

        % Publish message
            try
                if obj.UseObjectMsg
                    msgStruct = toROSStruct(msg);
                else
                    msgStruct = msg;
                end
                reply = publish(obj.InternalNode, ...
                                obj.ServerPublisherHandle, msgStruct);
                if ~reply
                    error(message('ros:mlroscpp:publisher:PublishReturnCallError'))
                end
            catch ex
                if strcmp(ex.identifier, ...
                          'ros:internal:transport:MasterNotReachableError')
                    error(message('ros:mlros:node:NoMasterConnection', obj.MasterURI));
                else
                    validateInputMessage(obj, msg, obj.MessageType, 'Publisher', 'send')
                    newEx = MException(message('ros:mlros:publisher:PublishError', ...
                                               obj.MessageType,obj.TopicName));
                    throw(newEx.addCause(ex));
                end
            end
        end

        function msg = rosmessage(obj, varargin)
        % ROSMESSAGE Create an empty message that you can send with this publisher
        %   MSG = ROSMESSAGE(PUB) creates and returns an empty message MSG. The
        %   message type of MSG is determined by the topic that the publisher
        %   PUB is advertising. The format of MSG is determined by the
        %   DataFormat of the publisher. The message MSG can be sent with this
        %   publisher.
        %
        %
        %   Example:
        %      % Create publisher and message
        %      chatPub = ros.Publisher(node,"/chatter","std_msgs/String");
        %      msgObj = ROSMESSAGE(chatPub);
        %
        %      % Improve performance by using struct messages
        %      posePub = ros.Publisher(node,"/pose","geometry_msgs/Pose2D","DataFormat","struct");
        %      msgStruct = ROSMESSAGE(posePub);
        %
        %   See also SEND.

            validateDataFormatROSMessage(obj, varargin{:})

            msg = rosmessage(obj.MessageType, 'DataFormat', obj.DataFormat);
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

        function num = get.NumSubscribers(obj)
        %get.NumSubscribers Custom getter for NumSubscribers property

            num = ros.internal.NetworkIntrospection.getNumberofSubscribers(obj.TopicName,obj.MasterURI);
        end
    end

    methods (Access = private)
        function setupInfo(obj)
        %setupInfo Set info based on the type

            obj.MessageInfo = ros.internal.ros.getMessageInfo(obj.MessageType);
            [obj.MessageInfo.cppFactoryClass , obj.MessageInfo.cppElementType] = ...
                ros.internal.ros.getCPPFactoryClassAndType(obj.MessageType);
        end

        function createPublisher(obj, serverNodeHandle, topic, type, additionalSettings)
        %createPublisher Create publisher on ROS network

            dllPaths = ros.internal.utilities.getPathOfDependentDlls(obj.MessageType,'ros');
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
                    error(message('ros:mlroscpp:node:InvalidReturnCallError'))
                elseif ~isfield(returnCall, 'handle') || ...
                        isempty(returnCall.handle)
                    error(message('ros:mlroscpp:node:InvalidReturnCallHandleError'))
                end
                obj.ServerPublisherHandle = returnCall.handle;
            catch ex
                newEx = MException(message('ros:mlroscpp:publisher:CreateError', ...
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
                error(message('ros:mlroscpp:publisher:InvalidInternalNodeError'))
            elseif isempty(obj.ServerNodeHandle) || ...
                    isempty(obj.ServerPublisherHandle)
                error(message('ros:mlroscpp:publisher:InvalidServerHandleError'))
            end

            % Extract node information
            try
                nodeInfo = nodeinfo(obj.InternalNode, ...
                                    obj.ServerNodeHandle, []);
            catch ex
                newEx = MException(message('ros:mlroscpp:publisher:GetInfoError'));
                throw(newEx.addCause(ex));
            end
            pubHandles = [nodeInfo.pubs.handle];
            whichPub = obj.ServerPublisherHandle == pubHandles;
            if ~any(whichPub)
                % Must be the wrong handle(s) if this publisher exists
                error(message('ros:mlroscpp:publisher:InvalidServerHandleError'))
            elseif nnz(whichPub) > 1
                % Duplicate publisher handles found, error on node side
                error(message('ros:mlroscpp:publisher:DuplicatePubHandlesError'))
            end
            pubInfo = nodeInfo.pubs(whichPub);
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
                                                'Publisher', ...
                                                'node'));
            addRequired(ordinalParser, 'topic', ...
                        @(x) validateattributes(x, ...
                                                {'char', 'string'}, ...
                                                {'scalartext', 'nonempty'}, ...
                                                'Publisher', ...
                                                'topic'));
            addOptional(ordinalParser, 'type', '', ...
                        @(x) validateattributes(x, ...
                                                {'char', 'string'}, ...
                                                {'scalartext', 'nonempty'}, ...
                                                'Publisher', ...
                                                'type'))

            % Set up name-value pairs
            paramParser = inputParser;
            addParameter(paramParser, 'IsLatching', obj.DefaultIsLatching, ...
                         @(x) validateattributes(x, {'numeric', 'logical'}, ...
                                                 {'scalar','nonempty'}, ...
                                                 'Publisher', 'IsLatching'));
            addDataFormatToParser(obj, paramParser, 'Publisher')
        end
    end

    %----------------------------------------------------------------------
    % MATLAB Code-generation
    %----------------------------------------------------------------------
    methods (Static = true, Access = private)
        function name = matlabCodegenRedirect(~)
            name = 'ros.internal.codegen.Publisher';
        end
    end
end
