classdef ros2publisher < ros.internal.mixin.InternalAccess & ...
        coder.ExternalDependency
    % ros2publisher - Code generation equivalent for ros2publisher
    %   Use the ros2publisher object to publish messages on a topic. When
    %   messages are published on that topic, ROS 2 nodes that subscribe to
    %   that topic receive those messages directly.
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

    % Copyright 2021 The MathWorks, Inc.
    %#codegen

    properties (SetAccess = private)
        %TopicName - The name of the publishing topic
        TopicName

        %MessageType - The message type of publishing messages
        MessageType

        %History - The message queue mode
        History

        %Depth - The message queue size
        Depth

        %Reliability - The delivery guarantee of messages
        Reliability

        %Durability - The persistence of messages
        Durability
    end

    properties (Access = private)
        MsgStruct
        PublisherHelper
    end

    methods
        function obj = ros2publisher(node, topic, varargin)
        %ros2publisher Create a ROS 2 publisher object
        %   Attach a new publisher to the ROS 2 "node" object. The "topic"
        %   argument is required and specifies the topic on which this
        %   publisher should publish. Please see the class documentation
        %   (help ros2publisher) for more details.
            coder.inline('never');
            narginchk(2, inf);
            coder.extrinsic('ros.codertarget.internal.getCodegenInfo');
            % Ensure varargin is not empty
            coder.internal.assert(nargin>2,'ros:mlros2:codegen:MissingMessageType',topic,'ros2publisher');

            % Specialize ros2publisher class based on messageType
            coder.internal.prefer_const(varargin{1});

            %% Check input arguments
            % Validate input ros2node
            validateattributes(node, {'ros2node'}, {'scalar'}, ...
                               'ros2publisher','node');
            % Validate input topic
            topic = convertStringsToChars(topic);
            validateattributes(topic,{'char','string'},{'nonempty'}, ...
                               'ros2publisher','topic');

            % Define optional arguments
            opArgs.MessageType = @(d)coder.internal.isTextRow(d);
            % Define the parameter names
            NVPairNames = {'History','Depth','Reliability', 'Durability'};
            % Select parsing options
            pOpts = struct('PartialMatching',true,'CaseSensitivity',false);
            % Parse the inputs
            pStruct = coder.internal.parseInputs(opArgs, NVPairNames,pOpts,varargin{:});
            % Retrieve input values
            messageType = coder.internal.getParameterValue(pStruct.MessageType,'default',varargin{:});
            messageType = convertStringsToChars(messageType);
            validateattributes(messageType,{'char','string'},{'nonempty'}, ...
                               'ros2publisher','messageType');
            coder.internal.assert(contains(messageType,'/'),'ros:mlros2:codegen:MissingMessageType',topic,'ros2publisher');

            % Retrieve name-value pairs
            qosHistory = coder.internal.getParameterValue(pStruct.History,'keeplast',varargin{:});
            validateStringParameter(qosHistory,{'keeplast', 'keepall'},'ros2publisher','History');

            qosDepth = coder.internal.getParameterValue(pStruct.Depth,1,varargin{:});
            validateattributes(qosDepth,{'numeric'},...
                               {'scalar','nonempty','integer','nonnegative'},...
                               'ros2publisher','Depth');

            qosReliability = coder.internal.getParameterValue(pStruct.Reliability,'reliable',varargin{:});
            validateStringParameter(qosReliability,{'reliable', 'besteffort'},'ros2publisher','Reliability');

            qosDurability = coder.internal.getParameterValue(pStruct.Durability,'volatile',varargin{:});
            validateStringParameter(qosDurability,{'transientlocal', 'volatile'},'ros2publisher','Durability');

            % Resolve the topic name based on the node
            resolvedTopic = resolveName(node, topic);

            % Store input arguments
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

            % Get and register code generation information
            cgInfo = coder.const(@ros.codertarget.internal.getCodegenInfo,topic,messageType,'pub', 'ros2');
            msgStructGenFcn = str2func(cgInfo.MsgStructGen);
            obj.MsgStruct = msgStructGenFcn();  % Setup return type

            % Create an instance of MATLABROS2Publisher object
            % template <class MsgType, class StructType>
            % MATLABROS2Publisher(MsgType* msgPtr)
            templateTypeStr = ['MATLABROS2Publisher<' cgInfo.MsgClass ',' cgInfo.MsgStructGen '_T>'];
            obj.PublisherHelper = coder.opaque(['std::unique_ptr<' templateTypeStr '>'],...
                'HeaderFile','mlros2_pub.h');
            obj.PublisherHelper = coder.ceval(['std::unique_ptr<', templateTypeStr, ...
                '>(new ', templateTypeStr, '{});//']);
            coder.ceval('MATLABROS2Publisher_createPublisher', ...
                obj.PublisherHelper, node.NodeHandle, coder.rref(obj.TopicName),...
                size(obj.TopicName,2), qos_profile);
        end

        function send(obj, msgToSend)
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
            validateattributes(msgToSend, {'struct'}, {'scalar'}, 'send', 'msg');
            coder.ceval('MATLABROS2Publisher_publish',obj.PublisherHelper,...
                coder.rref(msgToSend));
        end

        function msgFromPub = ros2message(obj)
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

            msgFromPub = ros2message(obj.MessageType);
        end
    end

    methods (Static)
        function props = matlabCodegenNontunableProperties(~)
            props = {'MessageType'};
        end

        function ret = getDescriptiveName(~)
            ret = 'ROS 2 Publisher';
        end

        function ret = isSupportedContext(bldCtx)
            ret = bldCtx.isCodeGenTarget('rtw');
        end

        function updateBuildInfo(buildInfo,bldCtx)
            if bldCtx.isCodeGenTarget('rtw')
                srcFolder = fullfile(toolboxdir('ros'),'codertarget','src');
                addIncludeFiles(buildInfo,'mlros2_pub.h',srcFolder);
                addIncludeFiles(buildInfo,'mlros2_qos.h',srcFolder);
            end
        end
    end
end

function validateStringParameter(value, options, funcName, varName)
% Separate function to suppress output and just validate
    validatestring(value, options, funcName, varName);
end
