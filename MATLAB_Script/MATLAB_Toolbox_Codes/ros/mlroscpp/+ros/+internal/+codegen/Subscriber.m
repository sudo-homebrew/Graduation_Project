classdef Subscriber < ros.internal.mixin.ROSInternalAccess & ...
        coder.ExternalDependency
%Subscriber Subscribe to messages on a topic
%   The primary tool for transferring data in ROS is publishing and
%   subscribing to messages. Messages are sent through topics and
%   ros.Subscriber allows you to subscribe to these topics.
%   When ROS nodes publish messages on that topic, MATLAB will receive
%   those message through this subscriber.
%
%   SUB = ros.Subscriber(NODE, 'TOPICNAME') subscribes to a topic with name
%   TOPICNAME. NODE is the ros.Node object handle that this subscriber
%   should attach to.
%   If the ROS master topic list includes TOPICNAME, this
%   syntax returns a subscriber object handle, SUB. If the ROS master
%   topic list does not include the topic, this syntax displays an error.
%
%   SUB = ros.Subscriber(NODE, 'TOPIC', 'TYPE') subscribes to a topic
%   that has the specified name TOPICNAME and type MESSAGETYPE. This syntax
%   returns a subscriber object handle. If the topic list on ROS master
%   does not include a topic with the specified name and type, a topic
%   with the specific name and type is added to the topic list.
%   Use this syntax to avoid errors when it is possible for the subscriber
%   to subscribe to a topic before a publisher has added the topic to
%   the topic list on the ROS master.
%
%   SUB = ros.Subscriber(NODE, 'TOPIC', CB) specifies a callback function CB,
%   and optional data, to run when the subscriber object handle receives
%   a topic message. Use this syntax to avoid blocking wait functions.
%   CB can be a single function handle or a cell array. The
%   first element of the cell array needs to be a function handle or a
%   string containing the name of a function. The remaining elements of
%   the cell array can be arbitrary user data that will be passed to
%   the callback function.
%
%   SUB = ros.Subscriber(NODE, 'TOPIC', 'TYPE', CB)  specifies a
%   callback function CB, and subscribes to a topic that has the
%   specified name TOPICNAME and type MESSAGETYPE.
%
%   SUB = ros.Subscriber(___, Name, Value) provides additional options
%   specified by one or more Name,Value pair arguments. Name must appear
%   inside single quotes (''). You can specify several name-value pair
%   arguments in any order as Name1,Value1,...,NameN,ValueN:
%
%      'BufferSize'  - specifies the size of the queue for incoming
%                      messages. If messages are arriving faster and
%                      your callback is unable to process them, they
%                      will be thrown away once the incoming queue
%                      is full. The default value is 1.
%
%   The subscriber callback function requires at least two input
%   arguments. The first argument SRC is the associated subscriber object.
%   The second argument is the received message object MSG. The function
%   header for the callback is as follows:
%
%      function subCallback(SRC, MSG)
%
%   You pass additional parameters to the callback function by including
%   both the callback function and the parameters as elements of a cell array
%   when setting the callback.
%
%
%   Subscriber properties:
%      TopicName       - (Read-only) The name of the subscribed topic
%      MessageType     - (Read-only) The message type of subscribed messages
%      LatestMessage   - (Read-only) The latest message that was received
%      BufferSize      - (Read-only) The buffer size of the incoming queue
%      NewMessageFcn   - Callback property for subscriber callbacks
%
%   Subscriber methods:
%      receive     - Block until data is received and return new message
%      rosmessage - Create an empty message based on the topic type
%
%
%   Example:
%
%      % Create the ROS master and a node
%      master = ros.Core;
%      node = ros.Node('/test1');
%
%      % Create subscriber
%      laser = ros.Subscriber(node, '/scan', 'sensor_msgs/LaserScan');
%
%      % Show latest message that was received (if any)
%      scan = laser.LatestMessage;
%
%      % Wait for next message to arrive (blocking)
%      scan = receive(laser);
%
%      % Create subscriber with callback function
%      % The topic type is inferred (if topic /chatter exists)
%      chatpub = ros.Publisher(node, '/chatter', 'std_msgs/String');
%      chatsub = ros.Subscriber(node, '/chatter', @testCallback);
%
%      % Create subscriber with buffer size of 5
%      chatbuf = ros.Subscriber(node, '/chatter', 'BufferSize', 5);
%
%   See also ROSSUBSCRIBER, ROSMESSAGE.

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

        %BufferSize - The buffer size of the incoming queue
        BufferSize

        %NewMessageFcn - Callback property for subscriber callbacks
        NewMessageFcn

        %DataFormat - Message format provided by subscriber
        DataFormat
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
        function obj = Subscriber(node, topic, varargin)
        %Subscriber - Create a ROS subscriber object
        %   Attach a new subscriber to the ROS node object. The topic
        %   argument is required and specifies the topic on which this
        %   subscriber should receive messages. Please see the class documentation
        %   (help ros.Subscriber) for more details.
            coder.inline('never');
            coder.extrinsic('ros.codertarget.internal.getCodegenInfo');
            coder.extrinsic('ros.codertarget.internal.ROSMATLABCgenInfo');
            coder.extrinsic('ros.codertarget.internal.ROSMATLABCgenInfo.getInstance');
            coder.extrinsic('ros.codertarget.internal.getEmptyCodegenMsg');

            % Ensure varargin is not empty
            coder.internal.assert(nargin>2,'ros:mlroscpp:codegen:MissingMessageType',topic,'rossubscriber');

            % Specialize Subscriber class based on messageType
            coder.internal.prefer_const(varargin{1});

            %% Check input arguments
            if ~isempty(node)
                % A node cannot create another node in codegen
                coder.internal.assert(false,'ros:mlroscpp:codegen:NodeMustBeEmpty');
            end

            % Message type must be specified for codegen. It is always the
            % first argument passed after topic (varargin{1})
            topic = convertStringsToChars(topic);
            validateattributes(topic,{'char','string'},{'nonempty'}, ...
                               '','topic');
            messageType = convertStringsToChars(varargin{1});
            validateattributes(messageType,{'char','string'},{'nonempty'}, ...
                               'Subscriber','messageType');

            % Ensure that the message type has been specified
            coder.internal.assert(contains(messageType,'/'),'ros:mlroscpp:codegen:MissingMessageType',topic,'rossubscriber');

            % Extract callback function if specified
            indx = 2;
            if nargin > 3
                % laserSub = Subscriber(node,"/scan","sensor_msgs/LaserScan");
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
            nvPairs = struct('DataFormat',uint32(0),...
                             'BufferSize',uint32(0));
            pOpts = struct('PartialMatching',true,'CaseSensitivity',false);
            pStruct = coder.internal.parseParameterInputs(nvPairs,pOpts,varargin{indx:end});
            dataFormat = coder.internal.getParameterValue(pStruct.DataFormat,'class',varargin{indx:end});
            validateStringParameter(dataFormat,{'class','struct'},'Subscriber','DataFormat');
            coder.internal.assert(strcmp(dataFormat,'struct'),...
                                  'ros:mlroscpp:codegen:InvalidDataFormat','rossubscriber');
            bufferSize = coder.internal.getParameterValue(pStruct.BufferSize,1,varargin{indx:end});
            validateattributes(bufferSize,{'numeric'},...
                               {'scalar','nonempty','integer','positive'},'Subscriber','BufferSize');

            % Store inputs
            obj.TopicName = topic;
            obj.MessageType = messageType;
            obj.BufferSize = bufferSize;
            obj.DataFormat = dataFormat;
            obj.MessageCount = 0;

            % Get and register code generation information
            cgInfo = coder.const(@ros.codertarget.internal.getCodegenInfo,topic,messageType,'sub');
            msgStructGenFcn = str2func(cgInfo.MsgStructGen);
            obj.MsgStruct = msgStructGenFcn();  % Setup return type

            %% Create an instance of MATLABSubscriber object
            % template <class MsgType, class StructType>
            %MATLABSubscriber(const StructType* structPtr, std::function<void(void)> callback)
            templateTypeStr = ['MATLABSubscriber<' cgInfo.MsgClass ',' cgInfo.MsgStructGen '_T>'];
            coder.ceval('auto structPtr = ', coder.wref(obj.MsgStruct));
            obj.SubscriberHelper = coder.opaque(['std::unique_ptr<', templateTypeStr, '>'],...
                                                'HeaderFile', 'mlroscpp_sub.h');
            if ros.internal.codegen.isCppPreserveClasses
                % Create subscriber by passing in class method as callback
                obj.SubscriberHelper = coder.ceval(['std::unique_ptr<', templateTypeStr, ...
                                                    '>(new ', templateTypeStr, '(structPtr, [this]{this->callback();}));//']);
            else
                % Create subscriber by passing in static function as
                % callback
                obj.SubscriberHelper = coder.ceval(['std::unique_ptr<', templateTypeStr, ...
                                                    '>(new ', templateTypeStr, '(structPtr, [obj]{Subscriber_callback(obj);}));//']);
            end
            coder.ceval('MATLABSUBSCRIBER_createSubscriber',obj.SubscriberHelper,...
                        coder.rref(obj.TopicName),size(obj.TopicName,2),obj.BufferSize);

            % Ensure callback is not optimized away by making an explicit
            % call here
            obj.callback();
            obj.IsInitialized = true;
        end

        %% Called from MATLABSubscriber class
        function callback(obj)
            coder.inline('never')
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
            coder.inline('never');
            messageCount = obj.MessageCount;
        end

        function [receivedMsg, status, statusText] = receive(obj, timeout)
        %RECEIVE Block until data is received
        %   MSG = RECEIVE(sub) blocks MATLAB from running the current program
        %   until the specified subscriber SUB receives a topic message
        %   and returns it in MSG. Press Ctrl+C to abort the wait.
        %
        %   MSG = RECEIVE(sub, TIMEOUT) specifies a TIMEOUT period, in
        %   seconds. If the subscriber does not receive a topic message
        %   and the timeout period elapses, RECEIVE displays an error
        %   message and lets MATLAB continue running the current program.
        %
        %   [MSG, STATUS, STATUSTEXT] = receive(____) returns the final receive status
        %   and the associated status text using any of the previous syntaxes.
        %   The STATUS indicates if the message has been received successfully or not and
        %   the associated STATUSTEXT returns "success" or "timeout".
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
        %      laser = ros.Subscriber(node, '/scan', 'sensor_msgs/LaserScan');
        %      scan = RECEIVE(laser);
        %
        %      % Receive data with 2 second timeout
        %      scan = RECEIVE(laser, 2)

        % Track current number of messages received
            coder.inline('never');
            nMessages = obj.MessageCount;
            statusText = 'unknown';

            % Warning if no status output
            if nargout<2
                coder.internal.compileWarning('ros:mlroscpp:codegen:MissingStatusOutput','receive');
            end

            if (nargin > 1) && isfinite(timeout)
                validateattributes(timeout,{'double','scalar'},{'nonnegative','finite'},...
                                   'receive','timeout');
                tDur = coder.opaque('::ros::Duration','HeaderFile','ros/duration.h');
                tDur = coder.ceval('tDur.fromSec',timeout);
                tStop = coder.opaque('::ros::Time','HeaderFile','ros/time.h');
                tStop = coder.ceval('::ros::Time::now() + ',tDur);
                status = true;
                while obj.MessageCount == nMessages
                    %Initialize variable currentTime as a required by coder
                    currentTime = tStop;
                    currentTime = coder.ceval('::ros::Time::now');
                    if currentTime >= tStop
                        status = false;
                        break;
                    end
                end
            else
                % timeout == inf
                while obj.MessageCount == nMessages
                    % Avoid being optimized away
                    coder.ceval('::ros::Time::now');
                end
                status = true;
            end

            coder.ceval('getStatusText',status, coder.wref(statusText));
            statusIndicator = status;
            if ~statusIndicator && nargout<2
                coder.internal.error('ros:mlros:subscriber:WaitTimeout');
            end
            receivedMsg = obj.LatestMessage;
        end

        function msgFromSub = rosmessage(obj)
        % ROS2MESSAGE Create an empty message structure based on the topic type
        %   MSG = ROS2MESSAGE(SUB) creates and returns an empty message
        %   structure MSG. The message type of MSG is determined by the
        %   topic this subscriber SUB is connected to.
        %
        %   Example:
        %
        %      % Create a ROS node
        %      node = ros.Node("/node_1");
        %
        %      % Create subscriber and message
        %      laserSub = ros.Subscriber(node,"/scan","sensor_msgs/LaserScan");
        %      msg = rosmessage(laserSub);
        %
        %   See also RECEIVE.

            coder.inline('never');
            msgFromSub = rosmessage(obj.MessageType,'DataFormat','struct');
        end

        function lastSubMsg = get.LatestMessage(obj)
            coder.inline('never')
            % MsgStruct is updated by MATLABSubscriber whenever a new
            % message is received. We need to syncronize access here so
            % that MsgStruct is not read while being written by
            % MATLABSubscriber as part of msg2struct call.
            coder.ceval('MATLABSUBSCRIBER_lock',obj.SubscriberHelper)
            lastSubMsg = obj.MsgStruct;
            coder.ceval('MATLABSUBSCRIBER_unlock',obj.SubscriberHelper)
        end
    end

    methods (Access = protected)
        function subInfo = getServerInfo(~)  %#ok<STOUT>
        %getServerInfo Get subscriber properties from node server
            if ~isempty(coder.target)
                coder.internal.assert(false,...
                                      'ros:mlroscpp:codegen:UnsupportedMethodCodegen','getServerInfo');
            end
        end
    end

    methods (Static)
        function props = matlabCodegenNontunableProperties(~)
            props = {'MessageType'};
        end

        function ret = getDescriptiveName(~)
            ret = 'ROS Subscriber';
        end

        function ret = isSupportedContext(bldCtx)
            ret = bldCtx.isCodeGenTarget('rtw');
        end

        function updateBuildInfo(buildInfo,bldCtx)
            if bldCtx.isCodeGenTarget('rtw')
                srcFolder = ros.slros.internal.cgen.Constants.PredefinedCode.Location;
                addIncludeFiles(buildInfo,'mlroscpp_sub.h',srcFolder);
                addSourceFiles(buildInfo,'mlroscpp_sub.cpp',srcFolder);
            end
        end
    end

    methods (Static, Access = ?ros.internal.mixin.ROSInternalAccess)
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
