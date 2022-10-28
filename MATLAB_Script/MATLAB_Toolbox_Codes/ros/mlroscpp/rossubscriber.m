function sub = rossubscriber(topic, varargin)
%ROSSUBSCRIBER Subscribe to messages on a topic
%   SUB = ROSSUBSCRIBER(TOPIC) subscribes to a topic with name
%   TOPICNAME. If the ROS master topic list includes TOPIC, this
%   syntax returns a subscriber object handle, SUB. If the ROS master
%   topic list does not include the topic, this syntax displays an error.
%   TOPIC is a string scalar.
%
%   SUB = ROSSUBSCRIBER(TOPIC,TYPE) subscribes to a topic
%   that has the specified name TOPIC and type TYPE. This syntax
%   returns a subscriber object handle. If the topic list on ROS master
%   does not include a topic with the specified name and type, a topic
%   with the specific name and type is added to the topic list.
%   Use this syntax to avoid errors when it is possible for the subscriber
%   to subscribe to a topic before a publisher has added the topic to
%   the topic list on the ROS master. TYPE is a string scalar.
%
%   SUB = ROSSUBSCRIBER(TOPIC,CB) specifies a callback function CB,
%   and optional data, to run when the subscriber object handle receives
%   a topic message. Use this syntax to avoid blocking wait functions.
%   CB can be a single function handle or a cell array. The
%   first element of the cell array needs to be a function handle or a
%   string containing the name of a function. The remaining elements of
%   the cell array can be arbitrary user data that will be passed to
%   the callback function.
%
%   SUB = ROSSUBSCRIBER(TOPIC,TYPE,CB) specifies a
%   callback function CB, and subscribes to a topic that has the
%   specified name TOPIC and type TYPE.
%
%   SUB = ROSSUBSCRIBER(__,Name,Value) provides additional options
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
%   ROSSUBSCRIBER allows you to transfer data by subscribing to messages.
%   When ROS nodes publish messages on that topic, MATLAB will receive
%   those messages through this subscriber.
%
%   The subscriber callback function requires at least two input
%   arguments. The first argument, SRC, is the associated subscriber object.
%   The second argument is the received message, MSG. The function
%   header for the callback is as follows:
%
%      function subCallback(SRC,MSG)
%
%   You pass additional parameters to the callback function by including
%   both the callback function and the parameters as elements of a cell
%   array when setting the callback.
%
%
%   Example:
%
%      % Create the ROS master
%      rosinit
%
%      % Create subscriber for lidar data
%      % Use struct message format for better performance
%      laserSub = ROSSUBSCRIBER("/scan","sensor_msgs/LaserScan",...
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
%      chatPub = rospublisher("/chatter","std_msgs/String",...
%          "DataFormat","object");
%      chatSub = ROSSUBSCRIBER("/chatter",@testCallback,...
%          "DataFormat","object");
%
%      % Create subscriber with buffer size of 5
%      bufferSub = ROSSUBSCRIBER("/chatter","BufferSize",5);
%
%   See also ros.Subscriber, ROSMESSAGE, ROSTOPIC.

%   Copyright 2014-2020 The MathWorks, Inc.
%#codegen

    if isempty(coder.target)
        try
            sub = ros.Subscriber([], topic, varargin{:});
        catch ex
            % Save stack traces and exception causes internally, but do not
            % print them to the console
            rosex = ros.internal.ROSException.fromException(ex);
            throwAsCaller(rosex);
        end
    else
        coder.internal.narginchk(1, 7, nargin)
        sub = ros.Subscriber([], topic, varargin{:});
    end
end
