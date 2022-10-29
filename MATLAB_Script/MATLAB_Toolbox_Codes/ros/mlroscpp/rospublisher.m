function [pub, msg] = rospublisher(topic, varargin)
%ROSPUBLISHER Publish messages on a topic
%   PUB = ROSPUBLISHER(TOPIC) creates a publisher, PUB, for a topic
%   with name TOPIC that already exists on the ROS master topic list. The
%   publisher gets the topic message type from the topic list on the ROS
%   master. If the topic is not on the ROS master topic list, this function
%   displays an error message. TOPIC is a string scalar.
%
%   PUB = ROSPUBLISHER(TOPIC,TYPE) creates a publisher for a topic
%   and adds that topic to the ROS master topic list. If the ROS master
%   topic list already contains a matching topic, the ROS master adds
%   the MATLAB global node to the list of publishers for that topic.
%   If the message type differs from the topic on ROS master topic list,
%   the function displays an error. TYPE is a string scalar.
%
%   PUB = ROSPUBLISHER(___,Name,Value) provides additional options
%   specified by one or more Name,Value pair arguments. You can specify
%   several name-value pair arguments in any order as
%   Name1,Value1,...,NameN,ValueN:
%
%      "DataFormat" - Determines format of ROS message to be used by
%                     the publisher, and returned from rosmessage.
%                     The "struct" format is typically faster to use, but
%                     does not validate message field data when set.
%                     Options are:
%                        "object" - Message object of the specified type
%                        "struct" - Message struct with compatible fields
%                     Default: "object"
%
%      "IsLatching" - If enabled, latch mode saves the last message
%                     sent by the publisher and re-sends it to new
%                     subscribers. By default, latch mode is enabled.
%                     To disable latch mode, set Value to false.
%                     Default: true
%
%   [PUB,MSG] = ROSPUBLISHER(___) returns a message MSG that can be sent
%   with the publisher PUB. The message will be initialized with default
%   values. The message type and format will be determined by the TYPE and
%   DataFormat inputs to ROSPUBLISHER.
%
%   ROSPUBLISHER(TOPIC,MSG) publishes a message, MSG, to the specified
%   topic without returning a publisher.
%
%   Use ROSPUBLISHER to publish messages on a topic. When MATLAB's
%   global node publishes messages on that topic, ROS nodes that
%   subscribe to that topic receive those messages.
%
%
%   Example:
%
%      % Create the ROS master
%      rosinit
%
%      % Create publisher and send string data
%      % Use struct message format for better performance
%      chatPub = ROSPUBLISHER("/chatter","std_msgs/String",...
%          "DataFormat","struct");
%      msg = rosmessage(chatPub);
%      msg.Data = 'Some test string';
%      send(chatPub,msg);
%
%      % Create another publisher on the same topic
%      % Message type will be inferred, disable latching
%      latchPub = ROSPUBLISHER("/chatter","IsLatching",false,...
%          "DataFormat","struct");
%      send(latchPub,msg);
%
%      % Create publisher that uses message objects
%      posePub = ROSPUBLISHER("/location","geometry_msgs/Pose2D",...
%          "DataFormat","object");
%      msgObj = rosmessage(posePub); % Uses DataFormat from publisher
%      msgObj.X = 1;
%      send(posePub,msgObj)
%
%      % Convenience function: just send one message
%      ROSPUBLISHER("/chatter",msg);
%
%   See also ros.Publisher, ROSMESSAGE, ROSTOPIC.

%   Copyright 2020 The MathWorks, Inc.
%#codegen

    if isempty(coder.target)
        try
            % Handle direct publishing quickly
            if nargin == 2 && (isa(varargin{1}, 'ros.Message') || ...
                               isstruct(varargin{1}))
                rospublisherDirect(topic, varargin{1});
                pub = [];
                return;
            end

            % Create and return publisher object
            pub = ros.Publisher([], topic, varargin{:});

            % Assign output message, if requested
            if nargout > 1
                msg = rosmessage(pub);
            end
        catch ex
            % Save stack traces and exception causes internally, but do not
            % print them to the console
            rosex = ros.internal.ROSException.fromException(ex);
            throwAsCaller(rosex);
        end
    else
        coder.internal.narginchk(1,8,nargin);
        pub = ros.Publisher([],topic,varargin{:});

        % Assign output message, if requested
        if nargout > 1
            msg = rosmessage(pub);
        end
    end
end

function rospublisherDirect(topic, msg)
%rospublisherDirect Directly publish to a topic.

% Make the publisher persistent to allow messages to be
% delivered before destruction
    persistent pubTmp;

    % Publish message directly
    pubTmp = []; %#ok<NASGU>
    dataFormat = 'object';
    if isstruct(msg)
        dataFormat = 'struct';
    end
    pubTmp = ros.Publisher([], topic, msg.MessageType, 'DataFormat', dataFormat);
    send(pubTmp, msg);
end
