function output = rostopic( op, varargin )
%ROSTOPIC Display or return information about ROS topics
%   Use ROSTOPIC to get a list of ROS topics from the ROS master; or
%   to get the message type, publishers, and subscribers for a specific
%   topic.
%
%   LST = ROSTOPIC('list') returns a list of ROS topics from the ROS
%   master as a string cell array LST. If the output argument LST is
%   not defined, the list is printed to the screen. Simplified form:
%   ROSTOPIC list
%
%   MSG = ROSTOPIC('echo', 'TOPICNAME') prints the messages that are being
%   sent on the topic with name TOPICNAME in the MATLAB command window.
%   You can stop the printing of messages by pressing Ctrl+C.
%   If the output argument MSG is defined, the first message that
%   arrives on that topic will be returned in that variable and ROSTOPIC
%   will return. Simplified form: ROSTOPIC echo TOPICNAME
%
%   TOPICINFO = ROSTOPIC('info', 'TOPICNAME') returns the message type,
%   publishers, and subscribers for a specific topic with name
%   TOPICNAME as a structure. Simplified form: ROSTOPIC info TOPICNAME
%
%   TP = ROSTOPIC('type', 'TOPICNAME') returns the message type that for a
%   specific topic as a string. Simplified form: ROSTOPIC type TOPICNAME
%
%
%   Example:
%      % List all topics on the ROS master
%      ROSTOPIC list
%
%      % Get information about the /scan topic
%      ROSTOPIC info /scan
%
%      % Display messages that are sent on the topic /scan
%      ROSTOPIC echo /scan

%   Copyright 2020 The MathWorks, Inc.

    op = convertStringsToChars(op);
    if nargin > 1
        [varargin{:}] = convertStringsToChars(varargin{:});
    end

    try
        if nargout == 0
            rostopicImpl(op, varargin{:});
        else
            output = rostopicImpl(op, varargin{:});
        end
    catch ex
        % Save stack traces and exception causes internally, but do not
        % print them to the console
        rosex = ros.internal.ROSException.fromException(ex);
        throwAsCaller(rosex);
    end
end

function output = rostopicImpl(op, varargin)
%rostopicImpl Actual implementation of rostopic functionality.

% Use validatestring to ensure string and char inputs to 'operation' are
% supported.
    try
        supportedOperations = {'list', 'info', 'type','echo'};
        operation = validatestring(op, supportedOperations, 'rostopic', 'operation');
    catch
        error(message('ros:mlros:node:OperationNotSupported', op));
    end

    % Parse the inputs to the function
    persistent parser;
    if isempty(parser)
        parser = inputParser;
        addOptional(parser, 'arg', '/', @(x) validateattributes(x, {'char'}, {'nonempty'}, 'rostopic', 'arg'));
    end

    if isempty(varargin)
        parse(parser);
    else
        %Input parser only includes one optional argument
        parse(parser, varargin{1});
    end
    argument = parser.Results.arg;

    switch operation
      case 'list'
        % Display a list of topic names that are registered on the master.
        % Sort the list and then display it.
        topicNames = rostopicList;

        % If output argument specified, return
        if nargout == 1
            output = topicNames;
            return;
        end

        % Display the list
        for name = topicNames
            disp(char(name));
        end

      case 'info'
        % Get information about a specific topic
        topicName = argument;
        info = rostopicInfo(topicName);
        if nargout == 1
            output = info;
            return;
        end

        % If no output argument specified, print to terminal
        rostopicInfoPrint( info );

      case 'type'
        % Get the message type that is published for a specific topic
        topicName = argument;
        type = rostopicType( topicName );

        if nargout == 1
            output = type;
            return;
        end

        % If no output argument specified, print to terminal
        disp(type);

      case 'echo'
        % Print the data that is sent on a topic on the console
        topicName = argument;

        % Parse timeout (only used for testing)
        defaults.Timeout = Inf;
        args = parseEchoNameValuePairs(defaults, varargin{2:end});

        sub = rossubscriber(topicName, 'BufferSize', 1);
        if nargout == 1
            output = receive(sub, args.Timeout);
            clear('sub');
            return;
        end
        while(1)
            msg = receive(sub, args.Timeout);
            disp(ros.msg.internal.MessageDisplay.printData(msg));
            disp('---');
        end
    end

end

function list = rostopicList
%rostopicList Return a list of active topic names on the master
%   This function will return a LIST of all published topic names. Be
%   aware that a publisher and/or subscriber needs to exist for a
%   topic. Otherwise, it will not be included in the list.

    list = sort(ros.internal.NetworkIntrospection.getPublishedTopicNamesTypes(''));
end

function info = rostopicInfo( topicName )
%rostopicInfo Return information about a topic
%   Information about the topic TOPICNAME is retrieved and returned in
%   an information structure INFO. The following information is
%   retrieved: message type, publishers, and subscribers

    info = ros.internal.NetworkIntrospection.getPublishedTopicInfo(topicName, true);
end

function rostopicInfoPrint( info )
%rostopicInfoPrint Print info structure to the terminal

    line = [message('ros:mlros:topic:RostopicType').getString ': ' info.MessageType];
    disp(line);

    disp(' '); disp([message('ros:mlros:topic:RostopicPublishers').getString ':']);
    for j = 1:numel(info.Publishers)
        disp(['* ' info.Publishers(j).NodeName ' (' info.Publishers(j).URI ')']);
    end
    disp(' '); disp([message('ros:mlros:topic:RostopicSubscribers').getString ':']);
    for j = 1:numel(info.Subscribers)
        disp(['* ' info.Subscribers(j).NodeName ' (' info.Subscribers(j).URI ')']);
    end
end


function type = rostopicType( topicName )
%rostopicType Retrieve the message type that is sent on a topic

    type = ros.internal.NetworkIntrospection.getPublishedTopicType(topicName, true);
end

function args = parseEchoNameValuePairs(defaults, varargin)
%parseEchoNameValuePairs Parse any name/value pairs for "rostopic echo"

% Return the defaults if no extra arguments
    if numel(varargin) == 0
        args = defaults;
        return;
    end

    parser = inputParser;
    addParameter(parser, 'Timeout', defaults.Timeout, ...
                 @(x) validateattributes(x, {'numeric'}, {'nonempty','positive','scalar'}, 'rostopic echo', 'Timeout'));
    parse(parser, varargin{:});

    % Assign output arguments
    args.Timeout = parser.Results.Timeout;
end
