function output = rosnode(op, varargin)
%ROSNODE Get information about nodes in the ROS network
%   LS = ROSNODE('list') returns a list of ROS nodes on the ROS network
%   as a cell array of names. To appear on the list, a node must publish
%   or subscribe to one or more topics. Simplified form: ROSNODE list
%
%   NODEINF = ROSNODE('info', 'NODENAME') returns the name, URI,
%   publications, subscriptions, and services of a specific ROS node
%   with name NODENAME as a structure. Simplified form: ROSNODE info NODENAME
%
%   ROSNODE('ping', 'NODENAME') pings a specific node and displays the
%   response time. Simplified form: ROSNODE ping NODENAME
%
%   Use ROSNODE to get information about a node or nodes in the ROS network.
%
%
%   Example:
%      % Show the list of all nodes on the ROS network
%      ROSNODE list
%
%      % Ping node '/somenode'
%      ROSNODE ping /somenode
%
%      % Show information about node '/somenode'
%      ROSNODE info /somenode

%   Copyright 2020 The MathWorks, Inc.

    try
        if nargout == 0
            rosnodeImpl(op, varargin{:});
        else
            output = rosnodeImpl(op, varargin{:});
        end
    catch ex
        % Save stack traces and exception causes internally, but do not
        % print them to the console
        rosex = ros.internal.ROSException.fromException(ex);
        throwAsCaller(rosex);
    end
end

function output = rosnodeImpl(op, varargin)
%rosnodeImpl Actual implementation of rosnode functionality.

% Use validatestring to ensure string and char inputs to 'operation' are
% supported.
    try
        supportedOperations = {'ping', 'info', 'list'};
        operation = validatestring(op, supportedOperations, 'rosnode', 'operation');
    catch
        error(message('ros:mlros:node:OperationNotSupported', ...
                      op));
    end

    if nargin > 1
        % Convert strings to characters to ensure that, together with
        % validateattributes, "" is flagged as invalid input.
        [varargin{:}] = convertStringsToChars(varargin{:});
    end

    % Parse the input to the function
    parser = getParser;
    parse(parser, varargin{:});

    switch operation
      case 'ping'
        % Ping a node with a given name
        nodeName = parser.Results.name;
        rosnodePing(nodeName);

      case 'info'
        % Display detailed information about specific node
        nodeName = parser.Results.name;
        info = ros.internal.NetworkIntrospection.getNodeInfo(nodeName, []);

        if nargout == 1
            output = info;
            return;
        end

        % Display the info structure
        rosnodeInfoPrint( info );

      case 'list'
        % Display list of existing nodes in ROS network
        nodeNames = ros.internal.NetworkIntrospection.getNodeNames([]);
        if nargout == 1
            output = nodeNames;
            return;
        end

        % Only print to console if no output argument specified
        for n = nodeNames
            disp(char(n));
        end
    end

    function parser = getParser()
        persistent p

        if isempty(p)
            p = inputParser;
            addOptional(p, 'name', '', @(x) validateattributes(x, {'char','string'}, ...
                                                              {'nonempty'}, 'rosnode', 'name'));
        end
        parser = p;
    end

end

function rosnodeInfoPrint(info)
%rosnodeInfoPrint Print the node information structure to the terminal

    disp([message('ros:mlros:node:RosnodeNode').getString ': [' info.NodeName ']']);
    disp([message('ros:mlros:node:RosnodeURI').getString ': [' info.URI ']']);

    % Display publications (don't display topic type to avoid clutter)
    activePubs = size(info.Publications, 1);
    disp(' ');
    disp([message('ros:mlros:node:RosnodePubs').getString ' ('  ...
          num2str(activePubs) ' ' ...
          message('ros:mlros:node:RosnodeActiveTopics').getString '): ']);

    for i = 1:activePubs
        p = info.Publications(i);
        disp([' * ' p.TopicName]);
    end

    % Display subscriptions (don't display topic type to avoid clutter)
    activeSubs = size(info.Subscriptions, 1);
    disp(' ');
    disp([message('ros:mlros:node:RosnodeSubs').getString ' ('  ...
          num2str(activeSubs) ' ' ...
          message('ros:mlros:node:RosnodeActiveTopics').getString '): ']);

    for i = 1:activeSubs
        s = info.Subscriptions(i);
        disp([' * ' s.TopicName]);
    end

    % Display offered services
    activeSvcs = length(info.Services);
    disp(' ');
    disp([message('ros:mlros:node:RosnodeSvcs').getString ' ('  ...
          num2str(activeSvcs) ' ' ...
          message('ros:mlros:node:RosnodeActive').getString '): ']);

    for i = 1:activeSvcs
        disp([' * ' info.Services(i).Name]);
    end
end

function rosnodePing(nodeName)
%rosnodePing Ping a particular node

    nodeURI = ros.internal.NetworkIntrospection.getNodeURI(nodeName, []);
    if isempty(nodeURI)
        error(message('ros:mlros:node:NodeNotExist', nodeName));
    end

    disp(getString(message('ros:mlros:node:PingNode', nodeName, 3)));

    % Setup cleanup function
    c = onCleanup(@()cleanup);

    % Try to ping the node 4 times with a 3 second timeout
    cumulativeTime = 0;
    pingCount = 0;
    for i = 1:4
        [reachable, time] = ros.internal.NetworkIntrospection.isNodeReachable(nodeName, [], 3);

        if ~reachable
            disp(getString(message('ros:mlros:node:CannotPingNode')));
            pause(1);
            continue;
        end

        msTime = time * 1000;
        pingCount = pingCount + 1;
        cumulativeTime = cumulativeTime + msTime;

        disp(getString(message('ros:mlros:node:PingReply', ...
                               nodeURI, num2str(msTime, '%.3f'))));
        pause(1);
    end


    function cleanup
    %cleanup Is called when user presses Ctrl+C or if an error occurs
        averagePingTimeMs = cumulativeTime / pingCount;
        disp(getString(message('ros:mlros:node:PingAverage', ...
                               num2str(averagePingTimeMs, '%.3f'))));
    end
end
