function rosinit(varargin)
%ROSINIT Initialize the global node and ROS master
%   ROSINIT starts the global ROS node with a default MATLAB name, and
%   tries to connect to a ROS master running on 'localhost' and port 11311.
%   If the global ROS node cannot connect to the master, ROSINIT will start
%   its own ROS master, a ROS parameter server, and a rosout logging
%   node. These three entities are collectively referred to as ROS core.
%
%   ROSINIT('HOST') tries to connect to the ROS master at the hostname
%   or IP address specified by HOST. This syntax uses 11311 as the
%   default port number.
%
%   ROSINIT('HOST', PORT) tries to connect to the ROS master at the hostname
%   or IP address specified by 'HOST'. The connection will be established on
%   the port number specified in PORT.
%
%   ROSINIT('URI') tries to connect to the ROS master at the given URI
%   (Uniform Resource Identifier).
%
%   ROSINIT(___, Name, Value) provides additional options
%   specified by one or more Name,Value pair arguments. Name must appear
%   inside single quotes (''). You can specify several name-value pair
%   arguments in any order as Name1,Value1,...,NameN,ValueN:
%
%      'NodeHost'     -   specifies the IP address or hostname under
%                         which the node advertises itself to the
%                         ROS network, for example '192.168.1.1' or
%                         'comp-home'
%      'NodeName'     -   The name of the global node as string, for
%                         example '/test_node'
%
%   Using ROSINIT is prerequisite for most ROS-related tasks in MATLAB
%   because:
%   - Communicating with a ROS network requires a ROS node connected to
%     a ROS master.
%   - By default, ROS functions in MATLAB operate upon the global ROS
%     node, or operate upon objects that depend upon the global ROS node.
%
%   For example, after creating a global ROS node with ROSINIT, you can
%   subscribe the global ROS node to a topic. When another node on the
%   ROS network publishes messages on that topic, the global ROS node
%   receives the messages.
%
%   If the global ROS node already exists, you have to shut it down using
%   "rosshutdown" before you can call ROSINIT again.
%   This prevents an accidental shutdown of the ROS network.
%   To start additional nodes, use the ros.Node class.
%
%
%   Example:
%      % Start a ROS master and the global node
%      ROSINIT
%
%      % Start the global node and connect to the ROS master at IP address 192.168.1.10
%      ROSINIT('192.168.1.10')
%
%      % Start the global node and connect to the ROS master running on hostname
%      % 'master_comp' and port 13000
%      ROSINIT('master_comp', 13000);
%
%      % Start the global node and try to connect to the
%      % master at URI 'http://192.168.1.33:11311'
%      ROSINIT('http://192.168.1.33:11311');
%
%      % Start the global node and make sure it advertises itself with
%      % the given IP '192.168.1.1'. Also give the global node the name '/test_node'.
%      ROSINIT('http://192.168.1.33:11311', ...
%          'NodeHost', '192.168.1.1', ...
%          'NodeName', '/test_node');
%
%   See also ROSSHUTDOWN.

%   Copyright 2020 The MathWorks, Inc.

    if nargin > 0
        [varargin{:}] = convertStringsToChars(varargin{:});
    end

    try
        % Enforce that the user has to call rosshutdown if the global node
        % has already been initialized.
        if ros.internal.Global.isNodeActive
            globalNode = ros.internal.Global.getNodeHandle(false);
            error(message('ros:mlros:node:GlobalNodeRunningError', ...
                          globalNode.MasterURI));
        end

        % Parse input arguments
        args = ros.Node.parseArguments(varargin{:});

        % Check if we should start the global core
        args.MasterURI = initGlobalCore(args.MasterURI);

        % Initialize the global node
        % Temporarily disable warnings, since they should have been issued
        % already during argument parsing above
        [warn1, warn2] = disableNodeHostWarnings;
        ros.internal.Global.node('init', args.NodeName, args.MasterURI, args.NodeHost);
        restoreNodeHostWarnings(warn1, warn2);
    catch ex
        % Save stack traces and exception causes internally, but do not
        % print them to the console
        rosex = ros.internal.ROSException.fromException(ex);
        throwAsCaller(rosex);
    end

end

function masterURI = initGlobalCore(masterURI)
%initGlobalCore Handles the starting and stopping of the global core

% Check external settings for Master URI and default core port
% Never start the global core if an external master address is
% specified.
    if ~isempty(getenv('ROS_DEFAULT_NODE_PORT')) && ros.internal.getDefaultCorePort == 0

        localMasterURI = ['http://localhost:',num2str(getenv('ROS_DEFAULT_NODE_PORT'))];
    else
        localMasterURI = ['http://localhost:' num2str(ros.internal.getDefaultCorePort)];
    end

    if ~strcmp(masterURI, localMasterURI)
        % If the global core was running already and we connect to an
        % external instance, shut down the local core.
        ros.internal.Global.core('clear');
        return;
    end

    % Check if a master already exists there
    masterReachable = ...
        ros.internal.NetworkIntrospection.isMasterReachable(masterURI);

    if ~masterReachable
        % Start the core for the user
        masterURI = ros.internal.Global.core('init', []);
    end

end

function [hostResWarn, hostIPWarn] = disableNodeHostWarnings
%disableNodeHostWarnings Disable warnings for NodeHost settings

% Disable Warnings and save previous state
    hostResWarn = warning('off', 'ros:mlros:util:NodeHostResolutionWarning');
    hostIPWarn = warning('off', 'ros:mlros:util:NodeHostIPWarning');
end

function restoreNodeHostWarnings(hostResWarn, hostIPWarn)
%restoreNodeHostWarnings Restore the original state of the warnings

    warning(hostResWarn.state, 'ros:mlros:util:NodeHostResolutionWarning');
    warning(hostIPWarn.state, 'ros:mlros:util:NodeHostIPWarning');
end
