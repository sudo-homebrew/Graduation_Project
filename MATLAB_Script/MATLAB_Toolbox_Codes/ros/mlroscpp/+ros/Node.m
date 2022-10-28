classdef Node < robotics.core.internal.mixin.Unsaveable & handle
%Node Launch a ROS node and connect to the specified ROS master.
%   This class represents a ROS node and allows you to communicate with
%   the rest of the ROS network. You have to create a node before you
%   can use other functionality like publishers, subscribers, and
%   services.
%
%   N = ros.Node("NAME") will initialize the ROS node with the given NAME.
%   Try to connect to the master at default URI http://localhost:11311,
%   unless otherwise specified by the ROS_MASTER_URI environment variable.
%
%   N = ros.Node("NAME","URI") will initialize the ROS node with NAME and try
%   to connect to the ROS master at URI.
%
%   N = ros.Node("NAME","HOST") will try to connect to the ROS master at hostname
%   or IP address HOST. This syntax will use the ROS default port 11311.
%
%   N = ros.Node("NAME","HOST",PORT) will try to connect to the ROS master at hostname
%   or IP address HOST and port number PORT.
%
%   N = ros.Node(___,Name,Value) allows the specification of optional
%   name/value pairs to control the creation of the node.
%   Potential name/value pairs are:
%      'NodeHost'     -   specifies the IP address or hostname under
%                         which the node advertises itself to the
%                         ROS network, e.g. '192.168.1.1' or
%                         'comp-home'
%
%
%   Node properties:
%      Name        - (Read-only) The name of the node
%      MasterURI   - (Read-only) The URI to connect to the ROS master
%      NodeURI     - (Read-only) The URI with which the node advertises itself
%      CurrentTime - (Read-only) The current ROS network time.
%
%   Node methods:
%      resolveName  - Check and resolve ROS name
%      delete       - Shut down node and attached ROS objects
%
%
%   Example:
%      % Create a ROS master on port 11311
%      master = ros.Core
%
%      % Initialize the named node '/test_node_1' and connect to the
%      % master in MATLAB (created in previous command)
%      node = ros.Node('/test_node_1');
%
%      % Create a separate node that connects to a different master
%      % running on hostname 'master_computer' and port 13000
%      externalNode = ros.Node('/test_node_2','master_computer',13000);
%
%      % Create a node that connects to the master at URI 'http://192.168.1.33:11311'
%      % and advertises itself on IP 192.168.1.1
%      uriNode = ros.Node('/test_node_3','http://192.168.1.33:11311', ...
%          'NodeHost','192.168.1.1');
%
%   See also ROSINIT.

%   Copyright 2020-2021 The MathWorks, Inc.

    properties (Dependent, SetAccess = private)
        %Name - Name of the node
        %   This needs to be unique in the ROS network
        Name

        %MasterURI - The URI to the ROS master
        %   The node is connected to the ROS master with the given URI.
        %   Default: http://localhost:11311
        MasterURI

        %NodeURI - The URI with which the node advertises itself
        %   Other nodes will try to reach this node through the
        %   NodeURI.
        NodeURI

        %CurrentTime - The current ROS network time
        %   If time is published on the /clock topic, returns the time
        %   according to the ROS clock. Otherwise, returns the current
        %   system time.
        %   The returned time can be used to timestamp messages or to
        %   measure time in the ROS network.
        CurrentTime
    end

    properties (Transient, Access = ?ros.internal.mixin.ROSInternalAccess)
        %InternalNode - Internal representation of the node object
        InternalNode = []

        %ServerNodeHandle - Designation of the node on the server
        ServerNodeHandle = []

        %ServerPath - Path to the out-of-process server executable
        ServerPath

        %LogPath - Path to store log files
        LogPath

        %ServerStartPath - Path in which to start the server
        ServerStartPath

        %ServerMode - Indicates if the server uses ROS or "echo" mode
        %   0 - Communication over ROS possible
        %   1 - Communication only reflected back by server
        ServerMode

        %LogLevel - Indicates types of logs created during operations
        %   0 - Trace (most logging)
        %   1 - Debug
        %   2 - Info
        %   3 - Warning
        %   4 - Error
        %   5 - Fatal (least logging)
        LogLevel

        %ListofNodeDependentHandles - List of weak handles to objects attached to the node
        ListofNodeDependentHandles =  {}
    end

    properties (Constant, Access = ?ros.internal.mixin.ROSInternalAccess)
        %DefaultServerMode - Default to using ROS network
        DefaultServerMode = 0

        %DefaultLogLevel - Default to logging only fatal exceptions
        DefaultLogLevel = 5

        %DefaultLogPath - Default to logging in temporary directory
        DefaultLogPath = fullfile(tempdir, 'ros1_log');

        %DebugMode - Do not debug in release
        DebugMode = false

        %MasterConnectionTimeout - Timeout (in secs) for master connection
        %   Pick a longer time than the default to avoid sporadic failures.
        MasterConnectionTimeout = 2
    end

    methods
        function obj = Node(name, varargin)
        %Node - Create a ROS node object
        %   The 'name' argument is required and specifies a node name that
        %   is unique in the ROS network. The rest of the arguments specify
        %   several parameter values. Please see the class documentation
        %   (help ros.Node) for more details.

        % Parse node name input
            name = convertStringsToChars(name);
            validateattributes(name, {'char', 'string'}, ...
                               {'nonempty', 'scalartext'}, ...
                               'Node', 'name')
            resolvedName = ros.internal.Namespace.resolveNodeName(name);

            % Parse other input arguments, if they exist
            args = ros.Node.parseArguments(varargin{:});
            masterURI = args.MasterURI;
            nodeHost = args.NodeHost;
            obj.ServerMode = args.ServerMode;
            obj.LogLevel = args.LogLevel;
            obj.LogPath = args.LogPath;

            % Create node
            setupPaths(obj);
            createNode(obj, resolvedName, masterURI, nodeHost);
        end

        function delete(obj)
        %delete Remove reference to ROS node
        %   delete(NODE) removes the reference in NODE to the ROS node on
        %   the network. If no further references to the node exist, such
        %   as would be in publishers and subscribers, the node is shut
        %   down.

            try
                if(numel(obj.ListofNodeDependentHandles) > 0)
                    for i = 1:numel(obj.ListofNodeDependentHandles)
                        if ~isempty(obj.ListofNodeDependentHandles{i})
                            delete(obj.ListofNodeDependentHandles{i}.get());
                        end
                    end
                end
                % Delete reference to node, without deleting internal node
                obj.InternalNode = [];
            catch ex
                warning(message('ros:mlroscpp:node:ShutdownError', ...
                                ex.message))
            end
        end

        function rosName = resolveName(obj, name)
        %resolveName Check and resolve ROS name
        %   ROSNAME = resolveName(OBJ, NAME) checks and resolves the ROS
        %   name, NAME, that is associated with a particular node
        %   object, OBJ. This function returns a valid and properly
        %   namespaced ROS name in ROSNAME.
        %
        %   This function checks the validity of the given ROS name by
        %   comparing it to the ROS naming standard. If the name does not
        %   conform, an error will be displayed.
        %
        %   This function will also take any name spacing into account if the name
        %   is defined as relative name without a leading slash. A relative
        %   name, e.g. 'testName', will be resolved relative to the root
        %   namespace. A private name, e.g. '~testName' will be resolved
        %   relative to the node namespace. A leading forward slash
        %   indicates a global name, e.g. '/testName'.
        %
        %   A full remapping of all relative ROS names within a node (including the
        %   node name) can be achieved by setting the ROS_NAMESPACE
        %   environment variable, e.g. in the example
        %   above if ROS_NAMESPACE=/global, then the resolved name of
        %   testName would be /global/testName

            try
                validName = ros.internal.Namespace.canonicalizeName(name);
                validName = resolvename(obj.InternalNode, validName);
                rosName = char(validName);
            catch ex
                newEx = ros.internal.ROSException(message('ros:mlros:util:ResolveError', name));
                throw(newEx.addCustomCause(ex));
            end
        end
    end

    % All dependent properties are read from the server
    methods
        function name = get.Name(obj)
        %get.Name Custom getter for Name property

        % Allow errors to be thrown from getServerInfo
            nodeInfo = getServerInfo(obj);
            name = nodeInfo.name;
        end

        function nodeURI = get.NodeURI(obj)
        %get.NodeURI Retrieve the URI that the node advertises
        %   Other nodes will try to reach this node through the
        %   NodeURI.

            nodeInfo = getServerInfo(obj);
            nodeURI = nodeInfo.nodeuri;
        end

        function masterURI = get.MasterURI(obj)
            nodeInfo = getServerInfo(obj);
            masterURI = nodeInfo.masteruri;
        end

        function currentTime = get.CurrentTime(obj)
            t = getCurrentTime(obj);
            currentTime = ros.msg.Time(t);
        end
    end
    methods (Static)
        function args = parseArguments(varargin)
        %parseNodeArguments Parse the inputs to the node
        %   ARGS = parseArguments(VARARGIN) parses the
        %   variable list of arguments and returns a structure ARGS
        %   that contains the following elements:
        %   ARGS.NodeName  -  The name for the created node
        %   ARGS.NodeHost  -  The hostname or IP for advertising
        %   ARGS.MasterURI -  The Master URI to connect to.
        %
        %   These inputs could be passed from rosinit or through the
        %   ros.Node constructor.

        % Initialize default values
            args = struct('NodeName', '', ...
                          'NodeHost', '', ...
                          'MasterURI','', ...
                          'ServerMode', ros.Node.DefaultServerMode, ...
                          'LogLevel', ros.Node.DefaultLogLevel, ...
                          'LogPath', ros.Node.DefaultLogPath);

            % Return default values if no input arguments are provided
            if nargin == 0
                args.MasterURI = ros.internal.Net.generateMasterURIforNode;
                args.NodeHost = ros.internal.Net.generateNodeHost(args.MasterURI);
                return
            end

            [varargin{:}] = convertStringsToChars(varargin{:});

            % Parse the name-value pairs first
            nvPairsStart = ros.internal.Parsing.findNameValueIndex(...
                varargin, fieldnames(args));

            if ~isempty(nvPairsStart)
                % Create input parser for name-value pairs
                parser = inputParser;
                addParameter(parser, 'NodeHost', args.NodeHost, ...
                             @(x) validateattributes(x, ...
                                                     {'char', 'string'}, ...
                                                     {'scalartext'}, ...
                                                     'Node', ...
                                                     'NodeHost'));
                addParameter(parser, 'NodeName', args.NodeName, ...
                             @(x) validateattributes(x, ...
                                                     {'char', 'string'}, ...
                                                     {'nonempty', 'scalartext'}, ...
                                                     'Node', ...
                                                     'NodeName'));
                addParameter(parser, 'ServerMode', args.ServerMode, ...
                             @(x) validateattributes(x, ...
                                                     {'numeric', 'logical'}, ...
                                                     {'scalar', '>=', 0, '<=' 2}, ...
                                                     'Node', ...
                                                     'ServerMode'))
                addParameter(parser, 'LogLevel', args.LogLevel, ...
                             @(x) validateattributes(x, ...
                                                     {'numeric'}, ...
                                                     {'scalar', '>=', 0, '<=' 5}, ...
                                                     'Node', ...
                                                     'LogLevel'))
                addParameter(parser, 'LogPath', args.LogPath, ...
                             @(x) validateattributes(x, ...
                                                     {'char', 'string'}, ...
                                                     {'nonempty', 'scalartext'}, ...
                                                     'Node', ...
                                                     'LogPath'))

                % Parse and extract the input
                parse(parser, varargin{nvPairsStart:end})
                args = parser.Results;
                varargin(nvPairsStart:end) = [];
            end

            % Parse the remaining arguments
            switch length(varargin)
              case 0
                % Arguments were only name-value pairs
                % Default the other values
                host = []; port = [];

              case 1
                % Argument could be of the form ('URI') or ('HOST')
                host = varargin{1};
                port = [];

              case 2
                % Arguments could be of the form ('HOST', PORT)
                host = varargin{1};
                port = varargin{2};

              otherwise
                error(message('ros:mlros:node:RosInitWrongNumberArguments'));
            end

            if ~isempty(host)
                validateattributes(host, {'char', 'string'}, ...
                                   {'nonempty', 'scalartext'}, ...
                                   'Node', 'host')
            end
            if ~isempty(port)
                validateattributes(port, {'numeric'}, ...
                                   {'scalar', 'nonempty', 'integer', 'positive'}, ...
                                   'Node', 'port');
            end
            args.MasterURI = ros.internal.Net.generateMasterURIforNode(host, port);
            args.NodeHost = ros.internal.Net.generateNodeHost(args.MasterURI, args.NodeHost);
        end
    end

    methods (Access = private)
        function setupPaths(obj)
        %setupPaths Set paths for out-of-process server, logging, and
        %   starting directory

        % System architecture keys
            archKeys = {'win64', 'glnxa64', 'maci64'};
            arch = computer('arch');

            % MCR separates toolbox MATLAB files from library files
            % Use matlabroot to find correct path even in compiled code
            mlRoot = matlabroot;

            % Server executable
            serverPathBase = ...
                fullfile(mlRoot, 'toolbox', 'ros', 'bin', arch);
            serverPathMap = ...
                containers.Map(archKeys, ...
                               {'libmwros1server.exe', ...   % win64
                                'libmwros1server', ...       % glnxa64
                                'libmwros1server'});         % maci64
            obj.ServerPath = fullfile(serverPathBase, serverPathMap(arch));

            % Server start directory suggestion
            startPathBase = fullfile(mlRoot, 'sys', 'ros1', ...
                                     arch, 'ros1');
            startPathMap = ...
                containers.Map(archKeys, ...
                               {'bin', ...    % win64
                                'lib', ...    % glnxa64
                                'lib'});      % maci64
            consolebridgePathBase = fullfile(startPathBase,'console_bridge',startPathMap(arch));

            obj.ServerStartPath = fullfile(startPathBase, ...
                                           startPathMap(arch));

            obj.ServerStartPath = [obj.ServerStartPath pathsep consolebridgePathBase];
        end

        function createNode(obj, name, masteruri, nodehost)
        %createNode Create node on ROS network

            try
                % Split fully qualified name into namespace and base name
                [baseName, namespace] = nodeNameParts(name);

                % Create internal node representation
                obj.InternalNode = ros.internal.Node;

                % Set path correctly and remove it after node creation
                cleanPath = ros.internal.setupRosEnv(); %#ok<NASGU>

                % Start node on ROS network
                returnCall = create(obj.InternalNode, baseName, ...
                                    [namespace ';' masteruri ';' nodehost], obj.ServerPath, ...
                                    obj.ServerStartPath, obj.ServerMode, ...
                                    obj.LogLevel, obj.LogPath, ...
                                    obj.DebugMode);

                % Check output and error if node not created
                if isempty(returnCall) || ~isstruct(returnCall)
                    error(message('ros:mlroscpp:node:InvalidReturnCallError'))
                elseif ~isfield(returnCall, 'handle') || ...
                        isempty(returnCall.handle)
                    error(message('ros:mlroscpp:node:InvalidReturnCallHandleError'))
                end
                obj.ServerNodeHandle = returnCall.handle;
            catch ex
                if strcmp(ex.identifier, ...
                          'ros:internal:transport:ServerNotUpError')
                    error(message('ros:mlroscpp:node:CreationServerError', ...
                                  name));
                elseif strcmp(ex.identifier, ...
                              'ros:internal:transport:NodeAlreadyExists')
                    error(message('ros:mlros:node:NodeNameExists', name, masteruri));
                elseif strcmp(ex.identifier, ...
                              'ros:internal:transport:MasterNotReachableError')
                    error(message('ros:mlros:node:NoMasterConnection', masteruri));
                else
                    newEx = MException(message('ros:mlroscpp:node:CreationGenericError', ...
                                               name));
                    throw(newEx.addCause(ex));
                end
            end
        end

        function nodeInfo = getServerInfo(obj)
        %getServerInfo Get node properties from server

        % Ensure properties are valid
            if isempty(obj.InternalNode) || ~isvalid(obj.InternalNode)
                error(message('ros:mlroscpp:node:InvalidInternalNodeError'))
            elseif isempty(obj.ServerNodeHandle)
                error(message('ros:mlroscpp:node:InvalidServerHandleError'))
            end

            % Extract node information
            try
                nodeInfo = nodeinfo(obj.InternalNode, ...
                                    obj.ServerNodeHandle, []);
            catch ex
                newEx = MException(message('ros:mlroscpp:node:GetInfoError'));
                throw(newEx.addCause(ex));
            end
        end

        function t = getCurrentTime(obj)% Ensure properties are valid
            if isempty(obj.InternalNode) || ~isvalid(obj.InternalNode)
                error(message('ros:mlroscpp:node:InvalidInternalNodeError'))
            elseif isempty(obj.ServerNodeHandle)
                error(message('ros:mlroscpp:node:InvalidServerHandleError'))
            end
            % Last argument is false meaning report simulation time.
            % Setting last parameter to true results in wall clock time
            % (system time) to be reported
            t = getCurrentTime(obj.InternalNode, ...
                               obj.ServerNodeHandle, false);
        end
    end
end

% Helper functions
function [baseName, namespace] = nodeNameParts(name)
%nodeNameParts Split name into namespace and node name
%   Requires fully qualified name as input
%   Assumes name is a character vector and non-empty

% Extract name as last text block separated by slash
    nameSplit = strsplit(name, '/', 'CollapseDelimiters', false);
    namespace = strjoin(nameSplit(1:end-1), '/');
    baseName = nameSplit{end};
end
