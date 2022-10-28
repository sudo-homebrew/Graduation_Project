classdef ros2node < robotics.core.internal.mixin.Unsaveable & handle
%ros2node Create a ROS 2 node on the specified network
%   The ros2node object represents a ROS 2 node, and provides a foundation
%   for communication with the rest of the ROS 2 network.
%
%   N = ros2node("NAME") initializes the ROS 2 node with the given NAME.
%   The node connects with the default domain identification 0 unless
%   otherwise specified by the ROS_DOMAIN_ID environment variable.
%
%   The node uses default ROS Middleware implementation 'rmw_fastrtps_cpp'
%   unless otherwise specified by the RMW_IMPLEMENTATION environment variable.
%
%   N = ros2node("NAME",ID) initializes the ROS 2 node with NAME and
%   connects it to the network using the given domain ID. The acceptable
%   values for the domain identification are between 0 and 232.
%
%   Node properties:
%      Name         - (Read-only) Name of the node
%      ID           - (Read-only) Network domain identification
%
%   Node methods:
%      resolveName  - Check and resolve ROS 2 name
%      delete       - Shut down node and attached ROS 2 objects
%
%   Example:
%      % Initialize the node "/node_1" on the default network
%      node1 = ros2node("/node_1");
%
%      % Create a separate node that connects to a different network
%      % identified with domain 2
%      node2 = ros2node("/node_2",2);

%   Copyright 2019-2021 The MathWorks, Inc.

    properties (Dependent, SetAccess = private)
        %Name - Name of the node
        %   This needs to be unique in the ROS 2 network
        Name
    end

    properties (SetAccess = private)
        %ID - Domain identification of the network
        %   Default: 0
        ID
    end

    properties (Transient, Access = ?ros.internal.mixin.InternalAccess)
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

        %EnvApplicablePath - Environment variable set to use the server
        EnvApplicablePath

        %DataArrayLibPath - Path to MATLAB data array dynamic library
        DataArrayLibPath

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

        %RMWImplementation - To use the ROS Middleware Implementation based on DDS
        RMWImplementation
    end

    properties (Constant, Access = ?ros.internal.mixin.InternalAccess)
        %EnvDomainId - Environment variable that stores domain ID
        EnvDomainId = 'ROS_DOMAIN_ID'

        %EnvRMW - Environment variable that indicates middleware to use
        EnvRMW = 'RMW_IMPLEMENTATION'

        %DefaultServerMode - Default to using ROS 2 network
        DefaultServerMode = 0

        %DefaultLogLevel - Default to logging only fatal exceptions
        DefaultLogLevel = 5

        %DefaultLogPath - Default to logging in temporary directory
        DefaultLogPath = fullfile(tempdir, 'ros2_log');

        %DebugMode - Do not debug in release
        DebugMode = false

        %MinimumWinVer - Minimum Windows version number supported by ROS 2
        MinimumWinVer = 10
    end

    methods
        function obj = ros2node(name, varargin)
        %ros2node Create a ROS 2 node object
        %   The "name" argument is required and specifies a node name to be
        %   used on the ROS 2 network.

        % Parse and assign input
            narginchk(1, inf)
            name = convertStringsToChars(name);
            parser = getParser(obj);
            parse(parser, name, varargin{:});
            obj.ID = parser.Results.id;
            obj.ServerMode = parser.Results.ServerMode;
            obj.LogLevel = parser.Results.LogLevel;
            obj.LogPath = parser.Results.LogPath;
            obj.RMWImplementation = parser.Results.RMWImplementation;

            % Create node
            setupPaths(obj)
            createNode(obj, parser.Results.name, parser.Results.id);

            function parser = getParser(obj)
            % Domain ID can be set by environment variable but input is
            % is given priority

                defaultId = ros.internal.utilities.getDefaultDomainID;
                defaultRMWImplementation = ros.internal.utilities.getDefaultRMWImplementation;

                % Set up parser
                parser = inputParser;
                addRequired(parser, 'name', ...
                            @(x) validateattributes(x, ...
                                                    {'char', 'string'}, ...
                                                    {'nonempty', 'scalartext'}, ...
                                                    'ros2node', ...
                                                    'name'));
                addOptional(parser, 'id', defaultId, ...
                            @(x) validateattributes(x, ...
                                                    {'numeric'}, ...
                                                    {'scalar', 'integer', 'nonnegative', '<=', 232}, ...
                                                    'ros2node', ...
                                                    'id'))
                addParameter(parser, 'ServerMode', obj.DefaultServerMode, ...
                             @(x) validateattributes(x, ...
                                                     {'numeric', 'logical'}, ...
                                                     {'scalar', '>=', 0, '<=', 2}, ...
                                                     'ros2node', ...
                                                     'ServerMode'))
                addParameter(parser, 'LogLevel', obj.DefaultLogLevel, ...
                             @(x) validateattributes(x, ...
                                                     {'numeric'}, ...
                                                     {'scalar', '>=', 0, '<=', 5}, ...
                                                     'ros2node', ...
                                                     'LogLevel'))
                addParameter(parser, 'LogPath', obj.DefaultLogPath, ...
                             @(x) validateattributes(x, ...
                                                     {'char', 'string'}, ...
                                                     {'nonempty', 'scalartext'}, ...
                                                     'ros2node', ...
                                                     'LogPath'))
                addParameter(parser, 'RMWImplementation', defaultRMWImplementation, ...
                             @(x) validateattributes(x, ...
                                                     {'char', 'string'}, ...
                                                     {'scalartext'}, ...
                                                     'ros2node', ...
                                                     'RMWImplementation'))
            end
        end

        function delete(obj)
        %delete Remove reference to ROS 2 node
        %   delete(NODE) removes the reference in NODE to the ROS 2 node on
        %   the network. If no further references to the node exist, such
        %   as would be in publishers and subscribers, the node is shut
        %   down.

            try
                % Delete reference to node, without deleting internal node
                obj.InternalNode = [];
            catch ex
                warning(message('ros:mlros2:node:ShutdownError', ...
                                ex.message))
            end
        end
    end

    % All dependent properties are read from the server
    methods
        function name = get.Name(obj)
        %get.Name Custom getter for Name property

        % Allow errors to be thrown from getServerInfo
            nodeInfo = getServerInfo(obj);
            name = strcat(nodeInfo.namespace, '/', nodeInfo.name);
        end
    end

    methods (Access = ?ros.internal.mixin.InternalAccess)
        function rosName = resolveName(obj, name)
        %resolveName Check and resolve ROS 2 name
        %   ROSNAME = resolveName(NODE, NAME) resolves the ROS 2 name, NAME
        %   that is associated with a particular node object, NODE. 
        %   This function returns a ROS 2 name with properly applied
        %   namespace in ROSNAME.
        %
        %   This function will take the relevant namespace into account if
        %   the name is defined as relative name without a leading slash.
        %   A relative name, e.g. 'testName', will be resolved relative to
        %   this node's root namespace. A private name, e.g. '~/testName'
        %   will be resolved relative to this node's namespace. A leading
        %   forward slash indicates a global name, e.g. '/testName'.
        %
        %   This function does not check the name against the ROS 2
        %   naming standard - it only resolves valid namespace indicators
        %   to be relative to the appropriate node-related namespace.

            rosName = name;                 % Default to no change
            if startsWith(name, '~/')       % Private namespace
                                            % Replace private token with full node name
                                            % (using full node namespace and name as the new namespace)
                rosName = strrep(name, '~', obj.Name);
            elseif ~startsWith(name, '/')   % Relative namespace
                nodeInfo = getServerInfo(obj);
                rosName = strcat(nodeInfo.namespace, '/', name);
            end
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
                               {'libmwros2server.exe', ...   % win64
                                'libmwros2server', ...       % glnxa64
                                'libmwros2server'});         % maci64
            obj.ServerPath = fullfile(serverPathBase, serverPathMap(arch));

            % Server start directory suggestion
            startPathBase = fullfile(mlRoot, 'sys', 'ros2', ...
                                     arch, 'ros2');
            startPathMap = ...
                containers.Map(archKeys, ...
                               {'bin', ...    % win64
                                'lib', ...    % glnxa64
                                'lib'});      % maci64
            obj.ServerStartPath = fullfile(startPathBase, ...
                                           startPathMap(arch));

            % Architecture-applicable load path environment variable
            envPathMap = ...
                containers.Map(archKeys, ...
                               {'PATH', ...             % win64
                                'LD_LIBRARY_PATH', ...  % glnxa64
                                'DYLD_LIBRARY_PATH'});  % maci64
            obj.EnvApplicablePath = envPathMap(arch);

            obj.DataArrayLibPath = fullfile(mlRoot,'extern','bin',arch);
        end

        function createNode(obj, name, id)
        %createNode Create node on ROS 2 network

            try
                % Split fully qualified name into namespace and base name
                [baseName, namespace] = nodeNameParts(name);

                % Create internal node representation
                obj.InternalNode = ros.internal.Node;

                % Set domain ID correctly and remove it after node creation
                domainIdCurrentValue = getenv(obj.EnvDomainId);
                setenv(obj.EnvDomainId, num2str(id, '%.0f'));
                cleanDomainId = onCleanup(...
                    @() setenv(obj.EnvDomainId, domainIdCurrentValue));

                % Setting for middleware implementation correctly and reset
                % it after node creation
                rmwCurrentValue = getenv(obj.EnvRMW);
                setenv(obj.EnvRMW, obj.RMWImplementation);
                clearRMW = onCleanup(...
                    @() setenv(obj.EnvRMW, rmwCurrentValue));

                % Set path correctly and remove it after node creation
                % The path environment variable needs the server start
                % directory on it, and the custom message libraries in case
                % they are required later in the node process
                customMsgRegistry = ros.internal.CustomMessageRegistry.getInstance('ros2');
                customMsgDirList = getBinDirList(customMsgRegistry);
                pathCurrentValue = getenv(obj.EnvApplicablePath);
                setenv(obj.EnvApplicablePath, ...
                       strjoin([obj.ServerStartPath customMsgDirList obj.DataArrayLibPath pathCurrentValue], pathsep))
                cleanPath = onCleanup(...
                    @() setenv(obj.EnvApplicablePath, pathCurrentValue));

                % Start node on ROS 2 network
                returnCall = create(obj.InternalNode, baseName, ...
                                    namespace, obj.ServerPath, ...
                                    obj.ServerStartPath, obj.ServerMode, ...
                                    obj.LogLevel, obj.LogPath, ...
                                    obj.DebugMode);

                % Check output and error if node not created
                if isempty(returnCall) || ~isstruct(returnCall)
                    error(message('ros:mlros2:node:InvalidReturnCallError'))
                elseif ~isfield(returnCall, 'handle') || ...
                        isempty(returnCall.handle)
                    error(message('ros:mlros2:node:InvalidReturnCallHandleError'))
                end
                obj.ServerNodeHandle = returnCall.handle;
            catch ex
                if strcmp(ex.identifier, ...
                          'ros:internal:transport:ServerNotUpError')
                    error(message('ros:mlros2:node:CreationServerError', ...
                                  name));
                elseif strcmp(ex.identifier,'ros:internal:transport:ServerFailedToStart') && ...
                        ~checkMinimumWinVer(obj.MinimumWinVer)
                    error(message('ros:mlros2:node:UnsupportedWindowsVersion', ...
                                  name));
                else
                    newEx = MException(message('ros:mlros2:node:CreationGenericError', ...
                                               name));
                    throw(newEx.addCause(ex));
                end
            end
        end

        function nodeInfo = getServerInfo(obj)
        %getServerInfo Get node properties from server

        % Ensure properties are valid
            if isempty(obj.InternalNode) || ~isvalid(obj.InternalNode)
                error(message('ros:mlros2:node:InvalidInternalNodeError'))
            elseif isempty(obj.ServerNodeHandle)
                error(message('ros:mlros2:node:InvalidServerHandleError'))
            end

            % Extract node information
            try
                nodeInfo = nodeinfo(obj.InternalNode, ...
                                    obj.ServerNodeHandle, []);
            catch ex
                newEx = MException(message('ros:mlros2:node:GetInfoError'));
                throw(newEx.addCause(ex));
            end
        end
    end

    methods (Static = true, Access = private)
        function name = matlabCodegenRedirect(~)
            % MATLAB codegen
            name = 'ros.internal.codegen.ros2node';
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

function ret = checkMinimumWinVer(minVerNum)
    ret = true;
    if ispc
        [status, result] = system('ver');
        if status == 0
            % Example result on Windows 10  --> Microsoft Windows [Version 10.0.17763.720]
            % Example result on Windows 7   --> Microsoft Windows [Version 6.1.7601]
            % The result string is consistent across machines with different locales
            % Pattern match to extract the leading version number, e.g. 10 or 6
            version = regexp(result, '(?<=.*Version )\d*(?=\..*)', 'match','once');
            if ~isempty(version) && ~isnan(str2double(version)) && (str2double(version) < minVerNum)
                ret = false;
            end
        end
    end
end

% LocalWords:  ROSNAME
