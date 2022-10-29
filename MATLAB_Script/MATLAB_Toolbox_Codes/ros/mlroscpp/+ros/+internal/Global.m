classdef Global
    %This class is for internal use only. It may be removed in the future.
    
    %GLOBAL Functions for handling the global node and global core
    %   MATLAB uses a unique, global ROS node object for each running MATLAB
    %   session. Optionally, MATLAB can also launch a global core.
    %   The ros.internal.Global class exposes functions that allow the
    %   manipulation of these global entities.
    %
    %   ros.internal.Global methods:
    %      core  - Initialize or clear the global ROS core object
    %      node  - Initialize or clear the global ROS node object
    
    %   Copyright 2020 The MathWorks, Inc.
    
    properties(Access = ?matlab.unittest.TestCase, Constant)
        %DefaultNodeName - The default prefix for the global node name
        DefaultNodeName = 'matlab_global_node_'
        
        %DefaultNodeNameDigits - Number of digits that are suffixed to the global node name
        DefaultNodeNameDigits = 5
        
        %RandomSource - Source of pseudo-random numbers for node name
        RandomSource = robotics.internal.Random
    end
    
    methods (Static)
        function masterURI = core( op, port )
            %CORE Initialize or clear the global core object
            %   The operation 'op' defines the behavior of this function.
            %
            %   ros.internal.Global.core('clear') clears (deletes) the global
            %   core. It will be reinitialized on the next call to ROSINIT or
            %   ros.internal.Global.core('init')
            %
            %   ros.internal.Global.core('init', PORT)
            %   initializes the global ROS core on PORT. This call will fail
            %   if there is another core already running on that port. If
            %   PORT is [], the default of 11311 will be used.
            
            switch op
                case 'clear'
                    ros.internal.Global.coreInstance('clear');
                    if nargout > 0
                        masterURI = [];
                    end
                case 'init'
                    if isempty(port)
                        port = ros.internal.getDefaultCorePort;
                    end
                    
                    core = ros.internal.Global.coreInstance('init', ...
                        port);
                    
                    if nargout > 0
                        % If Master is started in MATLAB, use 'localhost' in MasterURI. 
                        % core.Port is randomly assigned if input argument 'port' is 0.
                        masterURI = ['http://localhost:' num2str(core.Port)];             
                    end
                    
                otherwise
                    error(message('ros:mlros:core:GlobalCoreInvalidOp', op));
            end
        end
        
        function node( op, name, masterURI, nodeHost )
            %NODE Initialize or clear the global node object
            %   The operation 'op' defines the behavior of this function.
            %
            %   ros.internal.Global.node('clear') clears (deletes) the global
            %   node. It will be reinitialized on the next call to ROSINIT or
            %   ros.internal.Global.node('init')
            %
            %   ros.internal.Global.node('init', 'NAME', 'MASTERURI', 'NODEHOST')
            %   initializes the global ROS node.
            %   NAME is a the node name (specify '' if
            %   the default should be used). MASTERURI denotes the Master
            %   URI that the global node should connect to (use '' if the
            %   default should be used). The NODEHOST setting is used by
            %   the node to advertise itself (use '' for default).
            
            switch op
                case 'clear'
                    ros.internal.Global.nodeInstance('clear');
                    
                case 'init'
                    % Always clear existing node
                    ros.internal.Global.nodeInstance('clear');
                    
                    % Then re-initialize node
                    ros.internal.Global.nodeInstance('init', ...
                        name, masterURI, nodeHost);
                    
                otherwise
                    error(message('ros:mlros:node:GlobalNodeInvalidOp', op));
            end
        end
        
        function [active, nodeHandle] = isNodeActive
            %isNodeActive Check if global node is running or not.
            
            try
                nodeHandle = ros.internal.Global.getNodeHandle(false);
                active = true;
            catch
                active = false;
                nodeHandle = [];
            end
        end
        
        function [active, coreHandle] = isCoreActive
            %isCoreActive Check if global core is running or not.
            
            try
                coreHandle = ros.internal.Global.getCoreHandle(false);
                active = true;
            catch
                active = false;
                coreHandle = [];
            end
        end
        
        function node = getNodeHandle(init)
            %getNodeHandle Get instance of the global node
            
            node = ros.internal.Global.nodeInstance;
            if ~isempty(node)
                return;
            end
            
            % The global node is not initialized
            if init
                % Initialize the node if requested by the caller
                ros.internal.Global.node('init', '', '', '');
                node = ros.internal.Global.nodeInstance;
            else
                % If user did not request a node start, throw an
                % appropriate error message
                error(message('ros:mlros:node:GlobalNodeNotRunning'));
            end
        end
        
        function core = getCoreHandle(init)
            %getCoreHandle Get instance of the global node
            
            core = ros.internal.Global.coreInstance;
            if ~isempty(core)
                return;
            end
            
            % The global core is not initialized
            if init
                % Initialize the core if requested by the caller
                ros.internal.Global.core('init', []);
                core = ros.internal.Global.coreInstance;
            else
                % If user did not request a core start, throw an
                % appropriate error message
                error(message('ros:mlros:core:GlobalCoreNotRunning'));
            end
        end
    end
    
    methods(Access = private, Static)
        function n = nodeInstance(op, name, masterURI, nodeHost)
            persistent node
            mlock
            
            if nargin == 0
                n = node;
                return;
            end
            
            switch op
                case 'clear'
                    clearNode;
                case 'init'
                    initNode(name, masterURI, nodeHost);
            end
            
            function initNode(name, masterURI, nodeHost)
                if isempty(name)
                    name = ros.internal.Global.DefaultNodeName;
                    nodeCreateFcn = @(n,uri,nhost)ros.internal.Global.createNodeWithDefaultName(n, uri, 'NodeHost', nhost);
                else
                    nodeCreateFcn = @(n,uri,nhost)ros.Node(n, uri, 'NodeHost', nhost);
                end
                
                clearNode;
                node = nodeCreateFcn(name, masterURI, nodeHost);
                disp(getString(message('ros:mlros:node:InitializeGlobal', ...
                    node.Name, node.NodeURI, node.MasterURI)));
            end
            
            function clearNode
                % Check for the unlikely case that node is already a
                % deleted handle and recover gracefully.
                if isobject(node) && ~isvalid(node)
                    node = [];
                    return;
                end
                
                if ~isempty(node)
                    disp(getString(message('ros:mlros:node:ShuttingDownGlobal', ...
                        node.Name, node.NodeURI, node.MasterURI)));
                    delete(node);
                    node = [];
                end
            end
            
        end
        
        function c = coreInstance(op, port)
            persistent core
            mlock
            
            if nargin == 0
                c = core;
                return;
            end
            
            switch op
                case 'init'
                    initCore(port);
                    c = core;
                    
                case 'clear'
                    clearCore();
                    return;
            end
            
            function clearCore
                % Check for the unlikely case that core is already a
                % deleted handle and recover gracefully.
                if isobject(core) && ~isvalid(core)
                    core = [];
                    return;
                end
                
                if ~isempty(core)
                    disp(getString(message('ros:mlros:core:ShuttingDownGlobal', ...
                        core.MasterURI)));
                    delete(core);
                    core = [];
                end
            end
            
            function initCore(port)
                if isempty(core)
                    core = ros.Core(port,'IsMasterCheckDone',true);
                    disp(getString(message('ros:mlros:core:InitializeGlobal', ...
                        core.MasterURI)));
                    return;
                end
                
                % If port number is different, restart core
                if port ~= core.Port
                    clearCore();
                    core = ros.Core(port);
                    disp(getString(message('ros:mlros:core:InitializeGlobal', ...
                        core.MasterURI)));
                end
            end
        end
        
        function node = createNodeWithDefaultName(name, varargin)
            %createNodeWithDefaultName Create the global ROS node with a default name
            %   The input arguments are fed straight into the ros.Node constructor.
            
            nodeCreated = false;
            
            % Create random node name
            % Cycle multiple times in the (very unlikely) case that a node with
            % the same name already exists.
            retryMax = 3;
            retryCount = 0;
            
            numDigits = ros.internal.Global.DefaultNodeNameDigits;
            
            while ~nodeCreated
                % Use a random number as a suffix
                randNumStr = ros.internal.Global.RandomSource.timeNumericString(numDigits);
                try
                    node = ros.Node([name randNumStr], varargin{:});
                catch ex
                    switch ex.identifier
                        case 'ros:mlros:node:NodeNameExists'
                            % This exception occurred because our
                            % random name collides with a name of a
                            % different node. Retry the node creation with
                            % a different random name up to retryMax times.
                            if retryCount == retryMax
                                error(message('ros:mlros:node:DefaultNodeNameError'));
                            end
                            retryCount = retryCount + 1;
                            
                            % Pause for a little bit to ensure that time is
                            % changing. Randomize the wait time between 0.05 and 0.15
                            % seconds, in case that multiple instances of MATLAB
                            % use the same time base.
                            waitTime = 0.1 + 0.1*(rand-0.5);
                            pause(waitTime);
                            continue;
                        otherwise
                            % Some other exception occurred.
                            rethrow(ex);
                    end
                end
                nodeCreated = true;
            end
        end
    end
    
end
