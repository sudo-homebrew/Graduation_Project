classdef (Hidden,Abstract) RemoteDevice < ...
        ros.codertarget.internal.LinuxSystemInterface & ...
        robotics.core.internal.mixin.Unsaveable
    % This class is for internal use and may be removed in a future
    % release.

%   Copyright 2021 The MathWorks, Inc.
    
    %REMOTEDEVICE Abstract class with methods to connect and run commands
    %on a remote Linux device
    properties (Dependent, SetAccess = private)
        %DeviceAddress - Hostname or IP address of the Remote device
        %   For example, this can be an IP address or a hostname.
        DeviceAddress
    end
    
    properties (SetAccess = private)
        %Username - Username used to connect to the device
        Username
    end
    
    properties (Abstract,Constant,Hidden)
        % ROS or ROS 2 Device type ('rosdevice' or 'ros2device')
        DeviceType
        
        % NodeMarkerFile Marker file used to identify available node
        NodeMarkerFile
    end
    
    properties (Dependent, SetAccess = private)
        %AvailableNodes - Nodes that are available to run
        %   This list captures deployed Simulink nodes in the ROS or ROS 2
        %   workspace that are available to run.
        AvailableNodes
    end
    
    properties (Access = {?matlab.unittest.TestCase,?ros2device,?rosdevice})
        %Port - SSH port used to connect to the Remote device
        %   Default: 22
        Port = 22
        
        %Password - Password used to connect to the Remote device
        Password
        
        %Parser - Parser object for user inputs
        Parser
        
        %Diagnostic - Diagnostic helper object
        Diagnostic
    end
    
    properties (Access = protected)
        %Ssh - SSH client used to connect to the device
        Ssh
    end
    
    methods (Abstract,Access=protected)
        initProperties(obj,deviceParams);
        ret = findCandidateNodeNames(obj);
        ret = getNodeExecutable(obj, modelName);
        ret = getWorkspaceFolder(obj);
        cmd = getRunCommand(obj, workspaceFolder, ...
                nodeExecutable, logFile, varargin);
    end
    
    methods
        function obj = RemoteDevice(hostname, username, password, port)
            %ROSDEVICE Connect to remote ROS device
            %   Please see the class documentation for more details on how
            %   to create a ROSDEVICE object.
            %
            %   See also ROSDEVICE.
            
            narginchk(0, 5);
            
            obj.Parser = ros.slros.internal.DeviceParameterParser;
            deviceParams = ros.codertarget.internal.DeviceParameters;
            
            % Parse the user input and initialize the object
            % Since all inputs are optional, parse them progressively.
            if nargin < 1
                hostname = deviceParams.getHostname;
                assert(~isempty(hostname), message('ros:slros:rosdevice:InvalidDeviceAddress'));
            else
                % Validate provided host name
                obj.Parser.validateHostname(hostname, obj.DeviceType, 'hostname');
            end
            
            % Initialize the username
            if nargin < 2
                username = deviceParams.getUsername;
                assert(~isempty(username), message('ros:slros:rosdevice:InvalidUsername'));
            else
                % Validate provided username
                obj.Parser.validateUsername(username, obj.DeviceType, 'username');
            end
            
            % Initialize the password
            if nargin < 3
                password = deviceParams.getPassword;
                assert(~isempty(password), message('ros:slros:rosdevice:InvalidPassword'));
            else
                % Validate provided password
                obj.Parser.validatePassword(password, obj.DeviceType, 'password');
            end
            
            % Initialize the SSH port
            if nargin < 4
                obj.Port = deviceParams.getSSHPort;
            else
                % Validate provided SSH port
                obj.Port = obj.Parser.validateSSHPort(port, obj.DeviceType, 'port');
            end
            
            % Create an SSH client
            obj.Ssh = ros.codertarget.internal.ssh2client(hostname, ...
                username, password, obj.Port);
            
            obj.Username = username;
            obj.Password = password;
            
            % Initialize a diagnostic object
            obj.Diagnostic = ros.slros.internal.diag.DeviceDiagnostics;
            obj.Diagnostic.connect(obj.Ssh);
            
            % Retrieve workspace and ROS/ROS2 install folder.
            % These properties can be changed later.
            initProperties(obj, deviceParams);
        end
        
        %% Getter and setter methods
        function deviceAddress = get.DeviceAddress(obj)
            deviceAddress = obj.Ssh.Hostname;
        end
        
        function nodeList = get.AvailableNodes(obj)
            %get.AvailableNodes Get list of runnable ROS 2 nodes
            nodeList = cell(0,1);
            try
                nodeCandidateString = findCandidateNodeNames(obj);
            catch
                return;
            end
            if isempty(nodeCandidateString)
                % No folders matched
                return;
            end
            % Convert string with newlines into cell array
            nodeCandidates = strsplit(nodeCandidateString, '\n');
            % Verify that each node in the list has a valid executable
            for i = 1:length(nodeCandidates)
                modelName = nodeCandidates{i};
                nodeExecutable = getNodeExecutable(obj, modelName);
                if obj.Diagnostic.doesFileExist(nodeExecutable)
                    nodeList{end+1} = modelName; %#ok<AGROW>
                end
            end
        end
    end
    
    %% Public Interface
    methods
        function runNode(obj, modelName, varargin)
            %runNode Start the node on device
            %   runNode(DEVICE, MODELNAME) starts the ROS / ROS 2 node associated
            %   with the Simulink model with name MODELNAME on the connected
            %   DEVICE. The node needs to be deployed in the project workspace
            %   folder specified in the workspace property. The node connects
            %   to the same ROS master that MATLAB is connected to and advertises
            %   its address as the property value 'DeviceAddress'.
            %
            %
            %   Example:
            %       device = rosdevice
            %
            %       % Run the 'robotROSFeedbackControlExample' node
            %       % This model needs to be deployed to the ROS device
            %       runNode(device, 'robotROSFeedbackControlExample')
            %
            %       % Stop the node
            %       stopNode(device, 'robotROSFeedbackControlExample')
            %
            %   See also stopNode.
            
            % Parse inputs
            validateattributes(modelName, {'char'}, {'nonempty','row'}, 'runNode', 'modelName');
            
            % If node is already running, don't do anything
            if obj.isNodeRunning(modelName)
                disp(message('ros:slros:rosdevice:NodeAlreadyRunning', modelName).getString);
                return;
            end
            
            % Pass additional command-line arguments
            obj.runNodeInternal(modelName, varargin{:});
        end
        
        function stopNode(obj, modelName)
            %stopNode Stop the node on device
            %   stopNode(DEVICE, MODELNAME) stops the node associated
            %   with the Simulink model with name MODELNAME on the connected
            %   DEVICE.
            %
            %   If the node is not running, this function returns right
            %   away.
            %
            %
            %   Example:
            %       device = rosdevice
            %
            %       % Run the 'exampleModel' node
            %       % This model needs to be deployed to the ROS device.
            %       runNode(device, 'exampleModel')
            %
            %       % Stop the node
            %       stopNode(device, 'exampleModel')
            %
            %       % Calling stop again has no effect
            %       stopNode(device, 'exampleModel')
            %
            %   See also runNode.
            
            nodeName = obj.getNodeNameFromModel(modelName);
            
            try
                stopExecutable(obj, nodeName);
            catch ex
                % Parse exception
                exMsg = string(ex.message);
                
                if exMsg.contains('Operation not permitted')
                    % The user does not have the correct privileges to kill the node
                    error(message('ros:slros:rosdevice:StopROSNodeNoPrivileges', modelName, obj.Username));
                elseif exMsg.contains('no process found')
                    % This is okay. Silently swallow this exception.
                else
                    % Throw generic error if something else went wrong
                    rethrow(ex);
                end
            end
        end
        
        function isRunning = isNodeRunning(obj, modelName)
            %isNodeRunning Determine if ROS 2 node is running on device
            %   ISRUNNING = isNodeRunning(DEVICE, MODELNAME) returns TRUE
            %   if the ROS 2 node associated with the Simulink model with
            %   name MODELNAME is running on the DEVICE.
            %   The function returns FALSE if the node is not
            %   running on the device.
            
            validateattributes(modelName, {'char'}, {'nonempty','row'}, 'isNodeRunning', 'modelName');
            nodeName = obj.getNodeNameFromModel(modelName);
            
            if length(nodeName) > 15
                % If process name is longer than 15 characters, an exact
                % match will not work (process names are limited to 15
                % characters in /proc/pid/stat.
                matchFullCmdLine = true;
            else
                % Search for exact match
                matchFullCmdLine = false;
            end
            
            % Note that the function returns the PIDs as second output
            isRunning = obj.isProcessRunning(nodeName, matchFullCmdLine);
        end
        
        function openShell(obj)
            %openShell Open interactive command shell to the device
            %   openShell(DEVICE) opens an interactive SSH shell to the ROS
            %   device connected through the DEVICE object.
            
            openShell(obj.Ssh);
        end
    end
    
    methods (Access = ?ros.slros.internal.InternalAccess)
        function runNodeInternal(obj, modelName, varargin)
        %runNodeInternal Run node on the target
        %   This is an internal function and it assumes that all the
        %   input arguments have been validated.

            workspaceFolder = getWorkspaceFolder(obj);
            nodeName = obj.getNodeNameFromModel(modelName);
            nodeExecutable = obj.getNodeExecutable(modelName);

            if ~obj.Diagnostic.doesFileExist(nodeExecutable)
                error(message('ros:slros:rosdevice:NodeNotFound', nodeExecutable, modelName));
            end

            % Determine log file location. By default, try to create the
            % log file in the ROS / ROS 2 project folder root, but if that
            % folder is not writable, create the logfile in a temporary
            % location
            if obj.Diagnostic.isFileWritable(workspaceFolder)
                logFile = [workspaceFolder '/' nodeName '.log'];
            else
                [~,b] = fileparts(tempname);
                logFile = ['/tmp/' nodeName '_' b '.log'];
            end
            cmd = getRunCommand(obj, workspaceFolder, ...
                nodeExecutable, logFile, varargin{:});
            system(obj, cmd);

            % Check if ROS 2 node has launched correctly. This is a while
            % loop that tests up to 10 times if the process ID of the new
            % process can be found (1 second wait at each iteration)
            numTries = 10;
            cmd = ['n=0; while [ ! `pidof ' nodeName '` ] '...
                   '&& [ $n -lt  ' num2str(numTries) ' ]; do n=$((n+1)); sleep 1; done; echo $n'];

            % Return number of executions
            numExec = str2double(system(obj, cmd));

            % Get log file contents. This will return '' if the log file
            % does not exist.
            logOut = string(obj.Diagnostic.safeSSHExecute(['cat ' logFile]));

            % If the node has not launched or if the log file contains an
            % [ERROR] message, let the user know.
            if numExec == numTries || logOut.contains('[ERROR]')
                error(message('ros:slros:rosdevice:ROSNodeDidNotStart', nodeName, logFile, char(logOut)));
            end
        end
    end
    
    methods (Access = {?matlab.unittest.TestCase,?rosdevice,?ros2device})
        function [isRunning, pid] = isProcessRunning(obj, processName, matchFullCmdLine)
            %isProcessRunning Determines if process with name is running
            %   As optional second output, the function returns the process
            %   IDs (PIDs) of all processes with this name. If ISRUNNING is
            %   FALSE, PID is [].
            %   If MATCHFULLCMDLINE is TRUE, the PROCESSNAME is matched
            %   against the full command line. This can be useful if
            %   running commands with sudo or with some pipe. The
            %   PROCESSNAME is matched exactly if MATCHFULLCMDLINE is
            %   FALSE.
            
            if matchFullCmdLine
                args = '-f';
            else
                args = '-x';
            end
            
            % Make the process name a regular expression by enclosing the
            % first character with square brackets. This prevents pgrep from
            % matching its own invocation over SSH.
            % See http://unix.stackexchange.com/questions/74185/how-can-i-prevent-grep-from-showing-up-in-ps-results
            if length(processName) >= 1
                processName = ['[' processName(1) ']' processName(2:end)];
            end
            
            try
                % Run pgrep with specified arguments
                pgrepOutput = obj.system(['pgrep ' args ' ' processName]);
                
                % We have to use str2num to allow conversion of multiple PIDs
                pid = str2num(pgrepOutput); %#ok<ST2NM>
                isRunning = true;
            catch
                pid = [];
                isRunning = false;
            end
        end
        
        function sudoCmd = commandWithSudo(obj, command)
            %commandWithSudo Convert the given command to run with sudo if user has admin rights
            %   If the user has admin rights, prefix the sudo operation in
            %   front of the input command.
            %   If the user does not have admin rights, the command is
            %   returned verbatim.
            
            % By default, simple pass through the command
            sudoCmd = command;
            
            [hasSudo, requiresPw] = obj.Diagnostic.hasSudoAccess(obj.Password);
            
            % By default, sudo -E does not preserve the shared library path
            % LD_LIBRARY_PATH
            sharedPath = 'LD_LIBRARY_PATH="$LD_LIBRARY_PATH"';
            
            if hasSudo
                % In both cases, make sure that the environment variables
                % of the parent session are preserved (-E option)
                if requiresPw
                    % Echo password to sudo invocation
                    sudoStr = ['echo ' obj.Password '| sudo ' sharedPath ' -E -S'];
                else
                    % Use non-interactive (-n) mode
                    sudoStr = ['sudo ' sharedPath ' -E -n'];
                end
                
                % Prefix the sudo directives to the command
                sudoCmd = [sudoStr ' ' command];
            end
        end
        
        function stopExecutable(obj, exe)
            %stopExecutable Stops an executable running on the ROS device
            
            validateattributes(exe,{'char'},...
                {'nonempty', 'row'},'stopExecutable','exe');
            
            % We don't need the full path
            [~,name,ext] = fileparts(exe);
            exe = [name,ext];
            sudoCmd = commandWithSudo(obj, ['killall ',exe]);
            system(obj, sudoCmd);
        end
    end

    methods(Abstract,Static,Access=protected)
        nodeName = getNodeNameFromModel(modelName)
    end

end

