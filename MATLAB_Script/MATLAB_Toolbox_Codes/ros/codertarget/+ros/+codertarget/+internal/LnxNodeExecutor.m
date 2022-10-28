classdef LnxNodeExecutor < ros.codertarget.internal.NodeInterface & ...
        ros.codertarget.internal.NodeExecutorBase
    %LnxNodeExecutor Linux node executor

    % Copyright 2021 The MathWorks, Inc.
    methods
        function obj = LnxNodeExecutor(systemExecutor,rosVersion)
            obj@ros.codertarget.internal.NodeExecutorBase(systemExecutor,rosVersion);
            if isa(obj.SystemExecutor,'ros.codertarget.internal.RemoteLnxSystemExecutor')
                % Remote execution over SSH
                obj.IsRemote = true;
            end
        end

        function runNode(obj,modelName,workspaceFolder,rosMasterURI,nodeHost,varargin)
            %RUNNODE Run a node given modelName
            
            if obj.IsRemote
                runRemoteNode(obj,modelName,workspaceFolder,...
                    rosMasterURI,nodeHost,varargin{:});
            else
                if nargin > 6
                    cmdArgs = varargin{1};
                else
                    cmdArgs = '';
                end
                % Deployment site is local host
                nodeName = obj.getNodeNameFromModel(modelName);
                nodeExecutable = getNodeExecutable(obj,nodeName,...
                    getExecutableFolder(obj,workspaceFolder));
                if isempty(nodeExecutable)
                    error(message('ros:slros:rosdevice:NodeNotFound', nodeExecutable, modelName));
                end
                
                % Determine log file location. By default, try to create the
                % log file in the ROS / ROS 2 project folder root, but if that
                % folder is not writable, create the logfile in a temporary
                % location
                if obj.isFileWritable(workspaceFolder)
                    logFile = obj.handleSpaces([workspaceFolder '/' nodeName '.log']);
                else
                    [~,b] = fileparts(tempname);
                    logFile = obj.handleSpaces(['/tmp/' nodeName '_' b '.log']);
                end
                if isequal(obj.ROSVersion,'ros')
                    origROSMasterURI = getenv('ROS_MASTER_URI');
                    restoreROSMasterURI = onCleanup(@()setenv('ROS_MASTER_URI',origROSMasterURI));
                    setenv('ROS_MASTER_URI',rosMasterURI);
                    if ~isempty(nodeHost)
                        if obj.isIPAddress(nodeHost)
                            envVarName = 'ROS_IP';
                        else
                            envVarName = 'ROS_HOSTNAME';
                        end
                        origNodeHost = getenv(envVarName);
                        restoreNodeHost= onCleanup(@()setenv(envVarName,origNodeHost));
                        setenv(envVarName,nodeHost);
                    end

                    % Run command
                    cmd = ['"' nodeExecutable '" ' cmdArgs ' > ' logFile ' 2>&1 &'];
                    [status,result] = ros.internal.executeROSCommand(cmd);
                    if status ~= 0
                        error(message('ros:slros:rosdevice:StartROSNodeError',modelName,result));
                    end
                else
                    % Use "ros2 run <package_name> <executable_name>" command
                    installFolder = obj.SystemExecutor.fullfile(workspaceFolder,'install');
                    currDir = cd(installFolder);
                    cdToPwd = onCleanup(@()cd(currDir));
                    [~,packageName] = fileparts(fileparts(nodeExecutable));
                    cmd = ['run ' packageName ' ' nodeName ' > ' logFile ' 2>&1 &'];
                    [status,result] = ros.ros2.internal.runros2cmd(cmd);
                    if status ~= 0
                        error(message('ros:slros:rosdevice:StartROSNodeError',modelName,result));
                    end
                end
                pause(1); % Allow executable to fail
                
                % Check if node has launched correctly. This is a while
                % loop that tests up to 10 times if the process ID of the new
                % process can be found (1 second wait at each iteration)
                ts = tic;
                while toc(ts) < obj.Timeout
                    if isNodeRunning(obj,modelName)
                        return;
                    end
                end
                
                % Node did not start in timeout period. Get log file contents
                % and let the user know if an error is detected
                logOut = obj.readTextFile(logFile);
                if contains(logOut,'error','IgnoreCase',true)
                    error(message('ros:slros:rosdevice:ROSNodeDidNotStart',nodeName,...
                        logFile,logOut));
                end
            end
        end

        function stopNode(obj,modelName)
            if isNodeRunning(obj,modelName)
                nodeName = obj.getNodeNameFromModel(modelName);
                if isequal(obj.ROSVersion,'ros')
                    % Use 'rosnode kill' command to terminate the node
                    cmd = ['rosnode kill ' nodeName];
                    status = ros.internal.executeROSCommand(cmd);
                    if status ~= 0
                        killNodeForcefully(obj,nodeName);
                    end
                else
                    % ROS2
                    killNodeForcefully(obj,nodeName);
                end
            end
        end

        function isRunning = isNodeRunning(obj,modelName)
            nodeName = modelName;
            isRunning = false;

            % Output pid and args left justified
            cmd = sprintf('ps axo pid:1,args:1 | grep -Ei "%s(\\s|$)" | grep -v "grep"',nodeName);
            try
                result = system(obj.SystemExecutor,cmd);
            catch
                result = '';
            end
            if ~isempty(result)
                % Find out pid's of potential candidates by finding digits at each
                % line beginning.
                % result = '1257 bash\n3022 bash\n12757 bash\n' will return a cell
                % array {'1257','3022','12757'} using extract below
                pidList = extract(result,lineBoundary + digitsPattern);
                % Test comm to make sure it matches appName. In Linux, only
                % first 15 characters are stored in /proc/pid/comm file so make
                % sure we only test against first 15 characters of appName
                pat = nodeName(1:min(15,length(nodeName)));
                for k = 1:length(pidList)
                    try
                        % ^%s to match process name at the beginning of the
                        % string
                        result = system(obj.SystemExecutor,...
                            sprintf('cat /proc/%s/comm | grep -i "^%s"',pidList{k},pat));
                    catch
                        result = '';
                    end
                    if contains(result,pat,IgnoreCase=true)
                        isRunning = true;
                        break;
                    end
                end
            end
        end
        
        function nodeList = getAvailableNodes(obj,workspaceFolder)
            %getAvailableNodes Get list of runnable ROS nodes
            nodeList = cell(0,1);
            exeFolder = getExecutableFolder(obj,workspaceFolder);
            cmd = ['find ' obj.handleSpaces(exeFolder) ...   % Find (recursively) in executable workspace
                ' -executable -type f -printf ''%f\n'''];    % Just print filename and not the path
            try
                output = strtrim(system(obj.SystemExecutor,cmd));
            catch
                output = '';
            end
            if isempty(output)
                return;
            end
            nodeCandidates = unique(strsplit(output,'\n'));

            % Verify against sources
            srcFolder = obj.SystemExecutor.fullfile(workspaceFolder,'src');          
            for k = 1:numel(nodeCandidates)
                cmd = ['find ' obj.handleSpaces(srcFolder) ...
                    ' -type f'  ...                        % Only search for files
                    ' -iname ' nodeCandidates{k} '.cpp' ...
                    ' -o -iname ' nodeCandidates{k} '.cu' ];  % Case insensitive search
                try
                    output = system(obj.SystemExecutor,cmd);
                catch
                    output = '';
                end
                if ~isempty(output)
                    nodeList = [nodeList nodeCandidates{k}]; %#ok<AGROW> 
                end
            end
        end
    end
    
    methods (Access = ?ros.slros.internal.InternalAccess)
        function runRemoteNode(obj,modelName,workspaceFolder,varargin)
        %runRemoteNode Run node on remote device
        %   This is an internal function and it assumes that all the
        %   input arguments have been validated.

            nodeName = obj.getNodeNameFromModel(modelName);
            nodeExecutable = getNodeExecutable(obj,nodeName,...
                getExecutableFolder(obj,workspaceFolder));
            if isempty(nodeExecutable)
                error(message('ros:slros:rosdevice:NodeNotFound', nodeName, modelName));
            end

            % Determine log file location. By default, try to create the
            % log file in the ROS / ROS 2 project folder root, but if that
            % folder is not writable, create the logfile in a temporary
            % location
            if obj.isFileWritable(workspaceFolder)
                logFile = obj.handleSpaces([workspaceFolder '/' nodeName '.log']);
            else
                [~,b] = fileparts(tempname);
                logFile = obj.handleSpaces(['/tmp/' nodeName '_' b '.log']);
            end
            cmd = getRunCommand(obj,workspaceFolder,nodeExecutable,logFile,varargin{:});
            system(obj.SystemExecutor,cmd);
            pause(1); % Allow some time for the node to fail before checking if it is running

            % Check if ROS 2 node has launched correctly. This is a while
            % loop that tests up to 10 times if the process ID of the new
            % process can be found (1 second wait at each iteration)
            numTries = 10;
            cmd = ['n=0; while [ ! `pidof ' nodeName '` ] '...
                   '&& [ $n -lt  ' num2str(numTries) ' ]; do n=$((n+1)); sleep 1; done; echo $n'];

            % Return number of executions
            numExec = str2double(system(obj.SystemExecutor, cmd));

            % Get log file contents. This will return '' if the log file
            % does not exist.
            try
                logOut = string(system(obj.SystemExecutor,['cat ' logFile]));
            catch
                logOut = '';
            end

            % If the node has not launched or if the log file contains an
            % [ERROR] message, let the user know.
            if numExec == numTries || logOut.contains('[ERROR]')
                error(message('ros:slros:rosdevice:ROSNodeDidNotStart', nodeName, logFile, char(logOut)));
            end
        end
        
        function fileWritable = isFileWritable(obj, fileName)
        %isFileWritable Checks if file/folder with given name is writable by the user
        %   Returns TRUE if file/folder is writable and FALSE otherwise.
        %   Note that this function works for both files as well as
        %   folders.

            try
                % Call with the '-w' option to check if file/folder is
                % writable
                system(obj.SystemExecutor, ['[ -w ' obj.handleSpaces(fileName) ' ]']);

                % If the command executes successfully, the folder exists
                fileWritable = true;
            catch ex
                % If something unexpected goes wrong, e.g. SSH connection fails,
                % rethrow the associated exception
                if ~strcmp(ex.identifier, 'utils:sshclient:system')
                    rethrow(ex);
                end

                % File does not exist
                fileWritable = false;
            end
        end
    end
    
    methods (Access = protected)               
        function ret = getRunCommand(obj, workspaceFolder, ...
                nodeExecutable, logFile, varargin)
            if isequal(obj.ROSVersion,'ros')
                % Run command for ROS
                rosMasterURI = varargin{1};
                nodeHost = varargin{2};
                if nargin > 7
                    cmdArgs = varargin{3};
                else
                    cmdArgs = '';
                end
                sudoBashCmd = obj.SystemExecutor.commandWithSudo(nodeExecutable);
                
                % Construct run command
                ret = 'export DISPLAY=:0.0; export XAUTHORITY=~/.Xauthority; ';
                if ~isempty(rosMasterURI)
                    % Export ROS master URI
                    ret = [ret 'export ROS_MASTER_URI=' rosMasterURI '; '];
                end
                if ~isempty(nodeHost)
                    % Export ROS NodeHost
                    ret = [ret 'export ROS_IP=' nodeHost '; '];
                end
                ret = [ret 'cd ' workspaceFolder '/devel;' 'source ' workspaceFolder '/devel/setup.bash; ' ... % Source the Catkin workspace
                    sudoBashCmd ' ' cmdArgs ' ' ...                     % Run node executable
                    '&> ' logFile ' &'...                               % Pipe all output into a log file
                    ];
            else
                % Run command for ROS2
                if nargin < 5
                    args = '';
                else
                    args = varargin{1};
                end
                netProf = ros.slros.internal.sim.NetworkAddrProfile();
                domainID = num2str(netProf.getDefaultDomainID);
                rmwImplementation = netProf.getDefaultRMWImplementation;
                [exepath, ~] = fileparts(nodeExecutable);
                [~, modelname] = fileparts(exepath);
                % Run ROS 2 node
                ret = ['export DISPLAY=:0.0; ' ...
                    'export XAUTHORITY=~/.Xauthority; ' ...
                    'export ROS_DOMAIN_ID=' domainID '; ' ...     % Export ROS DOMAIN ID
                    'export RMW_IMPLEMENTATION=' rmwImplementation '; ' ...  % Export RMW Implementation
                    'cd ' workspaceFolder '/install/' modelname ';' ...
                    'source ' workspaceFolder '/install/setup.bash; ' ...  % Source the ROS 2 project folder
                    nodeExecutable ' ' args ' ' ...               % Run node executable
                    '&> ' logFile ' &'...                         % Pipe all output into a log file
                    ];
            end
        end
        
        function nodeExecutable = getNodeExecutable(obj,nodeName,exeFolder)
            % Find (recursively) in workspace folder dedicated for
            % executables
            cmd = ['find ' obj.handleSpaces(exeFolder) ' -executable -type f -name ' nodeName];
            try
                nodeExecutable = obj.handleSpaces(strtrim(system(obj.SystemExecutor,cmd)));
            catch
                nodeExecutable = '';
            end
        end
        
        function [status,result] = killProcess(obj,procName)
            % killProcess Kill a process
            cmd = sprintf('pkill -f %s',procName);
            sudoCmd = commandWithSudo(obj.SystemExecutor,cmd);
            try
                result = system(obj.SystemExecutor,sudoCmd);
                status = 0;
            catch EX
                result = EX.message;
                status = 1;
            end
        end

        function killNodeForcefully(obj,nodeName)
            [status,result] = obj.killProcess(nodeName);
            if status ~= 0
                if contains(result,'Operation not permitted')
                    % The user does not have the correct privileges to kill the node
                    error(message('ros:slros:rosdevice:StopROSNodeNoPrivileges',modelName,''));
                elseif contains(result,'no process found')
                    % This is okay. Silently swallow this exception.
                else
                    % Throw generic error if something else went wrong
                    error(message('ros:slros:rosdevice:StopROSNodeError',modelName,result));
                end
            end
        end
    end

    methods (Static, Access=protected)
        function escapedPath = handleSpaces(filePath)
            %handleSpaces Handle spaces in file or folder path
            %   In the system command that we send over SSH, spaces in the
            %   path to a file or a folder need to be escaped with a backslash.

            escapedPath = strrep(filePath, ' ', '\ ');

            % If the spaces were already escaped, we should undo the
            % double-escaping
            escapedPath = strrep(escapedPath, '\\ ', '\ ');
        end
    end
end