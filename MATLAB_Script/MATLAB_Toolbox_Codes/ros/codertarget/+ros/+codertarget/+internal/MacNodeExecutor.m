classdef MacNodeExecutor < ros.codertarget.internal.LnxNodeExecutor
    %MacNodeExecutor MAC node executor
    
    % Copyright 2021 The MathWorks, Inc.
    methods
        function runNode(obj,modelName,workspaceFolder,rosMasterURI,nodeHost)

            %RUNNODE Run a node given modelName
            % Get node executable
            if isequal(obj.ROSVersion,'ros')
                nodeName = obj.getNodeNameFromModel(modelName);
                nodeExecutable = obj.getNodeExecutable(modelName,getExecutableFolder(obj,workspaceFolder));
                if isempty(nodeExecutable)
                    error(message('ros:slros:rosdevice:NodeNotFound', ...
                        nodeExecutable, modelName));
                end

                % Create the log file in the ROS project folder root.
                logFile = [workspaceFolder '/' nodeName '.log'];
                if ~isempty(nodeHost)
                    if obj.isIPAddress(nodeHost)
                        envVarName = 'ROS_IP';
                    else
                        envVarName = 'ROS_HOSTNAME';
                    end
                    cmd = ['export ROS_MASTER_URI=' rosMasterURI...
                        ' && export ' envVarName '=' nodeHost...
                        ' && \"' replace(nodeExecutable,'\ ',' ') '\"'];
                else
                    cmd = ['export ROS_MASTER_URI=' rosMasterURI...
                        ' && \"' replace(nodeExecutable,'\ ',' ') '\"'];
                end
                % MACOSX: ros.internal.runroscmd MATLAB system call does not
                % inherit the shell variables - so use automation with
                % osascript to run the ROS Node
                [~, res] = ros.internal.roscmdshell();
                resString = string(res);
                sourceShell = char(extractBetween(resString,'''',''''));
                osaCmd =  sprintf(['/usr/bin/osascript -e ', ...
                    '''do shell script " %s;',...
                    ' { %s; } > \\"%s\\" 2>&1 &" '''], ...
                    sourceShell, cmd, logFile);
                system(osaCmd);
            else
                % ROS2: Call runNode method in the super class
                runNode@ros.codertarget.internal.LnxNodeExecutor(obj,...
                    modelName,workspaceFolder,rosMasterURI,nodeHost);
            end
        end

        function isRunning = isNodeRunning(obj,modelName)
            nodeName = obj.getNodeNameFromModel(modelName);

            % Output pid and args left justified
            cmd = sprintf('ps axo comm | grep -E "%s" | grep -v "grep"',nodeName);
            try
                result = system(obj.SystemExecutor,cmd);
            catch
                result = '';
            end
            isRunning = contains(result,nodeName);
        end

        function nodeList = getAvailableNodes(obj,workspaceFolder)
            %getAvailableNodes Get list of runnable ROS nodes
            nodeList = cell(0,1);
            exeFolder = getExecutableFolder(obj,workspaceFolder);
            cmd = ['find ' obj.handleSpaces(exeFolder) ...     % Find (recursively) in executable workspace
                ' -perm +111 -type f -exec basename {} \;'];   % Just print filename and not the path
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
                    ' -iname ' nodeCandidates{k} '.cpp'];  % Case insensitive search
                try
                    output = strtrim(system(obj.SystemExecutor,cmd));
                catch
                    output = '';
                end
                if ~isempty(output)
                    nodeList = [nodeList nodeCandidates{k}]; %#ok<AGROW> 
                end
            end
        end
    end

    methods (Access = protected)
        function nodeExecutable = getNodeExecutable(obj,nodeName,exeFolder)
            % Find (recursively) in workspace folder dedicated for
            % executables. -perm +111: List all files that has executable
            % bit set
            cmd = ['find ' obj.handleSpaces(exeFolder) ' -perm +111 -type f -name ' nodeName];
            try
                nodeExecutable = obj.handleSpaces(strtrim(system(obj.SystemExecutor,cmd)));
            catch
                nodeExecutable = '';
            end
        end
    end
end