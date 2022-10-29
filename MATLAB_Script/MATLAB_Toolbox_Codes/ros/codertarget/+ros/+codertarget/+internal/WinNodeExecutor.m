classdef WinNodeExecutor < ros.codertarget.internal.NodeInterface & ...
        ros.codertarget.internal.NodeExecutorBase
    %WinNodeExecutor Windows node executor

    % Copyright 2021 The MathWorks, Inc.
    methods
        function obj = WinNodeExecutor(systemExecutor,rosVersion)
            obj@ros.codertarget.internal.NodeExecutorBase(systemExecutor,rosVersion);
        end

        function runNode(obj,modelName,workspaceFolder,rosMasterURI,nodeHost)
          %runNode Run ROS node corresponding to modelName 

            % Get node executable
            nodeName = obj.getNodeNameFromModel(modelName);
            nodeExecutable = obj.getNodeExecutable(modelName,workspaceFolder);
            if ~isfile(nodeExecutable)
                error(message('ros:slros:rosdevice:NodeNotFound', nodeExecutable, modelName));
            end

            % Run node in a command window titled with nodeName. User
            % can see the output of the node executable directly on the
            % command window. Output is not piped to a logfile. Command
            % window is automatically closed upon termination of the node
            % excutable.
            if isequal(obj.ROSVersion,'ros')
                if ~isempty(nodeHost)
                    if obj.isIPAddress(nodeHost)
                        envVarName = 'ROS_IP';
                    else
                        envVarName = 'ROS_HOSTNAME';
                    end
                    cmd = ['title ' modelName ' && set ROS_MASTER_URI=' rosMasterURI ' && set ' ...
                        envVarName '=' nodeHost ' && "' nodeExecutable '" && exit(0) &'];
                else
                    cmd = ['title ' modelName ' && set ROS_MASTER_URI=' rosMasterURI ...
                        ' && "' nodeExecutable '" && exit(0) &'];
                end
                ros.internal.executeROSCommand(cmd);
            else
                % Use "ros2 run <package_name> <executable_name>" command
                installFolder = obj.SystemExecutor.fullfile(workspaceFolder,'install');
                currDir = cd(installFolder);
                cdToPwd = onCleanup(@()cd(currDir));
                [~,packageName] = fileparts(fileparts(nodeExecutable));
                cmd = ['run ' packageName ' ' nodeName ' & exit(0) &'];
                ros.ros2.internal.runros2cmd(cmd);
            end
        end

        function stopNode(obj,modelName)
            if isNodeRunning(obj,modelName)
                nodeName = obj.getNodeNameFromModel(modelName);
                if isequal(obj.ROSVersion,'ros')
                    % Kill node using 'rosnode kill' ROS command
                    cmd = ['rosnode kill ' nodeName];
                    status = ros.internal.executeROSCommand(cmd);
                    if status ~= 0
                        killNodeForcefully(obj,nodeName);
                    end
                else
                    killNodeForcefully(obj,nodeName);
                end
            end
        end

        function isRunning = isNodeRunning(obj,modelName)
            nodeName = obj.getNodeNameFromModel(modelName);
            appName = [nodeName '.exe'];
            cmd = sprintf('tasklist /FI "IMAGENAME eq %s" /FO list',appName);
            [status,result] = system(cmd);
            % If command fails, return not running
            isRunning = (status == 0) && contains(result,appName);
        end
        
        function nodeList = getAvailableNodes(obj,workspaceFolder)
            %getAvailableNodes Get list of runnable ROS nodes
            nodeList = cell(0,1);
            exeFolder = getExecutableFolder(obj,workspaceFolder);
            cmd = ['where /r "' exeFolder '" *.exe'];
            try
                output = strtrim(system(obj.SystemExecutor,cmd));
            catch
                output = '';
            end
            if isempty(output)
                return;
            end

            % Extract node candidates
            nodeCandidates = strsplit(output,'\n');
            for k = 1:length(nodeCandidates)
                [~,n] = fileparts(nodeCandidates{k});
                nodeCandidates{k} = n;
            end
            nodeCandidates = unique(nodeCandidates);

            % Verify against source folder
            srcFolder = obj.SystemExecutor.fullfile(workspaceFolder,'src');          
            for k = 1:numel(nodeCandidates)
                cmd = ['where /r "' srcFolder '" ' nodeCandidates{k} '.cpp'];  
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
        function nodeExecutable = getNodeExecutable(obj,modelName,workspaceFolder)
            nodeName = obj.getNodeNameFromModel(modelName);
            exeFolder = getExecutableFolder(obj,workspaceFolder);
            cmd = ['where /r "' exeFolder '" ' nodeName '.exe'];
            try
                output = strip(system(obj.SystemExecutor,cmd));
                nodeExecutable = strsplit(output,'\n');
                nodeExecutable = nodeExecutable{1};
            catch
                nodeExecutable = '';
            end
        end
        
        function [status,result] = killProcess(obj,procName)
            % killproc Kill a process
            if ~endsWith(procName,'.exe')
                procName = [procName '.exe'];
            end
            % Kill forcefully. Kill all child processes.
            cmd = sprintf('taskkill /F /T /IM %s',procName);
            try
                result = system(obj.SystemExecutor,cmd);
                status = 0;
            catch EX
                result = EX.message;
                status = 1;
            end
        end

        function killNodeForcefully(obj,nodeName)
            % Kill command window running the node
            cmd = ['for /f "tokens=2 delims=," %a in ('''...
                'tasklist /fi "imagename eq cmd.exe" /v /fo:csv /nh ^| '...
                'findstr /r /c:".*' nodeName '[^,]*$"'...
                ''') do taskkill /pid %a'];
            status = system(cmd);
            if status ~= 0
                [status,result] = killProcess(obj,nodeName);
                if status ~= 0
                    error(message('ros:slros:rosdevice:StopROSNodeError',...
                        nodeName,result));
                end
            end
        end
    end
end