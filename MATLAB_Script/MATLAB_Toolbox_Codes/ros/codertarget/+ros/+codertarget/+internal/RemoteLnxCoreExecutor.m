classdef RemoteLnxCoreExecutor < ros.codertarget.internal.CoreInterface
    %RemoteLnxCoreExecutor Implements CoreInterface for a remote Linux host

    % Copyright 2021 The MathWorks, Inc.
    properties (GetAccess = private, SetAccess = immutable)
        SystemExecutor
    end

    methods
        function obj = RemoteLnxCoreExecutor(systemExecutor)
            obj.SystemExecutor = systemExecutor;
        end
        
        function runCore(obj,rosFolder,catkinWorkspace)
        %runCore Start ROS core on device
        %   runCore(DEVICE) starts the ROS core on the device
        %   connected through the DEVICE object. The ROS master uses
        %   the default port number of 11311.
        %
        %   The version of the ROS core that is started is
        %   determined by the following heuristics:
        %   1. If ROSFolder contains a valid ROS installation folder,
        %      start the ROS core from there
        %   2. If the CatkinWorkspace is a valid workspace, start the
        %      ROS core based on the ROS installation that is associated
        %      with this workspace.
        %
        %   See also stopCore.

        % Check if roscore is running. Display an error if it is.
        % Note that we cannot determine on which port the existing
        % roscore is running, so only allow one instance.
            if obj.isCoreRunning
                disp(message('ros:slros:rosdevice:ROSCoreAlreadyRunning').getString);
                return;
            end

            if obj.isROSFolderValid(rosFolder)
                % 1. Check if ROS folder is valid. If so, launch ROS core from there.
                setupBash = [rosFolder '/setup.bash'];
            elseif obj.isCatkinWorkspaceValid(catkinWorkspace)
                % 2. Check if Catkin workspace is valid. If so, use the
                % setup.bash from there.
                setupBash = [catkinWorkspace '/devel/setup.bash'];
            else
                % We do not know where ROS is located
                error(message('ros:slros:rosdevice:UnknownROSFolder', rosFolder, catkinWorkspace));
            end

            % Now run roscore application
            [~,b] = fileparts(tempname);
            logFile = ['/tmp/roscore_' b '.log'];
            cmd = ['export ROS_MASTER_URI=http://' ...
                   obj.SystemExecutor.DeviceAddress ':11311;' ...     % Export the ROS_MASTER_URI
                   ' source ' setupBash ';' ...        % Source the setup.bash file we determined above
                   ' roscore &> ' logFile ...          % Run roscore and pipe output into log file
                   ' &'];                              % Put process in background

            system(obj.SystemExecutor, cmd);

            % Wait up to 5 seconds for ROS core to start up
            isCoreRunning = false;
            for i = 1:5
                isCoreRunning = obj.isCoreRunning;
                if isCoreRunning
                    break;
                else
                    pause(1);
                end
            end

            if ~isCoreRunning
                % If core did not start up, get tail of log file and display
                % an error.
                try
                    logOut = system(obj.SystemExecutor,['tail ' logFile]);
                catch
                    logOut = '';
                end
                error(message('ros:slros:rosdevice:ROSCoreDidNotStart', logOut));
            end
        end

        function stopCore(obj)
        %stopCore Stop ROS core on device
        %   stopCore(DEVICE) stops the ROS core on the device
        %   connected through the DEVICE object.
        %   If multiple roscore processes are running on the device,
        %   this function stops all of them.
        %
        %   If the core is not running, this function returns right
        %   away.
        %
        %   See also runCore.

        % Run command with 'sudo' if user has administrative
        % privileges. This enables killing roscore processes launched
        % by other users.

        % rosmaster and rosout might have been created by "roslaunch".
        % In that case, don't kill roslaunch, since that might affect
        % other running ROS nodes.

            try
                sudoCmd = obj.SystemExecutor.commandWithSudo('killall roscore rosmaster rosout');
                system(obj.SystemExecutor,sudoCmd);
            catch ex
                % Parse exception
                exMsg = string(ex.message);

                if exMsg.contains('Operation not permitted')
                    % The user does not have the correct privileges to kill
                    % at least one of the roscore processes
                    error(message('ros:slros:rosdevice:StopROSCoreNoPrivileges', 'current user'));
                elseif exMsg.contains('no process found')
                    % This is okay, since all roscore processes are already dead.
                    % Silently swallow this exception.
                else
                    % Throw generic error if something else went wrong
                    rethrow(ex);
                end
            end
        end
        
        function isRunning = isCoreRunning(obj)
        %isCoreRunning Determine if ROS core is running on device
        %   ISRUNNING = isCoreRunning(DEVICE) returns TRUE if the ROS
        %   core is running on the device connected through the DEVICE
        %   object. The function returns FALSE if the core is not
        %   running on the device.

        % There are two ways in which a ROS core could be active
        % - The user called roscore, so the following processes are
        %   active: roscore, rosmaster, rosout.
        % - The user called roslaunch and no other ROS core was
        %   running. In that case, the following processes are active:
        %   roslaunch, rosmaster, rosout.

            isROSMasterRunning = obj.isProcessRunning('rosmaster', false) && ...
                obj.isProcessRunning('rosout', false);

            isRoscoreRunning = obj.isProcessRunning('roscore', false) && ...
                isROSMasterRunning;

            isRoslaunchRunning = obj.isProcessRunning('roslaunch', false) && ...
                isROSMasterRunning;

            isRunning = isRoscoreRunning || isRoslaunchRunning;
        end
    end
    
    methods (Access = protected)
        function isWsValid = isCatkinWorkspaceValid(obj, catkinWs)
        %isCatkinWorkspaceValid Check if given folder contains a valid Catkin workspace
        %   The user creates the Catkin workspace with
        %   catkin_init_workspace inside the <workspace_root>/src
        %   folder. This will create the src/CMakeLists.txt file. Check
        %   for the existence of that file.
        %   We also enforce that the user called catkin_make on top of
        %   the workspace, since we rely on the "setup.bash" sourcing
        %   at deployment time.

            isWsValid = obj.doesFileExist([catkinWs '/src/CMakeLists.txt']) && ...
                obj.doesFileExist([catkinWs '/devel/setup.bash']);
        end
        
        function folderExists = doesFolderExist(obj, folderName)
        %doesFolderExist Checks if folder with given name exists
        %   Returns TRUE if folder exists and FALSE otherwise.

            try
                % Use the '-d' option to check for directory existence
                system(obj.SystemExecutor, ['[ -d ' handleSpaces(folderName) ' ]']);

                % If the command executes successfully, the folder exists
                folderExists = true;
            catch ex
                % If something unexpected goes wrong, e.g. SSH connection fails,
                % rethrow the associated exception
                if ~strcmp(ex.identifier, 'utils:sshclient:system')
                    rethrow(ex);
                end

                % Directory does not exist
                folderExists = false;
            end
        end
        
        function fileExists = doesFileExist(obj, fileName)
        %doesFileExist Checks if file with given name exists
        %   Returns TRUE if file exists and FALSE otherwise.

            try
                % Use the '-f' option to check for existence of regular files
                system(obj.SystemExecutor, ['[ -f ' handleSpaces(fileName) ' ]']);

                % If the command executes successfully, the folder exists
                fileExists = true;
            catch ex
                % If something unexpected goes wrong, e.g. SSH connection fails,
                % rethrow the associated exception
                if ~strcmp(ex.identifier, 'utils:sshclient:system')
                    rethrow(ex);
                end

                % File does not exist if exception 'utils:sshclient:system' is thrown
                fileExists = false;
            end
        end
        
        function isFolderValid = isROSFolderValid(obj, rosFolder)
        %isROSFolderValid Check if given folder contains a ROS distribution
        %   This is a simple test that only checks if the setup.bash
        %   file exists in the right place.

        % The existence test can handle multiple forward slashes,
        % so it is safe to concatenate.
            isFolderValid = obj.doesFolderExist(rosFolder) && ...
                obj.doesFileExist([rosFolder '/setup.bash']);
        end
        
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
                pgrepOutput = system(obj.SystemExecutor,['pgrep ' args ' ' processName]);
                
                % We have to use str2num to allow conversion of multiple PIDs
                pid = str2num(pgrepOutput); %#ok<ST2NM>
                isRunning = true;
            catch
                pid = [];
                isRunning = false;
            end
        end
    end
end

function escapedPath = handleSpaces(filePath)
%handleSpaces Handle spaces in file or folder path
%   In the system command that we send over SSH, spaces in the
%   path to a file or a folder need to be escaped with a backslash.

escapedPath = strrep(filePath, ' ', '\ ');

% If the spaces were already escaped, we should undo the
% double-escaping
escapedPath = strrep(escapedPath, '\\ ', '\ ');
end