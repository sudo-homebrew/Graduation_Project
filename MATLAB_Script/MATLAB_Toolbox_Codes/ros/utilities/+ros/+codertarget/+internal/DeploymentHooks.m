classdef DeploymentHooks < ros.slros.internal.InternalAccess
    %This class is for internal use only. It may be removed in the future.
    
    %DeploymentHooks Code hooks for deployment of Simulink model to ROS target
    %   Hooks are callbacks that are executed by the coder targets
    %   infrastructure when the ROS node is generated. For example, there is
    %   a callback when the build starts (`buildEntry`) and
    %   a callback that is executed once the node archive is created and should
    %   be loaded to the ROS target (`loadCommand`).
    
    %   Copyright 2016-2021 The MathWorks, Inc.
    
    methods (Static)
        function loadCommand(varargin)
            %LOADCOMMAND Executes as the load command for ROS model build.
            %   At this point, the source code archive and the build shell script are
            %   both available.
            %   The load command will copy these files to the ROS device, build the ROS
            %   node there, and (optionally) run the ROS node.
            %   Note that this function is not called if the user selected the
            %   "Code Generation Only" checkbox.
            
            if ros.codertarget.internal.isMATLABConfig(varargin{1})
                % MATLAB target load command arguments
                hCS = varargin{1};
                modelName = varargin{2};
                %exe = varargin{3};
                data.Runtime.BuildAction = hCS.Hardware.BuildAction;
                data.TargetHardware = hCS.Hardware.Name;
                data.ROS.RemoteBuild = ~isequal(hCS.Hardware.DeployTo,getString(message('ros:mlroscpp:codegen:Localhost')));
                modelRefNames = [];
                isMATLABCodegen = true;
            else
                modelName = varargin{1};
                data = codertarget.data.getData(getActiveConfigSet(modelName));
                % @todo update the usage of edit-time filter filterOutCodeInactiveVariantSubsystemChoices()
                % instead use the post-compile filter codeCompileVariants() - g2604300
                modelRefNames = find_mdlrefs(modelName ,'MatchFilter',@Simulink.match.internal.filterOutCodeInactiveVariantSubsystemChoices); % look only inside code active choice of VSS
                % Ignore the last name, since it's the top-level model name
                modelRefNames(end) = [];
                isMATLABCodegen = false;
            end
            depObj = ros.codertarget.internal.DeploymentHooks;
            supportedBoards = ros.codertarget.internal.getSupportedHardwareBoards;
            
            % Check hardware matches supported options
            if ~ismember(data.TargetHardware, supportedBoards)
                error(message('ros:slros:deploy:UnknownTargetHardware', ...
                    strjoin(supportedBoards,","), data.TargetHardware));
            end
            
            % Retrieve build action. This can either be "None", "Build and load", or "Build and run"
            buildAction = data.Runtime.BuildAction;
            if ~startsWith(buildAction,'build','IgnoreCase',true)
                % If the user did not request a build, do not connect to the ROS
                % device. The final artifacts are the tgz and sh files.
                return;
            end
            
            if isequal(getString(message('ros:slros:cgen:ui_hwboard')),data.TargetHardware) && ...
                    isfield(data.ROS, 'RemoteBuild') && ~data.ROS.RemoteBuild
                % Deploy ROS node locally using MATLAB ROS distribution
                depObj.runLocalROSNode(modelName, buildAction, varargin{2:end})
                return;
            end
            [rosVer, buildScriptShell, setupScriptShell] = depObj.getROSVersionString(data);
            if isMATLABCodegen
                validHost = hCS.Hardware.RemoteDeviceAddress;
                validUser = hCS.Hardware.RemoteDeviceUsername;
                validPassword = hCS.Hardware.RemoteDevicePassword;
                validSSH = 22;
                rosVerOpts = categorical({'ros','ros2'});
                rosVerIdx = (rosVerOpts==rosVer);
                wsFolderProps =  {'CatkinWorkspace','ROS2Workspace'};
                validRemoteWorkspace = hCS.Hardware.(wsFolderProps{rosVerIdx});                
            else
                % Verify connection settings
                [validHost, validSSH, validUser, validPassword, validRemoteWorkspace] = ...
                    depObj.verifyConnectionSettings(rosVer, modelName, buildAction);
                % Open new stage in diagnostic viewer and close it when function exits or
                % if an error occurs
                buildStage = sldiagviewer.createStage(buildAction, 'ModelName', modelName);
                stageCleanup = onCleanup(@() delete(buildStage));
            end
            
            % Create ROS/ROS2 connection
            rosTarget = depObj.getConnectionObject(rosVer, validHost, validUser, ...
                validPassword, validSSH, validRemoteWorkspace);
            
            % We need to first kill any instance(s) of the ROS node with
            % the same name before building. Otherwise the executable will be
            % locked.
            if isNodeRunning(rosTarget, modelName)
                disp(message('ros:slros:deploy:StopRunningNode', modelName).getString);
                disp(ros.slros.internal.diag.DeviceDiagnostics.StepSeparator);              
                try
                    stopNode(rosTarget, modelName);
                catch
                end
            end
            
            % Find all archives for main model and all model references
            modelNames = {modelName};
            
            hasModelRefs = ~isempty(modelRefNames);
            mdlRefListFile = strcat(modelName, 'ModelRefs.txt');
            
            modelNames = [modelNames; modelRefNames];
            
            % Create file listing all the model references
            if hasModelRefs
                mdlRefList = StringWriter;
                for i = 1:numel(modelRefNames)
                    mdlRefList.addcr(strcat(modelRefNames{i}, '.tgz'));
                end
                mdlRefList.write(mdlRefListFile);
            end
            
            if isMATLABCodegen
                modelUnchangedAndNodeExists = false;
                archives = strcat(modelNames, '.tgz');
            else
                modelUnchangedAndNodeExists = depObj.verifyModelUnchanged(rosVer, rosTarget, modelName);
                archives = fullfile('..', strcat(modelNames, '.tgz'));
            end
                
            if modelUnchangedAndNodeExists
                disp(message('ros:slros:deploy:NodeUpToDate', modelName).getString);
                % Still run node, if specified by user build action
                ros.codertarget.internal.DeploymentHooks.runROSNode(rosVer, rosTarget, modelName, buildAction, isMATLABCodegen);
                return;
            end
            
            % 1. Transfer build script and the Simulink model to the Catkin workspace
            % 2. Build Simulink node
            % If the "rosTarget.CatkinWorkspace" folder contains spaces, they are
            % already escaped, so I can simply concatenate it with our file names.
            
            % Transfer build scripts
            disp(message('ros:slros:deploy:TransferCode', modelName).getString);
            shellScriptWildcard = 'deploy_*.sh';
            backgroundScript = [validRemoteWorkspace '/deploy_background_build.sh'];
            buildScript = [validRemoteWorkspace buildScriptShell];
            
            buildScriptWildCard = [validRemoteWorkspace '/deploy_build_ros*_model.sh'];
            buildScripts = fullfile(toolboxdir('ros'), 'codertarget', 'src', shellScriptWildcard);
            putFile(rosTarget, buildScripts, validRemoteWorkspace);
            
            % Make the shell scripts executable
            system(rosTarget, ['chmod u+x ' validRemoteWorkspace '/' shellScriptWildcard]);
            
            % Transfer all Simulink model archives (main model + model references)
            for i = 1:numel(archives)
                putFile(rosTarget, archives{i}, validRemoteWorkspace);
            end
            
            % Transfer text file listing all model references
            if hasModelRefs
                putFile(rosTarget, mdlRefListFile, validRemoteWorkspace);
            end
            
            % Start build process and get the pid
            buildLog = [validRemoteWorkspace '/' modelName '_build.log'];
            buildStat = [validRemoteWorkspace '/' modelName '_build.stat'];
            
            % Without redirecting stdout and stderr to /dev/null,
            % the command does not return
            disp(message('ros:slros:deploy:StartBuild').getString);
            disp(ros.slros.internal.diag.DeviceDiagnostics.StepSeparator);
            
            pid = system(rosTarget, ...
                ['source ' validRemoteWorkspace setupScriptShell '; '...
                backgroundScript ' ' buildScript ' ' ...
                [validRemoteWorkspace '/' modelName] ' ' validRemoteWorkspace ' &>/dev/null & echo $!']);
            logSize = 0;
            done = false;
            while ~done
                try
                    % Kill -0 checks for existence of the process ID (it does not do
                    % any killing)
                    system(rosTarget, ['kill -0 ' pid]);
                catch
                    % The process does not exist anymore
                    done = true;
                end
                
                try
                    % Get the output of the buildLog (this is executed continuously)
                    output = system(rosTarget, ['stat --format=%s ' buildLog]);
                catch
                    % Log file is deleted
                    continue;
                end
                
                % Display running log in the diagnostic viewer
                currLogSize = str2double(output);
                if ~isnan(currLogSize) && (currLogSize > logSize)
                    try
                        % Print out new characters output to the log file since last
                        % visit
                        logOut = system(rosTarget,sprintf('tail -c +%d %s',logSize,buildLog));
                        logSize = logSize + numel(logOut);
                        disp(logOut);
                    catch
                    end
                end
                
                % Update build status every second
                pause(1);
            end
            
            try
                % wait for the buildStatus file to be generated on remote
                % device
                pause(2);
                status = str2double(system(rosTarget,['cat ' buildStat]));
            catch
                % Something went seriously wrong if the buildStat file does not exist
                status = 1;
            end
            
            if ~isnan(status) && (status ~= 0)
                error(message('ros:slros:deploy:BuildError', modelName))
            end
            
            % Delete the build scripts if there were no build errors
            depObj.systemNoException(rosTarget, ...
                ['rm -f ' backgroundScript ' ' buildScriptWildCard ' ' buildLog ' ' buildStat]);
            
            % Run node, if specified by user build action
            depObj.runROSNode(rosVer, rosTarget, modelName, buildAction, isMATLABCodegen);
        end
        
        function args = getLoadCommandArgs(varargin)
            %GETLOADCOMMANDARGS Return load command arguments.
            if nargin < 1
                args = bdroot;
            else
                % This tells codertarget infra-structure to pass
                %varargin{1} -> hCS
                %varargin{2} -> fcnName
                %varargin{3} -> exe
                args = [];
            end
        end
    end
    
    methods (Static, Access = private)
        function [validHost, validSSH, validUser, validPassword, validWorkspace] = ...
                verifyConnectionSettings(rosVer, modelName, buildAction)
            %verifyConnectionSettings Verify the connection settings to the ROS device
            %   Return valid settings that can be used for the actual deployment step.
            
            % Get the latest parameters from MATLAB preferences
            deviceParams = ros.codertarget.internal.DeviceParameters;
            [hostname, sshPort, username, password, ~, catkinWs, rosFolder, ...
                ros2Ws, ros2Folder] = deviceParams.getDeviceParameters;
            rosVerOpts = categorical({'ros','ros2'});
            rosVerIdx = (rosVerOpts==rosVer);
            installFoldersIn = {rosFolder, ros2Folder};
            wsFoldersIn = {catkinWs, ros2Ws};
            installFolder = installFoldersIn{rosVerIdx};
            wsFolder = wsFoldersIn{rosVerIdx};
            % Run the diagnostics in build mode. Each high-priority warning will result
            % in an error.
            diag = ros.slros.internal.diag.DeviceDiagnostics(modelName,rosVer);
            switch lower(buildAction)
                case 'build and load'
                    diag.RunMode = 'build';
                case 'build and run'
                    % The user explicitly requested to run the resulting ROS node
                    diag.RunMode = 'buildrun';
                otherwise
                    % By default, stick with the build mode
                    diag.RunMode = 'build';
            end
            
            % Now run a loop until we have valid connection settings.
            % If there are problems, display the connection dialog and let the user
            % make adjustments to the settings.
            notDone = true;
            while notDone
                % Run the diagnostics
                hasError = diag.runDiagnostics(hostname, sshPort, username, password, installFolder, wsFolder);
                if ~hasError
                    % If diagnostics run without problems, get out of this loop
                    break;
                end
                % If there are diagnostic errors, pop up the settings dialog and let
                % the user make adjustments.
                % This call is blocking until the user closes the dialog.
                userDlg = ros.slros.internal.dlg.DeviceParameterSpecifier;
                userDlg.ModelName = modelName;
                [isAccepted, hostname, sshPort, username, password, retRosFolder, retCatkinWs, retRos2Folder, retRos2Ws] = ...
                    userDlg.openDialogAndWait;
                % If the user hit "Cancel" or closed the dialog, abort the build.
                assert(isAccepted,'ros:slros:deploy:NoValidSettings',...
                    message('ros:slros:deploy:NoValidSettings', hostname, modelName).getString)
                installFoldersRet = {retRosFolder, retRos2Folder};
                wsFoldersRet = {retCatkinWs, retRos2Ws};
                installFolder = installFoldersRet{rosVerIdx};
                wsFolder = wsFoldersRet{rosVerIdx};
                % Retry the settings that the user made in the next loop iteration
                notDone = hasError;
            end
            % Initial settings have been validated. Store them.
            validHost = hostname;
            validSSH = sshPort;
            validUser = username;
            validPassword = password;
            validWorkspace = diag.handleSpaces(wsFolder);
        end
        
        function runLocalROSNode(nodeName, buildAction, varargin)
            if contains(buildAction, 'run')
                disp(ros.slros.internal.diag.DeviceDiagnostics.StepSeparator);
                disp(message('ros:slros:deploy:RunNode').getString);
                rosMaster = ros.slros.internal.sim.ROSMaster();
                % Cannot start the node without a valid ROS_MASTER connection
                verifyReachable(rosMaster);
                rosMasterURI = rosMaster.MasterURI;
                develPath = fullfile(fileparts(varargin{1}),'devel');
                pkgName = ros.codertarget.internal.ProjectTool.getValidPackageName(nodeName);
                currDir = cd(fullfile(develPath));
                changeToCurrDir = onCleanup(@()cd(currDir));
                origRosMasterUriEnv = getenv('ROS_MASTER_URI');
                resetROSMASTERURIEnv = onCleanup(@()setenv('ROS_MASTER_URI',origRosMasterUriEnv));
                setenv('ROS_MASTER_URI',rosMasterURI);
                runCmdsMap = containers.Map({'win64','glnxa64','maci64'},...
                    {sprintf('lib\\%s\\%s & exit(0) &',pkgName,nodeName),...
                    sprintf('./lib/%s/%s &',pkgName,nodeName),...
                    sprintf('./lib/%s/%s &',pkgName,nodeName)});
                runCmd = runCmdsMap(computer('arch'));
                if ismac
                    % MACOSX (ros.internal.runroscmd MATLAB system call does not
                    % inherit the shell variables - so use automation with
                    % osascript to run the ROS Node
                    [~, res] = ros.internal.roscmdshell();
                    resString = string(res);
                    sourceShell = char(extractBetween(resString,'''',''''));
                    osaCmd =  sprintf(['/usr/bin/osascript -e ', ...
                        '''do shell script " %s;',...
                        ' ./lib/%s/%s > /dev/null 2>&1 &" '''], ...
                        sourceShell, pkgName,nodeName);
                    system(osaCmd);
                else
                    ros.internal.runroscmd(runCmd);
                end
                % wait for the node to start - 30 s
                waitForNodeToStart(nodeName,rosMasterURI,30);
            end            
            function waitForNodeToStart(nodeName,rosMasterURI,timeout)
                tstart = tic;
                nodeNameWithPath = ['/',nodeName];
                while(toc(tstart) < timeout)
                    allNodes = ros.internal.NetworkIntrospection.getNodeNames(rosMasterURI);
                    % Add a small pause to avoid spinning the CPU
                    pause(0.2);
                    if (ismember(nodeNameWithPath,allNodes))
                        break;
                    end
                end
            end
        end
        
        function modelUnchangedNodePresent = verifyModelUnchanged(inputROSVer, rosConnObj, modelName)
            % If model did not change and node executable exists on target,
            % there is no need to regenerate it.
            modelUnchangedNodePresent = false;
            if ~ros.codertarget.internal.hasModelChanged
                rosVer = validatestring(inputROSVer,{'ros','ros2'});
                nodeExistFcnMap = containers.Map();
                nodeExistFcnMap('ros') = @(rosObj, mdl)contains(string(rosObj.AvailableNodes), lower(mdl));
                nodeExistFcnMap('ros2') = @(rosObj, mdl)contains(string(rosObj.AvailableNodes), mdl);
                nodeExistFcn = nodeExistFcnMap(rosVer);
                modelUnchangedNodePresent = feval(nodeExistFcn, rosConnObj, modelName);
            end
        end
        
        function rosTarget = getConnectionObject(inputROSVer, validHost, validUser, ...
                validPassword, validSSH, validRemoteWorkspace)
            rosVer = validatestring(inputROSVer,{'ros','ros2'});
            if strcmpi(rosVer,'ros2')
                % update message catalog
                disp(message('ros:slros:deploy:ConnectToROS2Device', validHost).getString);
                rosTarget = ros2device(validHost, validUser, validPassword, validSSH);
                disp(message('ros:slros:deploy:UseROS2Ws', validRemoteWorkspace).getString);
                rosTarget.ROS2Workspace = validRemoteWorkspace;
            else
                disp(message('ros:slros:deploy:ConnectToDevice', validHost).getString);
                rosTarget = rosdevice(validHost, validUser, validPassword, validSSH);
                disp(message('ros:slros:deploy:UseCatkin', validRemoteWorkspace).getString);
                rosTarget.CatkinWorkspace = validRemoteWorkspace;
            end
            disp(ros.slros.internal.diag.DeviceDiagnostics.StepSeparator);
        end
        
        function [rosVer, buildScriptSH, setupScriptSH] = getROSVersionString(data)
            isROS2 = strcmp(message('ros:slros2:codegen:ui_hwboard').getString, data.TargetHardware);
            rosVerOpts = {'ros','ros2'};
            buildScriptSHOpts = {'/deploy_build_ros_model.sh','/deploy_build_ros2_model.sh'};
            setupScriptSHOpts = {'/devel/setup.bash','/install/setup.bash'};
            idx = xor([true, false], isROS2);
            rosVer = rosVerOpts{idx};
            buildScriptSH = buildScriptSHOpts{idx};
            setupScriptSH = setupScriptSHOpts{idx};
        end
        
        function runROSNode(rosVer, rosTarget, modelName, buildAction, isMATLABConfig)
            if ros.codertarget.internal.Util.isROSControlEnabled(modelName)
                disp('### Nothing to run for ros_control plugin.');
                return;
            end
            if contains(buildAction, 'run')
                disp(ros.slros.internal.diag.DeviceDiagnostics.StepSeparator);
                if strcmpi(rosVer,'ros2')
                    disp(message('ros:slros:deploy:RunROS2Node').getString);
                    disp(ros.slros.internal.diag.DeviceDiagnostics.StepSeparator);
                    disp(message('ros:slros:deploy:UseROS2DEVICE').getString);
                    % Collect port information only for Simulink
                    if isMATLABConfig
                        runRemoteNode(rosTarget.NodeExecutor,...
                            modelName,...
                            rosTarget.ROS2Workspace,...
                            '',...  % ROSMasterURI
                            '');    % NodeHost
                    else
                        port = codertarget.attributes.getExtModeData('Port', getActiveConfigSet(bdroot(modelName)));
                        runRemoteNode(rosTarget.NodeExecutor,...
                            modelName,...
                            rosTarget.ROS2Workspace,...
                            '',...  % ROSMasterURI
                            '',...  % NodeHost
                            [' -port ' num2str(port)]);  % cmdArgs
                    end
                else
                    disp(message('ros:slros:deploy:RunNode').getString);
                    % Run Simulink node. Set MasterURI automatically for now
                    rosMasterURI = ros.slros.internal.sim.defaultSimMasterURI(rosTarget.DeviceAddress);
                    disp(ros.slros.internal.diag.DeviceDiagnostics.StepSeparator);
                    disp(message('ros:slros:deploy:NodeMasterHostConnection', ...
                        rosMasterURI, rosTarget.DeviceAddress).getString);
                    disp(message('ros:slros:deploy:UseRosdevice').getString);
                    if isMATLABConfig
                        runRemoteNode(rosTarget.NodeExecutor,...
                            modelName,...
                            rosTarget.CatkinWorkspace,...
                            rosMasterURI,...          % ROSMasterURI
                            rosTarget.DeviceAddress); % NodeHost
                    else
                        % Collect port information only for Simulink 
                        port = codertarget.attributes.getExtModeData('Port', getActiveConfigSet(bdroot(modelName)));
                        runRemoteNode(rosTarget.NodeExecutor,...
                            modelName,...
                            rosTarget.CatkinWorkspace,...
                            rosMasterURI,...             % ROSMasterURI
                            rosTarget.DeviceAddress,...  % NodeHost
                            [' -port ' num2str(port)]);  % cmdArgs
                    end
                end
            end
        end
        
        function output = systemNoException(hw,cmd,varargin)
            %systemNoException Make a system call and swallow all exceptions
            output = [];
            try
                output = system(hw,cmd,varargin{:});
            catch
            end
        end
    end
end
