function runApplication(varargin)
%This class is for internal use only. It may be removed in the future.

%RUNAPPLICATION Runs the ROS2 standalone application generated from
%Simulink model

% Copyright 2019-2021 The MathWorks, Inc.
if ros.codertarget.internal.isMATLABConfig(varargin{1})
    % MATLAB target load command arguments
    hCS = varargin{1};
    modelName = varargin{2};
    %exe = varargin{3};
    data.Runtime.BuildAction = hCS.Hardware.BuildAction;
    data.TargetHardware = hCS.Hardware.Name;
    data.ROS.RemoteBuild = ~isequal(hCS.Hardware.DeployTo,getString(message('ros:mlroscpp:codegen:Localhost')));
    exeFullPath = varargin{3};
else
    modelName = varargin{1};
    exeFullPath = varargin{2};
    data = codertarget.data.getData(getActiveConfigSet(modelName));
end

if isfield(data,'ROS') && isfield(data.ROS,'RemoteBuild') ...
        && islogical(data.ROS.RemoteBuild) && data.ROS.RemoteBuild
    % Remote build
    ros.codertarget.internal.DeploymentHooks.loadCommand(varargin{:})
else
    if isfield(data.Runtime, 'BuildAction')
        action = data.Runtime.BuildAction;
        if isequal(action, message('ros:slros2:codegen:ui_buildopts_buildaction_buildandrun').getString)
            % By default coder expects exe to be in folder where build started.
            % However, colcon tools generate the application in
            % ./install/<pkgname>/lib/<pkgname>/<AppName>
            installPath = fullfile(fileparts(exeFullPath), 'install');
            % verify that the "local_setup.bash/bat" scripts exist in the
            % folder
            localSetupScripts = dir(fullfile(installPath,'local_setup.*'));
            assert(~isempty(localSetupScripts),message('ros:slros2:codegen:LocalSetupNotFound',installPath))
            % get the package name from the modelName
            pkgName = ros.ros2.ProjectTool.getValidColconPackageName(modelName);
            % ros.ros2.internal.runros2cmd MATLAB function calls platform
            % specific run scripts which depend upon local_setup scripts being
            % present in the current working folder.
            currDir = cd(installPath);
            cdToPwd = onCleanup(@()cd(currDir));
            % Create command line arguments to be passed to "ros2" command
            % ros2 run <pkgName> <appName> <extraArgs>"
            if ispc
                % running a command in background creates a new ComamndWindow -
                % so append an extra "exit(0)" to close the current window once
                % the application is killed
                runCmdLine = sprintf(' run %s %s & exit(0) &', pkgName, modelName);
            else
                % On Unix the background task does not spawn a new shell - so
                % just killing the application should be enough
                runCmdLine = sprintf(' run %s %s &', pkgName, modelName);
            end
            ros.ros2.internal.runros2cmd(runCmdLine);
            % wait for the node to start - 30 s
            loc_waitForNodeToStart(modelName, 30);
        end
    end
end
end

function loc_waitForNodeToStart(modelName, timeout)
tstart = tic;
nodeName = ['/',modelName];
allNodes = ros.ros2.internal.NetworkIntrospection.getNodeNames();
while( (toc(tstart) < timeout) && (~ismember(nodeName,allNodes)) )
    allNodes = ros.ros2.internal.NetworkIntrospection.getNodeNames();
end
end

