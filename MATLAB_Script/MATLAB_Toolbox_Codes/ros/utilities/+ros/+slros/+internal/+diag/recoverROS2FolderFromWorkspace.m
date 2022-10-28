function returnInfo = recoverROS2FolderFromWorkspace( hostname, sshPort, username, password, ros2Ws, modelName )
%This function is for internal use only. It may be removed in the future.

%recoverROS2FolderFromWorkspace Recover the ROS 2 installation folder from a ROS 2 (colcon) workspace
%   This function is called by the "FixIt" diagnostic action if the ROS
%   folder is not valid and the user wants to use the Catkin workspace
%   to recover it.

%   Copyright 2020 The MathWorks, Inc.
    if nargin < 6
        modelName = '';
    end

    % Connect to device
    diag = ros.slros.internal.diag.DeviceDiagnostics(modelName,'ROS2');
    connect(diag, hostname, sshPort, username, password);

    % Recover the ROS 2 folder. This will return empty if the operation fails.
    ros2Folder = recoverROS2FolderFromROS2Workspace(diag, ros2Ws);

    if ~isempty(ros2Folder)
        % Found ROS installation folder
        distName = getROSDistribution(diag, ros2Folder);
        ros.slros.internal.dlg.DeviceParameterSpecifier.setROS2InstallFolder(modelName, ros2Folder);
        returnInfo = message('ros:slros:devicediag:ROS2FolderRecoverySuccess', upper(distName), ros2Folder).getString;
    else
        % Did not find a install ROS folder. Throw an exception.
        error(message('ros:slros:devicediag:ROS2FolderROS2WsRecoveryFailure', ros2Ws));
    end

end
