function returnInfo = recoverROS2FolderFromStdLocation( hostname, sshPort, username, password, modelName )
%This function is for internal use only. It may be removed in the future.

%recoverROS2FolderFromStdLocation Recover the ROS 2 installation folder from standard locations
%   This function is called by the "FixIt" diagnostic action if the ROS 2
%   folder is not valid and the user wants to search the standard
%   installation locations.

%   Copyright 2020 The MathWorks, Inc.

    if nargin < 5
        modelName = '';
    end
    % Connect to the device
    diag = ros.slros.internal.diag.DeviceDiagnostics(modelName,'ROS2');
    connect(diag, hostname, sshPort, username, password);

    % Recover the set of ROS folders that denote standard locations.
    % This will return empty if the operation fails.
    ros2Folders = recoverROS2FolderFromStandardLocations(diag);

    if isempty(ros2Folders)
        % Did not find a install ROS folder. Throw an exception.
        error(message('ros:slros:devicediag:ROS2FolderStdRecoveryFailure'));
    end

    defaultROS2FolderIndex = length(ros2Folders); % Latest distribution

    % Found at least 1 ROS 2 installation folder. Enumerate them.
    msgArray = cell(length(ros2Folders), 1);
    for i = 1:length(ros2Folders)
        ros2Folder = ros2Folders{i};
        distName = getROSDistribution(diag, ros2Folder);
        if i == defaultROS2FolderIndex
            ros.slros.internal.dlg.DeviceParameterSpecifier.setROS2InstallFolder(modelName, ros2Folder);
            msgArray{i} = message('ros:slros:devicediag:ROS2FolderRecoverySuccess', upper(distName), ros2Folder).getString;
        else
            msgArray{i} = message('ros:slros:devicediag:ROS2FolderRecoverySuccessNondefault', upper(distName), ros2Folder).getString;
        end
    end

    % Introduce a newline before printing the message for each rosfolder
    returnInfo = sprintf('\n%s', msgArray{end:-1:1}); % Reverse to reflect priority
end
