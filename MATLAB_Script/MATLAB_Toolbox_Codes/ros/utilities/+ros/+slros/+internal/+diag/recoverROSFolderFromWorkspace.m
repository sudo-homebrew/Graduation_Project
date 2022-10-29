function returnInfo = recoverROSFolderFromWorkspace( hostname, sshPort, username, password, catkinWs, modelName )
%This function is for internal use only. It may be removed in the future.

%recoverROSFolderFromWorkspace Recover the ROS installation folder from a Catkin workspace
%   This function is called by the "FixIt" diagnostic action if the ROS
%   folder is not valid and the user wants to use the Catkin workspace
%   to recover it.

%   Copyright 2016-2020 The MathWorks, Inc.
    if nargin < 6
        modelName = '';
    end

    % Connect to device
    diag = ros.slros.internal.diag.DeviceDiagnostics(modelName);
    diag.connect(hostname, sshPort, username, password);

    % Recover the ROS folder. This will return empty if the operation fails.
    rosFolder = diag.recoverROSFolderFromCatkinWorkspace(catkinWs);

    if ~isempty(rosFolder)
        % Found ROS installation folder
        distName = diag.getROSDistribution(rosFolder);
        ros.slros.internal.dlg.DeviceParameterSpecifier.setROSInstallFolder(modelName, rosFolder);
        returnInfo = message('ros:slros:devicediag:ROSFolderRecoverySuccess', upper(distName), rosFolder).getString;
    else
        % Did not find a install ROS folder. Throw an exception.
        error(message('ros:slros:devicediag:ROSFolderCatkinRecoveryFailure', catkinWs));
    end

end
