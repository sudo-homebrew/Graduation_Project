function returnInfo = recoverROSFolderFromStdLocation( hostname, sshPort, username, password, modelName )
%This function is for internal use only. It may be removed in the future.

%recoverROSFolderFromStdLocation Recover the ROS installation folder from standard locations
%   This function is called by the "FixIt" diagnostic action if the ROS
%   folder is not valid and the user wants to search the standard
%   installation locations.

%   Copyright 2016-2020 The MathWorks, Inc.
    if nargin < 5
        modelName = '';
    end

    % Connect to the device
    diag = ros.slros.internal.diag.DeviceDiagnostics(modelName);
    diag.connect(hostname, sshPort, username, password);

    % Recover the set of ROS folders that denote standard locations.
    % This will return empty if the operation fails.
    rosFolders = diag.recoverROSFolderFromStandardLocations;

    if isempty(rosFolders)
        % Did not find a install ROS folder. Throw an exception.
        error(message('ros:slros:devicediag:ROSFolderStdRecoveryFailure'));
    end

    defaultROSFolderIndex = length(rosFolders); % Latest distribution

    % Found at least 1 ROS installation folder. Enumerate them.
    msgArray = cell(length(rosFolders), 1);
    for i = 1:length(rosFolders)
        rosFolder = rosFolders{i};
        distName = diag.getROSDistribution(rosFolder);
        if i == defaultROSFolderIndex
            ros.slros.internal.dlg.DeviceParameterSpecifier.setROSInstallFolder(modelName, rosFolder);
            msgArray{i} = message('ros:slros:devicediag:ROSFolderRecoverySuccess', upper(distName), rosFolder).getString;
        else
            msgArray{i} = message('ros:slros:devicediag:ROSFolderRecoverySuccessNondefault', upper(distName), rosFolder).getString;
        end
    end

    % Introduce a newline before printing the message for each rosfolder
    returnInfo = sprintf('\n%s', msgArray{end:-1:1}); % Reverse to reflect priority
end
