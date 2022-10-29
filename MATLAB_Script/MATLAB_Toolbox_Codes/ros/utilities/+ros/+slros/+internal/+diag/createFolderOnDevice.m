function returnInfo = createFolderOnDevice( hostname, sshPort, username, password, folderPath )
%This function is for internal use only. It may be removed in the future.

%createFolderOnDevice Connect to the ROS device and create a folder in the given path
%   This function is called by the "FixIt" diagnostic action if the Catkin
%   workspace does not exist. If the ROS base folder is invalid as well,
%   then the best we can do is create the folder for the user.

%   Copyright 2016-2020 The MathWorks, Inc.

% Connect to the device
    diag = ros.slros.internal.diag.DeviceDiagnostics;
    diag.connect(hostname, sshPort, username, password);

    % Create the folder. This will throw an error if something goes wrong.
    diag.createFolder(folderPath);

    % The folder was created successfully
    returnInfo = message('ros:slros:devicediag:FolderCreateSuccess', folderPath).getString;

end
