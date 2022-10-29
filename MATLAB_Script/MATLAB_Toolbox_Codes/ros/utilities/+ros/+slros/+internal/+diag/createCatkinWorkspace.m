function returnInfo = createCatkinWorkspace( hostname, sshPort, username, password, rosFolder, catkinWs )
%This function is for internal use only. It may be removed in the future.

%createCatkinWorkspace Create a new Catkin workspace in the desired folder
%   This function is called by the "FixIt" diagnostic action if the Catkin
%   workspace does not exist. If the ROS base folder is valid, we can use
%   it to call catkin_init_workspace and catkin_make.

%   Copyright 2016-2020 The MathWorks, Inc.

% Connect to the device
    diag = ros.slros.internal.diag.DeviceDiagnostics;
    diag.connect(hostname, sshPort, username, password);

    % Create workspace. This throws an error if something goes wrong
    diag.createCatkinWorkspace(rosFolder, catkinWs);

    % The workspace was created successfully
    returnInfo = message('ros:slros:devicediag:CatkinWsSuccess', catkinWs).getString;

end
