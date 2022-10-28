function returnInfo = createROS2Workspace( hostname, sshPort, username, password, ros2Folder, ros2Ws )
%This function is for internal use only. It may be removed in the future.

%createROS2Workspace Create a new ROS 2 workspace in the desired folder
%   This function is called by the "FixIt" diagnostic action if the ROS 2
%   workspace does not exist. If the ROS 2 base folder is valid, it is used
%   to call ''colcon build''.

%   Copyright 2020 The MathWorks, Inc.

% Connect to the device
    diag = ros.slros.internal.diag.DeviceDiagnostics('ROS2');
    connect(diag, hostname, sshPort, username, password);
    
    % Create workspace. This throws an error if something goes wrong
    createROS2Workspace(diag, ros2Folder, ros2Ws);

    % The workspace was created successfully
    returnInfo = message('ros:slros:devicediag:ROS2WsSuccess', ros2Ws).getString;

end
