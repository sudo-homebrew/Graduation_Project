function [data, info] = frontier
%Frontier gives an empty data for explorer/Frontier

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'explorer/Frontier';
[data.Id, info.Id] = ros.internal.ros.messages.ros.default_type('int64',1);
[data.DetectedByRobot, info.DetectedByRobot] = ros.internal.ros.messages.ros.default_type('int64',1);
[data.RobotHomePositionX, info.RobotHomePositionX] = ros.internal.ros.messages.ros.default_type('double',1);
[data.RobotHomePositionY, info.RobotHomePositionY] = ros.internal.ros.messages.ros.default_type('double',1);
[data.XCoordinate, info.XCoordinate] = ros.internal.ros.messages.ros.default_type('double',1);
[data.YCoordinate, info.YCoordinate] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'explorer/Frontier';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'id';
info.MatPath{2} = 'detected_by_robot';
info.MatPath{3} = 'robot_home_position_x';
info.MatPath{4} = 'robot_home_position_y';
info.MatPath{5} = 'x_coordinate';
info.MatPath{6} = 'y_coordinate';
