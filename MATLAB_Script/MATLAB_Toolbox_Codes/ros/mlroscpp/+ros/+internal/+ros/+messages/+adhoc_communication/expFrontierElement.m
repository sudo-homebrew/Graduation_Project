function [data, info] = expFrontierElement
%ExpFrontierElement gives an empty data for adhoc_communication/ExpFrontierElement

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'adhoc_communication/ExpFrontierElement';
[data.Id, info.Id] = ros.internal.ros.messages.ros.default_type('int64',1);
[data.DetectedByRobotStr, info.DetectedByRobotStr] = ros.internal.ros.messages.ros.char('string',0);
[data.DetectedByRobot, info.DetectedByRobot] = ros.internal.ros.messages.ros.default_type('int64',1);
[data.RobotHomePositionX, info.RobotHomePositionX] = ros.internal.ros.messages.ros.default_type('double',1);
[data.RobotHomePositionY, info.RobotHomePositionY] = ros.internal.ros.messages.ros.default_type('double',1);
[data.XCoordinate, info.XCoordinate] = ros.internal.ros.messages.ros.default_type('double',1);
[data.YCoordinate, info.YCoordinate] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'adhoc_communication/ExpFrontierElement';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'id';
info.MatPath{2} = 'detected_by_robot_str';
info.MatPath{3} = 'detected_by_robot';
info.MatPath{4} = 'robot_home_position_x';
info.MatPath{5} = 'robot_home_position_y';
info.MatPath{6} = 'x_coordinate';
info.MatPath{7} = 'y_coordinate';
