function [data, info] = renameRobotStateInWarehouseRequest
%RenameRobotStateInWarehouse gives an empty data for moveit_msgs/RenameRobotStateInWarehouseRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'moveit_msgs/RenameRobotStateInWarehouseRequest';
[data.OldName, info.OldName] = ros.internal.ros.messages.ros.char('string',0);
[data.NewName, info.NewName] = ros.internal.ros.messages.ros.char('string',0);
[data.Robot, info.Robot] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'moveit_msgs/RenameRobotStateInWarehouseRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'old_name';
info.MatPath{2} = 'new_name';
info.MatPath{3} = 'robot';
