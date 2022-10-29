function [data, info] = checkIfRobotStateExistsInWarehouseRequest
%CheckIfRobotStateExistsInWarehouse gives an empty data for moveit_msgs/CheckIfRobotStateExistsInWarehouseRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'moveit_msgs/CheckIfRobotStateExistsInWarehouseRequest';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.Robot, info.Robot] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'moveit_msgs/CheckIfRobotStateExistsInWarehouseRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'name';
info.MatPath{2} = 'robot';
