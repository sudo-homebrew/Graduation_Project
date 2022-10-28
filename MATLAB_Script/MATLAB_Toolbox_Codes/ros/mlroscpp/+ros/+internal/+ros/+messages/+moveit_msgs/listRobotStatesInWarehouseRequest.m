function [data, info] = listRobotStatesInWarehouseRequest
%ListRobotStatesInWarehouse gives an empty data for moveit_msgs/ListRobotStatesInWarehouseRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'moveit_msgs/ListRobotStatesInWarehouseRequest';
[data.Regex, info.Regex] = ros.internal.ros.messages.ros.char('string',0);
[data.Robot, info.Robot] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'moveit_msgs/ListRobotStatesInWarehouseRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'regex';
info.MatPath{2} = 'robot';
