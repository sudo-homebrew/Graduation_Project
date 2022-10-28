function [data, info] = listRobotStatesInWarehouseResponse
%ListRobotStatesInWarehouse gives an empty data for moveit_msgs/ListRobotStatesInWarehouseResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'moveit_msgs/ListRobotStatesInWarehouseResponse';
[data.States, info.States] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'moveit_msgs/ListRobotStatesInWarehouseResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'states';
