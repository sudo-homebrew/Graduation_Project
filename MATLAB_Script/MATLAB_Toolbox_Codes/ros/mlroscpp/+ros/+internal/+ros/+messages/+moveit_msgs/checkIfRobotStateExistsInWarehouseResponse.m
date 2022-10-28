function [data, info] = checkIfRobotStateExistsInWarehouseResponse
%CheckIfRobotStateExistsInWarehouse gives an empty data for moveit_msgs/CheckIfRobotStateExistsInWarehouseResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'moveit_msgs/CheckIfRobotStateExistsInWarehouseResponse';
[data.Exists, info.Exists] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'moveit_msgs/CheckIfRobotStateExistsInWarehouseResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'exists';
