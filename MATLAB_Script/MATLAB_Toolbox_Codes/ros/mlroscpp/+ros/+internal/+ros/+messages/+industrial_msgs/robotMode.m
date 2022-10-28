function [data, info] = robotMode
%RobotMode gives an empty data for industrial_msgs/RobotMode

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'industrial_msgs/RobotMode';
[data.Val, info.Val] = ros.internal.ros.messages.ros.default_type('int8',1);
[data.UNKNOWN, info.UNKNOWN] = ros.internal.ros.messages.ros.default_type('int8',1, -1);
[data.MANUAL, info.MANUAL] = ros.internal.ros.messages.ros.default_type('int8',1, 1);
[data.AUTO, info.AUTO] = ros.internal.ros.messages.ros.default_type('int8',1, 2);
info.MessageType = 'industrial_msgs/RobotMode';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'val';
info.MatPath{2} = 'UNKNOWN';
info.MatPath{3} = 'MANUAL';
info.MatPath{4} = 'AUTO';
