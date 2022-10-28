function [data, info] = robotStateEvent
%RobotStateEvent gives an empty data for kobuki_msgs/RobotStateEvent

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'kobuki_msgs/RobotStateEvent';
[data.ONLINE, info.ONLINE] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.OFFLINE, info.OFFLINE] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.State, info.State] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'kobuki_msgs/RobotStateEvent';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'ONLINE';
info.MatPath{2} = 'OFFLINE';
info.MatPath{3} = 'state';
