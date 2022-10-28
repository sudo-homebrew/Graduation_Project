function [data, info] = elevatorAction
%ElevatorAction gives an empty data for robotnik_msgs/ElevatorAction

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/ElevatorAction';
[data.RAISE, info.RAISE] = ros.internal.ros.messages.ros.default_type('int32',1, 1);
[data.LOWER, info.LOWER] = ros.internal.ros.messages.ros.default_type('int32',1, -1);
[data.STOP, info.STOP] = ros.internal.ros.messages.ros.default_type('int32',1, 0);
[data.NOACTION, info.NOACTION] = ros.internal.ros.messages.ros.default_type('int32',1, 1000);
[data.Action, info.Action] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'robotnik_msgs/ElevatorAction';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'RAISE';
info.MatPath{2} = 'LOWER';
info.MatPath{3} = 'STOP';
info.MatPath{4} = 'NO_ACTION';
info.MatPath{5} = 'action';
