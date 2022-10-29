function [data, info] = cliffEvent
%CliffEvent gives an empty data for kobuki_msgs/CliffEvent

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'kobuki_msgs/CliffEvent';
[data.LEFT, info.LEFT] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.CENTER, info.CENTER] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.RIGHT, info.RIGHT] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.FLOOR, info.FLOOR] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.CLIFF, info.CLIFF] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.Sensor, info.Sensor] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.State, info.State] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.Bottom, info.Bottom] = ros.internal.ros.messages.ros.default_type('uint16',1);
info.MessageType = 'kobuki_msgs/CliffEvent';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'LEFT';
info.MatPath{2} = 'CENTER';
info.MatPath{3} = 'RIGHT';
info.MatPath{4} = 'FLOOR';
info.MatPath{5} = 'CLIFF';
info.MatPath{6} = 'sensor';
info.MatPath{7} = 'state';
info.MatPath{8} = 'bottom';
