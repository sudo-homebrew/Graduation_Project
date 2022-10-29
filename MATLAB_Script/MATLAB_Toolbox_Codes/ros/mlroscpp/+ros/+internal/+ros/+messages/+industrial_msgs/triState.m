function [data, info] = triState
%TriState gives an empty data for industrial_msgs/TriState

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'industrial_msgs/TriState';
[data.Val, info.Val] = ros.internal.ros.messages.ros.default_type('int8',1);
[data.UNKNOWN, info.UNKNOWN] = ros.internal.ros.messages.ros.default_type('int8',1, -1);
[data.TRUE, info.TRUE] = ros.internal.ros.messages.ros.default_type('int8',1, 1);
[data.ON, info.ON] = ros.internal.ros.messages.ros.default_type('int8',1, 1);
[data.ENABLED, info.ENABLED] = ros.internal.ros.messages.ros.default_type('int8',1, 1);
[data.HIGH, info.HIGH] = ros.internal.ros.messages.ros.default_type('int8',1, 1);
[data.CLOSED, info.CLOSED] = ros.internal.ros.messages.ros.default_type('int8',1, 1);
[data.FALSE, info.FALSE] = ros.internal.ros.messages.ros.default_type('int8',1, 0);
[data.OFF, info.OFF] = ros.internal.ros.messages.ros.default_type('int8',1, 0);
[data.DISABLED, info.DISABLED] = ros.internal.ros.messages.ros.default_type('int8',1, 0);
[data.LOW, info.LOW] = ros.internal.ros.messages.ros.default_type('int8',1, 0);
[data.OPEN, info.OPEN] = ros.internal.ros.messages.ros.default_type('int8',1, 0);
info.MessageType = 'industrial_msgs/TriState';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'val';
info.MatPath{2} = 'UNKNOWN';
info.MatPath{3} = 'TRUE';
info.MatPath{4} = 'ON';
info.MatPath{5} = 'ENABLED';
info.MatPath{6} = 'HIGH';
info.MatPath{7} = 'CLOSED';
info.MatPath{8} = 'FALSE';
info.MatPath{9} = 'OFF';
info.MatPath{10} = 'DISABLED';
info.MatPath{11} = 'LOW';
info.MatPath{12} = 'OPEN';
