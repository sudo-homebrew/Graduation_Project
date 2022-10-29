function [data, info] = debugLevel
%DebugLevel gives an empty data for industrial_msgs/DebugLevel

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'industrial_msgs/DebugLevel';
[data.Val, info.Val] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.DEBUG, info.DEBUG] = ros.internal.ros.messages.ros.default_type('uint8',1, 5);
[data.INFO, info.INFO] = ros.internal.ros.messages.ros.default_type('uint8',1, 4);
[data.WARN, info.WARN] = ros.internal.ros.messages.ros.default_type('uint8',1, 3);
[data.ERROR, info.ERROR] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.FATAL, info.FATAL] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.NONE, info.NONE] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
info.MessageType = 'industrial_msgs/DebugLevel';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'val';
info.MatPath{2} = 'DEBUG';
info.MatPath{3} = 'INFO';
info.MatPath{4} = 'WARN';
info.MatPath{5} = 'ERROR';
info.MatPath{6} = 'FATAL';
info.MatPath{7} = 'NONE';
