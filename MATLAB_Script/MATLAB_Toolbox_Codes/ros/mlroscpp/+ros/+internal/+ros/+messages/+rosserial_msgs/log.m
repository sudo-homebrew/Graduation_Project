function [data, info] = log
%Log gives an empty data for rosserial_msgs/Log

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rosserial_msgs/Log';
[data.ROSDEBUG, info.ROSDEBUG] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.INFO, info.INFO] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.WARN, info.WARN] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.ERROR, info.ERROR] = ros.internal.ros.messages.ros.default_type('uint8',1, 3);
[data.FATAL, info.FATAL] = ros.internal.ros.messages.ros.default_type('uint8',1, 4);
[data.Level, info.Level] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.Msg, info.Msg] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'rosserial_msgs/Log';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'ROSDEBUG';
info.MatPath{2} = 'INFO';
info.MatPath{3} = 'WARN';
info.MatPath{4} = 'ERROR';
info.MatPath{5} = 'FATAL';
info.MatPath{6} = 'level';
info.MatPath{7} = 'msg';
