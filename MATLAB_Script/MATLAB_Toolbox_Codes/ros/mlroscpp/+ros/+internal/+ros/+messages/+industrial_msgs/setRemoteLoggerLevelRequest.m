function [data, info] = setRemoteLoggerLevelRequest
%SetRemoteLoggerLevel gives an empty data for industrial_msgs/SetRemoteLoggerLevelRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'industrial_msgs/SetRemoteLoggerLevelRequest';
[data.Level, info.Level] = ros.internal.ros.messages.industrial_msgs.debugLevel;
info.Level.MLdataType = 'struct';
info.MessageType = 'industrial_msgs/SetRemoteLoggerLevelRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'level';
info.MatPath{2} = 'level.val';
info.MatPath{3} = 'level.DEBUG';
info.MatPath{4} = 'level.INFO';
info.MatPath{5} = 'level.WARN';
info.MatPath{6} = 'level.ERROR';
info.MatPath{7} = 'level.FATAL';
info.MatPath{8} = 'level.NONE';
