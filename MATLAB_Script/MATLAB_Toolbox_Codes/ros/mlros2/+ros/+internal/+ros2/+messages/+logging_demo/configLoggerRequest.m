function [data, info] = configLoggerRequest
%ConfigLogger gives an empty data for logging_demo/ConfigLoggerRequest

% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'logging_demo/ConfigLoggerRequest';
[data.logger_name, info.logger_name] = ros.internal.ros2.messages.ros2.char('string',1,NaN,0);
[data.level, info.level] = ros.internal.ros2.messages.ros2.char('string',1,NaN,0);
info.MessageType = 'logging_demo/ConfigLoggerRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'logger_name';
info.MatPath{2} = 'level';
