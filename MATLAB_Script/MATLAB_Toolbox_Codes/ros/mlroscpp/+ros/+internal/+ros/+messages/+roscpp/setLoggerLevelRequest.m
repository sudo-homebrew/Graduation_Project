function [data, info] = setLoggerLevelRequest
%SetLoggerLevel gives an empty data for roscpp/SetLoggerLevelRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'roscpp/SetLoggerLevelRequest';
[data.Logger, info.Logger] = ros.internal.ros.messages.ros.char('string',0);
[data.Level, info.Level] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'roscpp/SetLoggerLevelRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'logger';
info.MatPath{2} = 'level';
