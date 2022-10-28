function [data, info] = logger
%Logger gives an empty data for roscpp/Logger

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'roscpp/Logger';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.Level, info.Level] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'roscpp/Logger';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'name';
info.MatPath{2} = 'level';
