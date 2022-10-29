function [data, info] = stringStringResponse
%StringString gives an empty data for roseus/StringStringResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'roseus/StringStringResponse';
[data.Str, info.Str] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'roseus/StringStringResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'str';
