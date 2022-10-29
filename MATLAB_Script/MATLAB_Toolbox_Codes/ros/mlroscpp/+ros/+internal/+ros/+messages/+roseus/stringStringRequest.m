function [data, info] = stringStringRequest
%StringString gives an empty data for roseus/StringStringRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'roseus/StringStringRequest';
[data.Str, info.Str] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'roseus/StringStringRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'str';
