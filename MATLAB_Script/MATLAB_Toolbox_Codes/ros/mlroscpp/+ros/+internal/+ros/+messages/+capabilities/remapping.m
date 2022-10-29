function [data, info] = remapping
%Remapping gives an empty data for capabilities/Remapping

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'capabilities/Remapping';
[data.Key, info.Key] = ros.internal.ros.messages.ros.char('string',0);
[data.Value, info.Value] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'capabilities/Remapping';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'key';
info.MatPath{2} = 'value';
