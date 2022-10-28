function [data, info] = listParametersResult
%ListParametersResult gives an empty data for rcl_interfaces/ListParametersResult

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rcl_interfaces/ListParametersResult';
[data.names, info.names] = ros.internal.ros2.messages.ros2.char('string',NaN,NaN,0);
[data.prefixes, info.prefixes] = ros.internal.ros2.messages.ros2.char('string',NaN,NaN,0);
info.MessageType = 'rcl_interfaces/ListParametersResult';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'names';
info.MatPath{2} = 'prefixes';
