function [data, info] = setParametersResult
%SetParametersResult gives an empty data for rcl_interfaces/SetParametersResult

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rcl_interfaces/SetParametersResult';
[data.successful, info.successful] = ros.internal.ros2.messages.ros2.default_type('logical',1,0);
[data.reason, info.reason] = ros.internal.ros2.messages.ros2.char('string',1,NaN,0);
info.MessageType = 'rcl_interfaces/SetParametersResult';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'successful';
info.MatPath{2} = 'reason';
