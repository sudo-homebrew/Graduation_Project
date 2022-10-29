function [data, info] = serviceReturnCode
%ServiceReturnCode gives an empty data for industrial_msgs/ServiceReturnCode

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'industrial_msgs/ServiceReturnCode';
[data.Val, info.Val] = ros.internal.ros.messages.ros.default_type('int8',1);
[data.SUCCESS, info.SUCCESS] = ros.internal.ros.messages.ros.default_type('int8',1, 1);
[data.FAILURE, info.FAILURE] = ros.internal.ros.messages.ros.default_type('int8',1, -1);
info.MessageType = 'industrial_msgs/ServiceReturnCode';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'val';
info.MatPath{2} = 'SUCCESS';
info.MatPath{3} = 'FAILURE';
