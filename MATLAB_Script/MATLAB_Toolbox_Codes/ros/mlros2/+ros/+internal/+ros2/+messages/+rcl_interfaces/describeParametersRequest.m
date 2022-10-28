function [data, info] = describeParametersRequest
%DescribeParameters gives an empty data for rcl_interfaces/DescribeParametersRequest

% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'rcl_interfaces/DescribeParametersRequest';
[data.names, info.names] = ros.internal.ros2.messages.ros2.char('string',NaN,NaN,0);
info.MessageType = 'rcl_interfaces/DescribeParametersRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'names';
