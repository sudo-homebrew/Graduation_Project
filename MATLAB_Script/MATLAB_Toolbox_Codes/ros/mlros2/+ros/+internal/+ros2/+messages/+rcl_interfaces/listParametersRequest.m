function [data, info] = listParametersRequest
%ListParameters gives an empty data for rcl_interfaces/ListParametersRequest

% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'rcl_interfaces/ListParametersRequest';
[data.DEPTH_RECURSIVE, info.DEPTH_RECURSIVE] = ros.internal.ros2.messages.ros2.default_type('uint64',1,0, 0, [NaN]);
[data.prefixes, info.prefixes] = ros.internal.ros2.messages.ros2.char('string',NaN,NaN,0);
[data.depth, info.depth] = ros.internal.ros2.messages.ros2.default_type('uint64',1,0);
info.MessageType = 'rcl_interfaces/ListParametersRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'DEPTH_RECURSIVE';
info.MatPath{2} = 'prefixes';
info.MatPath{3} = 'depth';
