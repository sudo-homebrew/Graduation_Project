function [data, info] = integerRange
%IntegerRange gives an empty data for rcl_interfaces/IntegerRange

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rcl_interfaces/IntegerRange';
[data.from_value, info.from_value] = ros.internal.ros2.messages.ros2.default_type('int64',1,0);
[data.to_value, info.to_value] = ros.internal.ros2.messages.ros2.default_type('int64',1,0);
[data.step, info.step] = ros.internal.ros2.messages.ros2.default_type('uint64',1,0);
info.MessageType = 'rcl_interfaces/IntegerRange';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'from_value';
info.MatPath{2} = 'to_value';
info.MatPath{3} = 'step';
