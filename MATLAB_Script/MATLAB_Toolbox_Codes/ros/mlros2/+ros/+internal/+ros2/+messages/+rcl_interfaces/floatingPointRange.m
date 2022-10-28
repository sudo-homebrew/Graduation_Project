function [data, info] = floatingPointRange
%FloatingPointRange gives an empty data for rcl_interfaces/FloatingPointRange

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rcl_interfaces/FloatingPointRange';
[data.from_value, info.from_value] = ros.internal.ros2.messages.ros2.default_type('double',1,0);
[data.to_value, info.to_value] = ros.internal.ros2.messages.ros2.default_type('double',1,0);
[data.step, info.step] = ros.internal.ros2.messages.ros2.default_type('double',1,0);
info.MessageType = 'rcl_interfaces/FloatingPointRange';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'from_value';
info.MatPath{2} = 'to_value';
info.MatPath{3} = 'step';
