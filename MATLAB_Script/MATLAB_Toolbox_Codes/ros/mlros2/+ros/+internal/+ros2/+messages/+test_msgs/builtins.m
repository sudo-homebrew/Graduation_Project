function [data, info] = builtins
%Builtins gives an empty data for test_msgs/Builtins

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'test_msgs/Builtins';
[data.duration_value, info.duration_value] = ros.internal.ros2.messages.builtin_interfaces.duration;
info.duration_value.MLdataType = 'struct';
[data.time_value, info.time_value] = ros.internal.ros2.messages.builtin_interfaces.time;
info.time_value.MLdataType = 'struct';
info.MessageType = 'test_msgs/Builtins';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'duration_value';
info.MatPath{2} = 'duration_value.sec';
info.MatPath{3} = 'duration_value.nanosec';
info.MatPath{4} = 'time_value';
info.MatPath{5} = 'time_value.sec';
info.MatPath{6} = 'time_value.nanosec';
