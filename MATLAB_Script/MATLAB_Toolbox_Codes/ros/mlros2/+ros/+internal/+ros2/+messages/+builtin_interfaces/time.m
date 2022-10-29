function [data, info] = time
%Time gives an empty data for builtin_interfaces/Time

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'builtin_interfaces/Time';
[data.sec, info.sec] = ros.internal.ros2.messages.ros2.default_type('int32',1,0);
[data.nanosec, info.nanosec] = ros.internal.ros2.messages.ros2.default_type('uint32',1,0);
info.MessageType = 'builtin_interfaces/Time';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'sec';
info.MatPath{2} = 'nanosec';
