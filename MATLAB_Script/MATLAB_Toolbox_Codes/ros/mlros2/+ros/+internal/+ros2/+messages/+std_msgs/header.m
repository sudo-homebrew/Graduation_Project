function [data, info] = header
%Header gives an empty data for std_msgs/Header

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'std_msgs/Header';
[data.stamp, info.stamp] = ros.internal.ros2.messages.builtin_interfaces.time;
info.stamp.MLdataType = 'struct';
[data.frame_id, info.frame_id] = ros.internal.ros2.messages.ros2.char('string',1,NaN,0);
info.MessageType = 'std_msgs/Header';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'stamp';
info.MatPath{2} = 'stamp.sec';
info.MatPath{3} = 'stamp.nanosec';
info.MatPath{4} = 'frame_id';
