function [data, info] = joy
%Joy gives an empty data for sensor_msgs/Joy

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sensor_msgs/Joy';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.axes, info.axes] = ros.internal.ros2.messages.ros2.default_type('single',NaN,0);
[data.buttons, info.buttons] = ros.internal.ros2.messages.ros2.default_type('int32',NaN,0);
info.MessageType = 'sensor_msgs/Joy';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.stamp';
info.MatPath{3} = 'header.stamp.sec';
info.MatPath{4} = 'header.stamp.nanosec';
info.MatPath{5} = 'header.frame_id';
info.MatPath{6} = 'axes';
info.MatPath{7} = 'buttons';
