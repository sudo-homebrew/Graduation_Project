function [data, info] = joy
%Joy gives an empty data for sensor_msgs/Joy

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sensor_msgs/Joy';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Axes, info.Axes] = ros.internal.ros.messages.ros.default_type('single',NaN);
[data.Buttons, info.Buttons] = ros.internal.ros.messages.ros.default_type('int32',NaN);
info.MessageType = 'sensor_msgs/Joy';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'axes';
info.MatPath{8} = 'buttons';
