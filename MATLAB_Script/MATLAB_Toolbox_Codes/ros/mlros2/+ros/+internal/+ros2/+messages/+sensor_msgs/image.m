function [data, info] = image
%Image gives an empty data for sensor_msgs/Image

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sensor_msgs/Image';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.height, info.height] = ros.internal.ros2.messages.ros2.default_type('uint32',1,0);
[data.width, info.width] = ros.internal.ros2.messages.ros2.default_type('uint32',1,0);
[data.encoding, info.encoding] = ros.internal.ros2.messages.ros2.char('string',1,NaN,0);
[data.is_bigendian, info.is_bigendian] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0);
[data.step, info.step] = ros.internal.ros2.messages.ros2.default_type('uint32',1,0);
[data.data, info.data] = ros.internal.ros2.messages.ros2.default_type('uint8',NaN,0);
info.MessageType = 'sensor_msgs/Image';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,11);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.stamp';
info.MatPath{3} = 'header.stamp.sec';
info.MatPath{4} = 'header.stamp.nanosec';
info.MatPath{5} = 'header.frame_id';
info.MatPath{6} = 'height';
info.MatPath{7} = 'width';
info.MatPath{8} = 'encoding';
info.MatPath{9} = 'is_bigendian';
info.MatPath{10} = 'step';
info.MatPath{11} = 'data';
