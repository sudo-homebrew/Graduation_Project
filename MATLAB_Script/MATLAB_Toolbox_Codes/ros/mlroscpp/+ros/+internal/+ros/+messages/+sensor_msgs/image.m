function [data, info] = image
%Image gives an empty data for sensor_msgs/Image

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sensor_msgs/Image';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Height, info.Height] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Width, info.Width] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Encoding, info.Encoding] = ros.internal.ros.messages.ros.char('string',0);
[data.IsBigendian, info.IsBigendian] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.Step, info.Step] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Data, info.Data] = ros.internal.ros.messages.ros.default_type('uint8',NaN);
info.MessageType = 'sensor_msgs/Image';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'height';
info.MatPath{8} = 'width';
info.MatPath{9} = 'encoding';
info.MatPath{10} = 'is_bigendian';
info.MatPath{11} = 'step';
info.MatPath{12} = 'data';
