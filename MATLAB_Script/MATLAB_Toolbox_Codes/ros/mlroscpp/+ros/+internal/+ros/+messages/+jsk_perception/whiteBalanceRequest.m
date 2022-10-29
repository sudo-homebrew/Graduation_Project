function [data, info] = whiteBalanceRequest
%WhiteBalance gives an empty data for jsk_perception/WhiteBalanceRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_perception/WhiteBalanceRequest';
[data.ReferenceColor, info.ReferenceColor] = ros.internal.ros.messages.ros.default_type('single',3);
[data.Input, info.Input] = ros.internal.ros.messages.sensor_msgs.image;
info.Input.MLdataType = 'struct';
info.MessageType = 'jsk_perception/WhiteBalanceRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,14);
info.MatPath{1} = 'reference_color';
info.MatPath{2} = 'input';
info.MatPath{3} = 'input.header';
info.MatPath{4} = 'input.header.seq';
info.MatPath{5} = 'input.header.stamp';
info.MatPath{6} = 'input.header.stamp.sec';
info.MatPath{7} = 'input.header.stamp.nsec';
info.MatPath{8} = 'input.header.frame_id';
info.MatPath{9} = 'input.height';
info.MatPath{10} = 'input.width';
info.MatPath{11} = 'input.encoding';
info.MatPath{12} = 'input.is_bigendian';
info.MatPath{13} = 'input.step';
info.MatPath{14} = 'input.data';
