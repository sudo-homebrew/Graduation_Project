function [data, info] = whiteBalanceResponse
%WhiteBalance gives an empty data for jsk_perception/WhiteBalanceResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_perception/WhiteBalanceResponse';
[data.Output, info.Output] = ros.internal.ros.messages.sensor_msgs.image;
info.Output.MLdataType = 'struct';
info.MessageType = 'jsk_perception/WhiteBalanceResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,13);
info.MatPath{1} = 'output';
info.MatPath{2} = 'output.header';
info.MatPath{3} = 'output.header.seq';
info.MatPath{4} = 'output.header.stamp';
info.MatPath{5} = 'output.header.stamp.sec';
info.MatPath{6} = 'output.header.stamp.nsec';
info.MatPath{7} = 'output.header.frame_id';
info.MatPath{8} = 'output.height';
info.MatPath{9} = 'output.width';
info.MatPath{10} = 'output.encoding';
info.MatPath{11} = 'output.is_bigendian';
info.MatPath{12} = 'output.step';
info.MatPath{13} = 'output.data';
